from typing import Dict
from dataclasses import dataclass
from abc import ABC, abstractmethod

import numpy as np
import yaml

import casadi as ca
from acados_template import AcadosModel


@dataclass
class ModelConfig:
    """
    Base configuration dataclass for vehicle
    The function load_model_config can be used to load the configuration from a yaml file

    """  

    model_name: str

    mass: float
    radius: float
    wheel_radius: float
    wheel_base: float

    lin_friction_coeff: float
    ang_friction_coeff: float
    inertia: Dict[str, float]

    v_max: float
    omega_max: float
    wheel_speed_max: float
    a_max: float
    tau_max: float
    safety_radius: float

    nx: int = 5
    nu: int = 2

    def load_model_config(file_path: str):

        with open(file_path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)
        
        robot = data["robot"]
        nx = int(robot.get("nx", ModelConfig.__dataclass_fields__["nx"].default))
        nu = int(robot.get("nu", ModelConfig.__dataclass_fields__["nu"].default))
        
        robot = data["robot"]
        robot_name = robot["name"]
        robot_type = robot["type"]
        model_name = f"{robot_name}_{robot_type}"

        inertia = {
            "x": float(robot["inertia"]["Ixx"]),
            "y": float(robot["inertia"]["Iyy"]),
            "z": float(robot["inertia"]["Izz"]),
            "xy": float(robot["inertia"].get("Ixy", 0.0)),
            "xz": float(robot["inertia"].get("Ixz", 0.0)),
            "yz": float(robot["inertia"].get("Iyz", 0.0)),
        }

        config = ModelConfig(
            model_name=model_name,
        
            mass=float(robot["mass"]),
            radius=float(robot["radius"]),
            wheel_radius=float(robot["wheel_radius"]),
            wheel_base=float(robot["wheel_base"]),

            lin_friction_coeff=float(robot["linear_friction"]),
            ang_friction_coeff=float(robot["angular_friction"]),
            inertia=inertia,

            v_max=float(robot["max_linear_velocity"]),
            omega_max=float(robot["max_angular_velocity"]),
            wheel_speed_max=float(robot["max_wheel_angular_velocity"]),
            a_max=float(robot["a_max"]),
            tau_max=float(robot["tau_max"]),
            safety_radius=float(robot["safety_radius"]),

            nx=nx,
            nu=nu
        )
        return config


class VehicleModel(ABC):
    ## Base model class for definining the different vehicle dynamics.
    ## Config is a instance of the ModelConfig dataclass containing all the necessary parameters to define the dynamics, as well as the model name and robot type.
    def __init__(self, config: ModelConfig):
        self.config = config
        self.name = config.model_name
        self.nx = config.nx
        self.nu = config.nu
        return
    
    @property
    def rt_param_n(self) -> int:
        """
        Number of runtime model parameters.

        Nonlinear model:
            np = 0

        Linearized model:
            np = nx + nu
            p = [x_lin, u_lin]
        """
        return 0
    
    def get_runtime_parameters(
        self,
        x_lin: np.ndarray,
        u_lin: np.ndarray | None = None,
    ) -> np.ndarray | None:
        """
        Return the runtime parameter vector for the current solve.

        Nonlinear models return None.
        Linearized models override this.
        """
        return None

    @abstractmethod
    def build_acados_model(self):
        """
        Build and return an acados model.

        Must define:
            model.x
            model.u
            model.xdot
            model.f_expl_expr
            model.f_impl_expr
        """
        pass

class LinearDynamicsModel(VehicleModel):
    """
    Returns the linearized dynamics for the differential drive robot around a runtime linearization point (x_lin, u_lin) provided as parameters. 
    Inertia J should be provided as a dictionary in the config with the key "z"
    """
    
    @property
    def rt_param_n(self) -> int:
        return self.nx + self.nu
    
    def get_runtime_parameters(
        self,
        x_lin: np.ndarray,
        u_lin: np.ndarray | None = None,
    ) -> np.ndarray:
        x_lin = np.asarray(x_lin, dtype=float).reshape(self.nx)

        if u_lin is None:
            u_lin = np.zeros(self.nu)
        else:
            u_lin = np.asarray(u_lin, dtype=float).reshape(self.nu)

        return np.concatenate([x_lin, u_lin])
    
    def build_acados_model(self) -> AcadosModel:
        # States
        px = ca.SX.sym("px")
        py = ca.SX.sym("py")
        theta = ca.SX.sym("theta")
        v = ca.SX.sym("v")
        omega = ca.SX.sym("omega")

        x = ca.vertcat(px, py, theta, v, omega)

        # Inputs
        tau_l = ca.SX.sym("tau_l")
        tau_r = ca.SX.sym("tau_r")

        u = ca.vertcat(tau_l, tau_r)
        
        # Runtime linearization point
        px0 = ca.SX.sym("px_0")
        py0 = ca.SX.sym("py_0")
        theta0 = ca.SX.sym("theta_0")
        v0 = ca.SX.sym("v_0")
        omega0 = ca.SX.sym("omega_0")

        x0 = ca.vertcat(px0, py0, theta0, v0, omega0)

        tau_l0 = ca.SX.sym("tau_l_0")
        tau_r0 = ca.SX.sym("tau_r_0")

        u0 = ca.vertcat(tau_l0, tau_r0)

        p = ca.vertcat(
            px0,
            py0,
            theta0,
            v0,
            omega0,
            tau_l0,
            tau_r0,
        )

        # State derivatives
        px_dot = ca.SX.sym("px_dot")
        py_dot = ca.SX.sym("py_dot")
        theta_dot = ca.SX.sym("theta_dot")
        v_dot = ca.SX.sym("v_dot")
        omega_dot = ca.SX.sym("omega_dot")

        xdot = ca.vertcat(px_dot, py_dot, theta_dot, v_dot, omega_dot)

        A = ca.SX.zeros(5, 5)
        A[0, 2] = -v0 * ca.sin(theta0)
        A[0, 3] = ca.cos(theta0)
        A[1, 2] = v0 * ca.cos(theta0)
        A[1, 3] = ca.sin(theta0)
        A[2, 4] = 1.0
        A[3, 3] = -self.config.lin_friction_coeff / self.config.mass
        A[4, 4] = -self.config.ang_friction_coeff / self.config.inertia["z"]

        B = ca.SX.zeros(5, 2)
        B[3, 0] = 1.0 / (self.config.mass * self.config.wheel_radius)
        B[3, 1] = 1.0 / (self.config.mass * self.config.wheel_radius)
        B[4, 0] = -self.config.wheel_base / (2.0 * self.config.wheel_radius * self.config.inertia["z"])
        B[4, 1] = self.config.wheel_base / (2.0 * self.config.wheel_radius * self.config.inertia["z"])

        force_l0 = tau_l0 / self.config.wheel_radius
        force_r0 = tau_r0 / self.config.wheel_radius

        force_total0 = force_l0 + force_r0
        yaw_moment0 = 0.5 * self.config.wheel_base * (force_r0 - force_l0)

        f0 = ca.vertcat(
            v0 * ca.cos(theta0),
            v0 * ca.sin(theta0),
            omega0,
            force_total0 / self.config.mass - self.config.lin_friction_coeff * v0 / self.config.mass,
            yaw_moment0 / self.config.inertia["z"] - self.config.ang_friction_coeff * omega0 / self.config.inertia["z"],
        )

        f_expl = f0 + A @ (x - x0) + B @ (u - u0)
        f_impl = xdot - f_expl

        model = AcadosModel()
        model.name = self.name
        model.x = x
        model.u = u
        model.p = p
        model.xdot = xdot
        model.f_expl_expr = f_expl
        model.f_impl_expr = f_impl

        return model


class NonLinearDynamicsModel(VehicleModel):
    """
    Returns the nonlinear dynamics for the differential drive robot.
    Inertia J should be provided as a dictionary in the config with the key "z"
    """
    
    def build_acados_model(self) -> AcadosModel:
        # States
        px = ca.SX.sym("px")
        py = ca.SX.sym("py")
        theta = ca.SX.sym("theta")
        v = ca.SX.sym("v")
        omega = ca.SX.sym("omega")

        x = ca.vertcat(px, py, theta, v, omega)

        # Inputs: wheel torques
        tau_l = ca.SX.sym("tau_l")
        tau_r = ca.SX.sym("tau_r")
        u = ca.vertcat(tau_l, tau_r)

        # State derivatives
        px_dot = ca.SX.sym("px_dot")
        py_dot = ca.SX.sym("py_dot")
        theta_dot = ca.SX.sym("theta_dot")
        v_dot = ca.SX.sym("v_dot")
        omega_dot = ca.SX.sym("omega_dot")

        xdot = ca.vertcat(px_dot, py_dot, theta_dot, v_dot, omega_dot)

        force_l = tau_l / self.config.wheel_radius
        force_r = tau_r / self.config.wheel_radius

        force_total = force_l + force_r
        yaw_moment = 0.5 * self.config.wheel_base * (force_r - force_l)

        f_expl = ca.vertcat(
            v * ca.cos(theta),
            v * ca.sin(theta),
            omega,
            force_total / self.config.mass - self.config.lin_friction_coeff * v / self.config.mass,
            yaw_moment / self.config.inertia["z"] - self.config.ang_friction_coeff * omega / self.config.inertia["z"],
        )


        model = AcadosModel()
        model.name = self.name
        model.x = x
        model.u = u
        model.xdot = xdot
        model.f_expl_expr = f_expl

        return model
