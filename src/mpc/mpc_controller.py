from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver

from src.mpc.vehicle_models import VehicleModel


# Human-readable acados solver return codes (see acados_template status enum).
ACADOS_STATUS = {
    0: "success",
    1: "NaN detected in solver",
    2: "maximum number of iterations reached",
    3: "minimum step size reached",
    4: "QP solver failed (infeasible / ill-conditioned QP)",
    5: "solver created but not yet solved",
    6: "QP solver failed with an unbounded subproblem",
}


@dataclass
class MPCConfig:
    N: int = 150
    dt: float = 0.1

    # Solver settings
    qp_solver: str = "PARTIAL_CONDENSING_HPIPM"
    nlp_solver_type: str = "SQP_RTI"
    hessian_approx: str = "GAUSS_NEWTON"
    integrator_type: str = "ERK"

    # Levenberg-Marquardt regularization added to the Gauss-Newton Hessian.
    # Smooths the solution between solves (reduces prediction flicker) on
    # lightly-damped / ill-conditioned models. 0.0 disables it.
    levenberg_marquardt: float = 1e-3

    # Useful during early experiments
    print_level: int = 0
    nlp_solver_max_iter: int = 100
    json_file: str = "build/acados_ocp_tracking_diff_drive.json"

    # Directory acados generates C code into. Override per-run so parallel
    # batch builds don't clobber a shared c_generated_code directory.
    code_export_directory: Optional[str] = None

    # Optional diagonal cost weights (lists of floats). When None, the
    # Weights defaults below are used. Otherwise np.diag is applied.
    #   Q, Qe: length nx (x, y, theta, v, omega)
    #   R:     length nu (tau_l, tau_r)
    Q: Optional[list] = None
    R: Optional[list] = None
    Qe: Optional[list] = None


@dataclass
class Weights:
    # Weights for the MPC cost function. These can be tuned to achieve different performance.
    Q = np.diag([
        20.0,   # x
        20.0,   # y
        10.0,    # theta
        10.0,    # v
        10.0,    # omega
    ])
    # positive definite
    R = np.diag([
        0.5,   # tau_l
        0.5,   # tau_r
    ])
    # final state cost
    Qe = np.diag([
        40.0,
        40.0,
        0.0,
        0.0,
        0.0,
    ])


@dataclass
class VehicleConstraints:
    ## Vehicle constraints for the MPC. This includes state and input constraints.
    ## Minimum velocity is 0 since the robot cannot drive backwards, but it can have negative angular velocity.
    def __init__(self, vehicle_model: VehicleModel):
        # State constraints: lower and upper bounds
        self.x_min = np.array([
            -np.inf, 
            -np.inf, 
            -np.inf, 
            0.0, 
            -vehicle_model.config.omega_max
        ])
        self.x_max = np.array([
            np.inf, 
            np.inf, 
            np.inf, 
            vehicle_model.config.v_max, 
            vehicle_model.config.omega_max
        ])

        # Input constraints: lower and upper bounds
        self.u_min = np.array([-vehicle_model.config.tau_max, -vehicle_model.config.tau_max])
        self.u_max = np.array([vehicle_model.config.tau_max, vehicle_model.config.tau_max])

        ### wheel speed max limit unused for now ###


class MPC:
    """
    MPC for any VehicleModel --- for now just tracking MPC
    Args:
    - vehicle_model: an instance of VehicleModel, which defines the dynamics and constraints.
    - config: MPCConfig dataclass instance containing MPC settings.
    Inputs to compute_control():
    - x0: current state, shape (nx,)
    - x_ref_traj: reference state trajectory over the horizon, shape (N + 1, nx)    
    """
    def __init__(
        self,
        vehicle_model: VehicleModel,
        config: MPCConfig,
    ):
        self.vehicle_model = vehicle_model
        self.config = config

        self.nx = vehicle_model.nx
        self.nu = vehicle_model.nu
        self.N = config.N

        self.weights = Weights()
        # Use config-provided diagonal weights when present, else defaults.
        self.Q = np.diag(config.Q) if config.Q is not None else self.weights.Q
        self.R = np.diag(config.R) if config.R is not None else self.weights.R
        self.Qe = np.diag(config.Qe) if config.Qe is not None else self.weights.Qe

        self.constraints = VehicleConstraints(vehicle_model)

        self.ocp = self._build_ocp()
        self.solver = AcadosOcpSolver(self.ocp, json_file=config.json_file)
        self.prev_u_traj = np.zeros((self.N, self.nu))
        self.first_iter = True


    def _build_ocp(self) -> AcadosOcp:
        ocp = AcadosOcp()

        ocp.model = self.vehicle_model.build_acados_model()

        if self.config.code_export_directory is not None:
            ocp.code_export_directory = self.config.code_export_directory

        nx = self.nx
        nu = self.nu
        N = self.N

        ocp.dims.N = N

        if self.vehicle_model.rt_param_n > 0:
            ocp.parameter_values = np.zeros(self.vehicle_model.rt_param_n)

        # Prediction horizon length in seconds
        ocp.solver_options.tf = N * self.config.dt

        # ------------------------------------------------------------------
        # Cost: linear least squares
        #
        # y = [x; u]
        # y_ref = [x_ref; u_ref]
        #
        # terminal y_e = x
        # terminal y_ref_e = x_ref_N
        # ------------------------------------------------------------------

        ny = nx + nu
        ny_e = nx

        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        Vx = np.zeros((ny, nx))
        Vu = np.zeros((ny, nu))

        Vx[:nx, :] = np.eye(nx)
        Vu[nx:nx + nu, :] = np.eye(nu)

        ocp.cost.Vx = Vx
        ocp.cost.Vu = Vu
        ocp.cost.Vx_e = np.eye(nx)

        ocp.cost.W = self._block_diag(self.Q, self.R)
        ocp.cost.W_e = self.Qe

        ocp.cost.yref = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(ny_e)

        # ------------------------------------------------------------------
        # Initial condition
        #
        # This is overwritten every solve() call.
        # ------------------------------------------------------------------

        ocp.constraints.x0 = np.zeros(nx)

        # ------------------------------------------------------------------
        # Static input constraints
        # ------------------------------------------------------------------

        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbu = self.constraints.u_min
        ocp.constraints.ubu = self.constraints.u_max

        # ------------------------------------------------------------------
        # Static state constraints
        # ------------------------------------------------------------------

        constrained_state_idx = self._finite_box_indices(
            self.constraints.x_min,
            self.constraints.x_max,
        )

        if constrained_state_idx.size > 0:
            ocp.constraints.idxbx = constrained_state_idx
            ocp.constraints.lbx = self.constraints.x_min[constrained_state_idx]
            ocp.constraints.ubx = self.constraints.x_max[constrained_state_idx]

        # ------------------------------------------------------------------
        # Solver options
        # ------------------------------------------------------------------

        ocp.solver_options.qp_solver = self.config.qp_solver
        ocp.solver_options.nlp_solver_type = self.config.nlp_solver_type
        ocp.solver_options.hessian_approx = self.config.hessian_approx
        ocp.solver_options.integrator_type = self.config.integrator_type
        ocp.solver_options.print_level = self.config.print_level
        ocp.solver_options.nlp_solver_max_iter = self.config.nlp_solver_max_iter

        if self.config.levenberg_marquardt > 0.0:
            ocp.solver_options.levenberg_marquardt = self.config.levenberg_marquardt

        return ocp


    def compute_control(
        self,
        x0: np.ndarray,
        x_ref_traj: np.ndarray,
        use_prev_u: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Solve MPC.

        Args:
            x0:
                Current state, shape (nx,)
            x_ref_traj:
                Reference states over the horizon, shape (N + 1, nx)

        Returns:
            u0:
                First control action, shape (nu,)
            x_pred:
                Predicted state trajectory, shape (N + 1, nx)
        """

        x0 = np.asarray(x0, dtype=float).reshape(self.nx)
        x_ref_traj = np.asarray(x_ref_traj, dtype=float)[:self.N + 1, :]

        # Set control action reference
        if use_prev_u and not self.first_iter:
            u_ref_traj = self._shift_u_traj(self.prev_u_traj)
        else:
            u_ref_traj = np.zeros((self.N, self.nu))

        # Set initial condition
        self.solver.set(0, "lbx", x0)
        self.solver.set(0, "ubx", x0)

        # Runtime parameters for the linearized model.
        #
        # Linearize each stage around the reference state/input at that stage
        # (linearization along the reference) instead of around the current
        # state held constant over the whole horizon. 
        if self.vehicle_model.rt_param_n > 0:
            for k in range(self.N):
                p_k = self.vehicle_model.get_runtime_parameters(
                    x_lin=x_ref_traj[k],
                    u_lin=u_ref_traj[k],
                )
                self.solver.set(k, "p", p_k)

        # Set stage references
        for k in range(self.N):
            yref = np.concatenate([
                x_ref_traj[k],
                u_ref_traj[k],
            ])
            self.solver.set(k, "yref", yref)

        # Set terminal reference.
        self.solver.set(self.N, "yref", x_ref_traj[self.N])

        # Use previous solution as initial guess
        if use_prev_u and not self.first_iter:
            for k in range(self.N):
                self.solver.set(k, "u", u_ref_traj[k])

        status = self.solver.solve()

        if status != 0:
            raise RuntimeError(
                f"acados solver failed with status {status} "
                f"({ACADOS_STATUS.get(status, 'unknown status')})"
            )

        u0 = self.solver.get(0, "u")

        self.prev_u_traj = np.zeros((self.N, self.nu))
        for k in range(self.N):
            self.prev_u_traj[k, :] = self.solver.get(k, "u")

        x_pred = np.zeros((self.N + 1, self.nx))
        for k in range(self.N + 1):
            x_pred[k, :] = self.solver.get(k, "x")
        
        cost_value = self.solver.get_cost()

        self.first_iter = False
        return u0, x_pred, cost_value
    

    def get_solver_residuals(self) -> dict[str, float | None]:
        """
        Return acados residuals.

        For SQP_RTI, recompute=True is useful because residuals are not always
        freshly available after a real-time iteration.
        """
        try:
            res = np.asarray(
                self.solver.get_residuals(recompute=True),
                dtype=float,
            ).reshape(-1)

            return {
                "res_stat": float(res[0]),
                "res_eq": float(res[1]),
                "res_ineq": float(res[2]),
                "res_comp": float(res[3]),
            }

        except Exception:
            return {
                "res_stat": None,
                "res_eq": None,
                "res_ineq": None,
                "res_comp": None,
            }
    

    @staticmethod
    def _shift_u_traj(u_traj: np.ndarray) -> np.ndarray:
        shifted = np.zeros_like(u_traj)
        shifted[:-1, :] = u_traj[1:, :]
        shifted[-1, :] = u_traj[-1, :]
        return shifted


    @staticmethod
    def _block_diag(A: np.ndarray, B: np.ndarray) -> np.ndarray:
        out = np.zeros((A.shape[0] + B.shape[0], A.shape[1] + B.shape[1]))
        out[:A.shape[0], :A.shape[1]] = A
        out[A.shape[0]:, A.shape[1]:] = B
        return out


    @staticmethod
    def _finite_box_indices(lb: np.ndarray, ub: np.ndarray) -> np.ndarray:
        lb = np.asarray(lb)
        ub = np.asarray(ub)

        if lb.shape != ub.shape:
            raise ValueError("Lower and upper state bounds must have same shape.")

        return np.where(np.isfinite(lb) & np.isfinite(ub))[0]
