import argparse
import sys
import yaml
import time
import threading
from pathlib import Path
import csv
import json
import traceback

from pynput import keyboard

import casadi as ca
import numpy as np

# Direct script execution adds src/sim, rather than the project root, to sys.path.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.sim.dynamics import RobotParams
from src.sim.solver import (
    euler_step,
    rk4_step,
)
from src.mpc.reference_generation import (
    build_full_ref_traj,
    sample_ref_window,
)

from src.mpc.vehicle_models import (
    ModelConfig,
    LinearDynamicsModel,
    NonLinearDynamicsModel,
)
from src.mpc.mpc_controller import MPC, MPCConfig 
# from mpc.mpc_report import (
#     make_mpc_report_row,
#     write_mpc_report,
# )

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--exp-config",
        type=str,
        default="./configs/experiment.yaml",
        help="Path to experiment YAML config.",
    )

    parser.add_argument(
        "--robot-config",
        type=str,
        default="./configs/robots/e_puck.yaml",
        help="Path to robot YAML config.",
    )
    

    return parser.parse_args()


def load_config(file_path: str) -> dict:
    with open(file_path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    if data is None:
        return {}

    return data


def build_vehicle_model(robot_name: str, model_type: str):
    """
    Load MPC vehicle model config from:

        config/robot/{robot_name}.yaml

    model_type:
        "linear" or "nonlinear"

    Returns an instance of LinearDynamicsModel or NonLinearDynamicsModel depending on the model_type argument.
    """

    robot_config_path = Path("configs/robots") / f"{robot_name}.yaml"
    model_config = ModelConfig.load_model_config(str(robot_config_path))

    if model_type == "linear":
        return LinearDynamicsModel(model_config)

    if model_type == "nonlinear":
        return NonLinearDynamicsModel(model_config)

    raise ValueError(f"Unknown vehicle model type: {model_type}")


def setup_mpc_sim(exp_config: dict):
    """
    Build vehicle model, MPC, and full reference trajectory.
    """

    robot_name = exp_config["robot"]["name"]
    vehicle_model_type = exp_config["mpc"].get("vehicle_model", "nonlinear")

    vehicle_model = build_vehicle_model(
        robot_name=robot_name,
        model_type=vehicle_model_type,
    )

    dt = float(exp_config["mpc"].get("dt", 0.1))
    N = int(exp_config["mpc"].get("horizon", 40))

    cost = exp_config["mpc"].get("cost") or {}

    mpc_config = MPCConfig(
        N=N,
        dt=dt,
        json_file=exp_config["mpc"].get(
            "acados_json_file",
            "build/acados_ocp_tracking_diff_drive.json",
        ),
        code_export_directory=exp_config["mpc"].get("code_export_directory"),
        Q=cost.get("Q"),
        R=cost.get("R"),
        Qe=cost.get("Qe"),
    )

    mpc = MPC(
        vehicle_model=vehicle_model,
        config=mpc_config,
    )

    v_max = vehicle_model.config.v_max
    a_max = vehicle_model.config.a_max

    # Validate the full reference geometry before starting the simulation.
    full_ref_traj = build_full_ref_traj(
        ref=exp_config["ref"],
        v_max=v_max,
        a_max=a_max,
    )

    return mpc, full_ref_traj


def align_ref_theta_to_state(
    x_ref_window: np.ndarray,
    theta: float,
) -> np.ndarray:
    x_ref_window = x_ref_window.copy()

    k = np.round((theta - x_ref_window[0, 2]) / (2.0 * np.pi))
    x_ref_window[:, 2] += 2.0 * np.pi * k

    return x_ref_window



def run_sim_np(
    robot_params: RobotParams,
    exp_config: dict,
    mpc: MPC | None = None,
    full_ref_traj: tuple | None = None,
) -> dict | None:
    """
    Run the sim loop. Returns None on success, or an error-info dict if the
    simulation aborted on a solver/runtime error (the partial report + metrics
    are still written so the failure can be inspected).
    """

    # -------------- setup ------------- #
    dt = float(exp_config["plant"]["sim"].get("dt", 0.1))
    mpc_dt = float(exp_config["mpc"].get("dt", 0.1))
    mpc_ratio = max(int(mpc_dt / dt), 1)

    solver_type = exp_config["mpc"].get("solver", "rk4")  

    state = np.asarray(exp_config["ref"]["start_position"], dtype=float)  # State: [x, y, theta, v, omega]

    xy_spline = None
    v_spline = None
    if full_ref_traj is not None:
        xy_spline, v_spline = full_ref_traj
        if state.shape != (5,) or not np.all(np.isfinite(state)):
            raise ValueError("ref.start_position must contain five finite values.")
        if not np.allclose(state[:2], xy_spline(0.0), rtol=0.0, atol=1e-9):
            raise ValueError(
                "ref.start_position[:2] must match the reference start. "
                "Change parameters in experiment.yaml."
            )
    s_ref = 0.0  # Arc-length progress along the reference
    x_ref_window = None

    # --------------------------------------------- #

    step = 0
    t = 0.0
    
    action = np.zeros(2)
    cost = None
    residuals = None
    
    mpc_report_rows = []
    csv_rows = []
    error_info: dict | None = None

    try:
        while True:
            
            if (step % mpc_ratio) == 0:
 
                x_ref_window, s_ref = sample_ref_window(
                    xy_spline=xy_spline,
                    v_spline=v_spline,
                    state=state,
                    N=mpc.N,
                    dt=mpc.config.dt,
                    s_prev=s_ref,
                )

                x_ref_window = align_ref_theta_to_state(
                    x_ref_window=x_ref_window,
                    theta=state[2],
                )

                action, x_pred, cost = mpc.compute_control(
                    x0=state,
                    x_ref_traj=x_ref_window,
                    use_prev_u=True,
                )
                residuals = mpc.get_solver_residuals()
                print(f"[mpc] step={step} t={t:.3f} action = {action} cost={cost:.4f} res={residuals}")


            if solver_type == "euler":
                state = euler_step(state, action, dt, robot_params)
            elif solver_type == "rk4":
                state = rk4_step(state, action, dt, robot_params)
            else:
                raise ValueError(f"Unknown solver_type: {solver_type}")
            print(f"[sim] step={step} t={t:.3f} state = {state}")

            step += 1
            t += dt


    except KeyboardInterrupt:
        print("Simulation stopped by user.")

    except Exception as e:
        # Solver / runtime failure (e.g. acados status != 0 from a badly
        # perturbed model). Record it so that the failure is reported. Partial data is still saved
        # by the finally block below.
        error_info = {
            "type": type(e).__name__,
            "message": str(e),
            "step": int(step),
            "t": float(t),
            "traceback": traceback.format_exc(),
        }
        print(f"[error] run aborted at step {step} (t={t:.3f}): {e}")

    return error_info



def main() -> None:

    args = parse_args()

    # Sim-only and MPC-only configs are merged into the single dict the rest
    # of the code consumes. MPC keys win on shared keys (e.g. mpc_dt).
    exp_config = load_config(str(Path(args.exp_config).resolve()))
    robot_config = load_config(str(Path(args.robot_config).resolve()))

    robot_params = RobotParams(
        m=float(robot_config.get("mass", 10.0)),
        J=float(robot_config.get("inertia", {}).get("Izz", 1.0)),
        L=float(robot_config.get("wheel_base", 0.5)),
        r=float(robot_config.get("wheel_radius", 0.1)),
        b_v=float(robot_config.get("linear_friction", 0.1)),
        b_omega=float(robot_config.get("angular_friction", 0.1)),
    )

    status: dict = {"ok": True, "error": None}

    try:
        mpc = None
        full_ref_traj = None

        mpc, full_ref_traj = setup_mpc_sim(exp_config)

        error_info = run_sim_np(
            robot_params=robot_params,
            exp_config=exp_config,
            mpc=mpc,
            full_ref_traj=full_ref_traj,
        )
        if error_info is not None:
            status["ok"] = False
            status["error"] = error_info

    except Exception as e:
        # Failure during setup (e.g. acados model build for a badly perturbed model)
        status["ok"] = False
        status["error"] = {
            "type": type(e).__name__,
            "message": str(e),
            "phase": "setup",
            "traceback": traceback.format_exc(),
        }
        traceback.print_exc()

    finally:
        with open("experiments/status.json", "w") as f:
            json.dump(status, f, indent=2)
        print(f"[status] ok={status['ok']} -> experiments/status.json")


if __name__ == "__main__":
    main()
