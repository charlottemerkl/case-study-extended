import argparse
import sys
import yaml
from pathlib import Path
import traceback


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
    sample_full_traj,
    sample_rotation_window,
)

from src.mpc.vehicle_models import (
    ModelConfig,
    LinearDynamicsModel,
    NonLinearDynamicsModel,
)
from src.mpc.mpc_controller import MPC, MPCConfig 
from src.utils.experiment_recorder import ExperimentRecorder

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


def build_vehicle_model(robot_name: str, model_type: str, robot_config_path=None):
    """
    Load MPC vehicle model config from:

        config/robot/{robot_name}.yaml

    model_type:
        "linear" or "nonlinear"

    Returns an instance of LinearDynamicsModel or NonLinearDynamicsModel depending on the model_type argument.
    """

    robot_config_path = robot_config_path or Path("configs/robots") / f"{robot_name}.yaml"
    model_config = ModelConfig.load_model_config(str(robot_config_path))

    if model_type == "linear":
        return LinearDynamicsModel(model_config)

    if model_type == "nonlinear":
        return NonLinearDynamicsModel(model_config)

    raise ValueError(f"Unknown vehicle model type: {model_type}")


def setup_mpc_sim(exp_config: dict, robot_config_path=None):
    """
    Build vehicle model, MPC, and full reference trajectory.
    """

    robot_name = exp_config["robot"]["name"]
    vehicle_model_type = exp_config["mpc"].get("vehicle_model", "nonlinear")

    vehicle_model = build_vehicle_model(
        robot_name=robot_name,
        model_type=vehicle_model_type,
        robot_config_path=robot_config_path,
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
    recorder: ExperimentRecorder | None = None,
) -> dict | None:
    """
    Run the sim loop. Returns None on success, or an error-info dict if the
    simulation aborted on a solver/runtime error. The caller finalizes the
    recorder, including after KeyboardInterrupt, to preserve partial results.
    """

    # -------------- setup ------------- #
    dt = float(exp_config["plant"]["dt"])
    mpc_dt = float(exp_config["mpc"].get("dt", 0.1))
    if dt <= 0 or mpc_dt < dt or not np.isclose(mpc_dt / dt, round(mpc_dt / dt)):
        raise ValueError("mpc.dt must be a positive integer multiple of plant.dt.")
    mpc_ratio = int(round(mpc_dt / dt))

    solver_type = exp_config["plant"]["sim"].get("solver", "rk4")

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
    startup = exp_config["ref"].get("startup", {})
    heading_tolerance = float(startup.get("heading_tolerance", 0.02))
    angular_speed_tolerance = float(startup.get("angular_speed_tolerance", 0.02))
    alpha_max = float(startup.get("angular_acceleration", 1.0))
    model_config = getattr(getattr(mpc, "vehicle_model", None), "config", None)
    omega_max = min(float(startup.get("angular_velocity", 1.0)),
                    float(getattr(model_config, "omega_max", 1.0)))
    if not np.isfinite([heading_tolerance, angular_speed_tolerance, alpha_max, omega_max]).all() or min(
            heading_tolerance, angular_speed_tolerance, alpha_max, omega_max) <= 0:
        raise ValueError("ref.startup parameters must be finite and positive.")
    tangent = xy_spline(0.0, 1)
    target_heading = float(np.arctan2(tangent[1], tangent[0]))
    heading_error = np.arctan2(np.sin(target_heading - state[2]), np.cos(target_heading - state[2]))
    rotating = (abs(heading_error) > heading_tolerance
                or abs(state[4]) > angular_speed_tolerance)
    # Nonzero initial linear speeds continue with the existing spatial envelope.
    drive_started_at = None if rotating else 0.0
    start_from_rest = abs(state[3]) < 1e-9

    # --------------------------------------------- #

    step = 0
    t = 0.0
    
    action = np.zeros(2)
    cost = None
    residuals = None
    
    error_info: dict | None = None

    try:
        while t < float(exp_config["ref"].get("t_final", 100.0)):
            
            mpc_updated = (step % mpc_ratio) == 0
            if mpc_updated:
                heading_error = np.arctan2(np.sin(target_heading - state[2]),
                                           np.cos(target_heading - state[2]))
                if rotating and abs(heading_error) <= heading_tolerance and abs(state[4]) <= angular_speed_tolerance:
                    rotating = False
                    drive_started_at = t
                    start_from_rest = True
                if rotating:
                    x_ref_window = sample_rotation_window(
                        state, xy_spline(0.0), target_heading, mpc.N, mpc.config.dt,
                        omega_max, alpha_max,
                    )
                else:
                    x_ref_window, s_ref = sample_ref_window(
                        xy_spline=xy_spline,
                        v_spline=v_spline,
                        state=state,
                        N=mpc.N,
                        dt=mpc.config.dt,
                        s_prev=s_ref,
                        drive_time=max(0.0, t - drive_started_at) if start_from_rest else None,
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


            if recorder is not None:
                recorder.record(step=step, time_s=t, state=state, action=action,
                                cost=cost, mpc_updated=mpc_updated,
                                v_ref=float(x_ref_window[0, 3]),
                                theta_ref=float(x_ref_window[0, 2]),
                                omega_ref=float(x_ref_window[0, 4]))

            if solver_type == "euler":
                state = euler_step(state, action, dt, robot_params)
            elif solver_type == "rk4":
                state = rk4_step(state, action, dt, robot_params)
            else:
                raise ValueError(f"Unknown solver_type: {solver_type}")
            print(f"[sim] step={step} t={t:.3f} state = {state}")

            step += 1
            t = step * dt


    except KeyboardInterrupt:
        print("Simulation stopped by user.")
        raise

    except Exception as e:
        # Solver / runtime failure (e.g. acados status != 0 from a badly
        # perturbed model). The caller saves the report and partial data.
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

    exp_config = load_config(str(Path(args.exp_config).resolve()))
    robot_config = load_config(str(Path(args.robot_config).resolve()))

    recorder = ExperimentRecorder(
        Path(__file__).resolve().parents[2] / "experiments",
        plant_type="sim",
        experiment_config=exp_config,
        robot_config=robot_config,
    )
    print(f"[experiment] {recorder.directory}")
    status = {"ok": True, "state": "completed", "error": None}
    try:
        robot = robot_config["robot"]
        robot_params = RobotParams(
            m=float(robot["mass"]),
            J=float(robot["inertia"]["Izz"]),
            L=float(robot["wheel_base"]),
            r=float(robot["wheel_radius"]),
            b_v=float(robot["linear_friction"]),
            b_omega=float(robot["angular_friction"]),
        )
        mpc, full_ref_traj = setup_mpc_sim(exp_config, Path(args.robot_config).resolve())
        recorder.set_reference(sample_full_traj(*full_ref_traj, ds=0.02)[:, :2])
        error_info = run_sim_np(
            robot_params=robot_params,
            exp_config=exp_config,
            mpc=mpc,
            full_ref_traj=full_ref_traj,
            recorder=recorder,
        )
        if error_info is not None:
            status.update(ok=False, state="failed", error=error_info)
    except KeyboardInterrupt:
        status.update(ok=False, state="interrupted")
    except Exception as exc:
        status.update(ok=False, state="failed", error={
            "type": type(exc).__name__, "message": str(exc),
            "phase": "setup", "traceback": traceback.format_exc(),
        })
        traceback.print_exc()
    finally:
        recorder.finalize(status)
        print(f"[status] {status['state']} -> {recorder.directory / 'report.md'}")


if __name__ == "__main__":
    main()
