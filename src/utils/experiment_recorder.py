"""Shared experiment artifacts for NumPy simulation and Webots controllers."""
import csv
from copy import deepcopy
from datetime import datetime
import json
from pathlib import Path
import re

import numpy as np
import yaml
from matplotlib.figure import Figure
from matplotlib.backends.backend_agg import FigureCanvasAgg

from src.utils.trajectory_metrics import trajectory_metrics, velocity_metrics


class ExperimentRecorder:
    """
    Stream pre-step states and applied controls to a unique experiment folder.

    State order: x, y, theta, v, omega; inputs: tau_left, tau_right.
    cost is the latest MPC horizon objective, held between control updates.
    Call finalize in a finally block, including after errors or interruption.
    """

    columns = (
        "step", "time_s", "x", "y", "theta", "v", "omega",
        "tau_left", "tau_right", "cost", "mpc_updated", "v_ref",
    )

    def __init__(self, root, *, plant_type, experiment_config, robot_config):
        if plant_type not in {"sim", "twin"}:
            raise ValueError("plant_type must be 'sim' or 'twin'.")
        self.experiment_config = deepcopy(experiment_config)
        self.robot_config = deepcopy(robot_config)
        self.plant_type = plant_type
        self.started_at = datetime.now().astimezone()
        self.samples = 0
        self.last_time = None
        self.reference_xy = None
        self.plot_name = None
        self.metrics = None
        self.speed_metrics = None
        self.speed_plot_name = None

        def slug(value):
            return re.sub(r"[^A-Za-z0-9_-]+", "-", str(value)).strip("-") or "unknown"

        name = "_".join((
            self.started_at.strftime("%Y-%m-%d_%H-%M-%S-%f"),
            plant_type, slug(experiment_config["robot"]["name"]),
            slug(experiment_config["ref"]["traj_type"]),
        ))
        root = Path(root)
        root.mkdir(parents=True, exist_ok=True)
        for suffix in range(10000):
            self.directory = root / (name if suffix == 0 else f"{name}_{suffix}")
            try:
                self.directory.mkdir()
                break
            except FileExistsError:
                continue
        else:
            raise FileExistsError("Cannot allocate a unique experiment directory.")

        for filename, config in (("experiment.yaml", self.experiment_config),
                                 ("robot.yaml", self.robot_config)):
            (self.directory / filename).write_text(
                yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8"
            )
        self._file = (self.directory / "data.csv").open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.columns)
        self._file.flush()
        self._write_report({"state": "running", "ok": None, "error": None})

    def set_reference(self, xy):
        """Accept sampled reference coordinates, independently of an MPC model."""
        xy = np.asarray(xy, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
            raise ValueError("Reference must be a finite (N, 2) array.")
        self.reference_xy = xy.copy()
        np.savetxt(self.directory / "reference.csv", xy, delimiter=",", header="x,y", comments="")

    def record(self, *, step, time_s, state, action, cost, mpc_updated, v_ref=None):
        state, action = np.asarray(state, dtype=float), np.asarray(action, dtype=float)
        if state.shape != (5,) or action.shape != (2,):
            raise ValueError("Expected state shape (5,) and action shape (2,).")
        if not np.isfinite(state).all() or not np.isfinite(action).all():
            raise ValueError("Cannot record non-finite state or action.")
        if v_ref is not None and not np.isfinite(float(v_ref)):
            raise ValueError("v_ref must be finite when provided.")
        self._writer.writerow((int(step), float(time_s), *state, *action,
                               "" if cost is None else float(cost), int(mpc_updated),
                               "" if v_ref is None else float(v_ref)))
        self._file.flush()
        self.samples += 1
        self.last_time = float(time_s)

    def _read_actual_xy(self):
        with (self.directory / "data.csv").open(newline="", encoding="utf-8") as stream:
            actual = np.array([(float(row["x"]), float(row["y"]))
                               for row in csv.DictReader(stream)])
        return actual.reshape(-1, 2)

    def _plot_trajectory(self, actual):
        fig = Figure(figsize=(7, 7), layout="constrained")
        FigureCanvasAgg(fig)  # No GUI/display required, also usable from Webots.
        ax = fig.subplots()
        if self.reference_xy is not None and len(self.reference_xy):
            ax.plot(*self.reference_xy.T, "--", label="Reference", color="tab:orange")
        if len(actual):
            ax.plot(*actual.T, label="Actual trajectory", color="tab:blue")
            ax.scatter(*actual[0], marker="o", color="green", label="Start", zorder=3)
            ax.scatter(*actual[-1], marker="x", color="red", label="Last sample", zorder=3)
        else:
            ax.text(0.5, 0.5, "No plant samples recorded", transform=ax.transAxes,
                    ha="center", va="center")
        bounds = self.experiment_config.get("ref", {}).get("bounds", {})
        if "x" in bounds and "y" in bounds:
            xmin, xmax = bounds["x"]
            ymin, ymax = bounds["y"]
            ax.plot([xmin, xmax, xmax, xmin, xmin], [ymin, ymin, ymax, ymax, ymin],
                    ":", color="gray", label="Configured bounds")
        ax.set(xlabel="x [m]", ylabel="y [m]", title="Reference and actual trajectory")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.3)
        if ax.get_legend_handles_labels()[0]:
            ax.legend()
        self.plot_name = "trajectory.png"
        fig.savefig(self.directory / self.plot_name, dpi=160)
        fig.clear()

    def _evaluate_velocity(self):
        with (self.directory / "data.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        paired = [row for row in rows if row.get("v_ref", "") != ""]
        if not paired:
            return
        actual = [float(row["v"]) for row in paired]
        reference = [float(row["v_ref"]) for row in paired]
        self.speed_metrics = velocity_metrics(actual, reference)
        time = [float(row["time_s"]) for row in rows]
        fig = Figure(figsize=(9, 4), layout="constrained")
        FigureCanvasAgg(fig)
        ax = fig.subplots()
        ax.plot(time, [float(row["v"]) for row in rows], label="Actual v")
        ax.step(time, [float(row["v_ref"]) if row.get("v_ref", "") else np.nan
                       for row in rows], where="post", linestyle="--", label="Reference v")
        ax.set(xlabel="Time [s]", ylabel="Velocity [m/s]", title="Reference and actual velocity")
        ax.grid(True, alpha=0.3)
        ax.legend()
        self.speed_plot_name = "velocity.png"
        fig.savefig(self.directory / self.speed_plot_name, dpi=160)
        fig.clear()

    def _evaluation_table(self):
        lines = ["| Deviation | Samples | MSE | MAE | RMSE |",
                 "| --- | ---: | ---: | ---: | ---: |"]
        for name, metrics, unit, squared in (
            ("Position-to-reference distance", self.metrics, "m", "m²"),
            ("Linear velocity v − v_ref", self.speed_metrics, "m/s", "(m/s)²"),
        ):
            if metrics is None:
                lines.append(f"| {name} | 0 | — | — | — |")
            else:
                lines.append(
                    f"| {name} | {metrics['samples']} | {metrics['mse']:.6g} {squared} | "
                    f"{metrics['mae']:.6g} {unit} | {metrics['rmse']:.6g} {unit} |"
                )
        lines += ["", "MSE = mean(e²), MAE = mean(abs(e)), RMSE = sqrt(MSE). "
                  "All evaluated samples have equal weight, including partial runs. "
                  "Position error is the shortest distance to the line segments in reference.csv; "
                  "this approximates the continuous reference and does not measure timing, "
                  "lap progress or path completion. Velocity error is v − v_ref at the same "
                  "recorded time. v_ref is the first velocity of the latest MPC reference "
                  "window, held between MPC updates. Samples without v_ref are excluded "
                  "from velocity evaluation. Evaluation unavailable (—) indicates missing data."]
        return "\n".join(lines)

    def _write_report(self, status):
        parts = [
            f"# Experiment {self.directory.name}",
            f"Started: {self.started_at.isoformat()}",
            f"Plant: `{self.plant_type}`",
            f"Status: `{status['state']}`",
            f"Samples: {self.samples}; last recorded simulation time: {self.last_time} s",
            "## Results",
            f"![Reference and actual trajectory]({self.plot_name})" if self.plot_name
            else "Trajectory plot will be generated when the run finishes.",
            f"![Reference and actual velocity]({self.speed_plot_name})" if self.speed_plot_name
            else "Velocity comparison unavailable: no paired velocity samples.",
            "## Tracking evaluation",
            self._evaluation_table(),
            "## Data convention",
            "`data.csv` contains the plant state at time_s before integration, and the "
            "control applied over the following plant interval. Angles are in rad, "
            "positions in m, velocities in m/s and rad/s, torques in N m. "
            "`cost` is the latest MPC horizon objective (not an instantaneous stage cost); "
            "it is held between updates. `mpc_updated=1` marks a new MPC solution. "
            "The last row can precede an interrupted or failed integration step.",
            "## Experiment configuration",
            "```yaml\n" + yaml.safe_dump(self.experiment_config, sort_keys=False, allow_unicode=True) + "```",
            "## Robot configuration",
            "```yaml\n" + yaml.safe_dump(self.robot_config, sort_keys=False, allow_unicode=True) + "```",
        ]
        if status.get("error"):
            parts += ["## Error", "```json\n" + json.dumps(status["error"], indent=2) + "\n```"]
        (self.directory / "report.md").write_text("\n\n".join(parts) + "\n", encoding="utf-8")

    def finalize(self, status):
        """Close streamed data and save status, plot and report, even for partial runs."""
        self._file.close()
        payload = {**status, "finished_at": datetime.now().astimezone().isoformat(),
                   "samples": self.samples, "last_time_s": self.last_time}
        (self.directory / "status.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        try:
            actual = self._read_actual_xy()
            self.metrics = trajectory_metrics(actual, self.reference_xy)
            self._plot_trajectory(actual)
            self._evaluate_velocity()
        finally:
            self._write_report(payload)
