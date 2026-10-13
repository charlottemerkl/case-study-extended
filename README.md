# MPC Differential-Drive Robot

## Project status

**Status: Work in Progress — Active Development**

## Project overview

This project extends a group case study at the Technical University of Munich
(TUM) on combining Model Predictive Control (MPC) and Reinforcement Learning (RL) for vehicle control.
Only the MPC and simulation components, which I helped develop as part of the
group, were carried over into this repository. RL is outside the scope of the
current extension.

The inherited code has been refactored to prepare it for further development.
The goal is to extend MPC trajectory tracking from a Python simulation to a
Webots digital twin and evaluate how the controller behaves when the simulated
plant differs from its prediction model.

## Features

- Reference trajectory generation for straight, circular, and sinusoidal paths,
  with velocity profiles and initial heading alignment.
- MPC trajectory tracking using `acados`, with linear and nonlinear
  differential-drive prediction models and configurable state and torque limits.
- YAML experiment configuration and example robot configurations for the e-puck,
  Pioneer, and TurtleBot 4; current results focus on the e-puck.
- Python plant simulation with Euler and RK4 integration.
- Experiment recording with configuration snapshots, CSV measurements,
  trajectory and state plots, and tracking metrics. 

## Current results and outlook

Experiments with the e-puck simulation cover straight, circular, and sinusoidal
trajectories. The recorded runs demonstrate trajectory tracking under the chosen
model assumptions. A short [analysis of the e-puck simulation results](docs/sim-e-puck-analysis.md)
summarizes the scenarios, tracking errors, and limitations.

The next development focus is reliable control of the Webots digital twin.
Initial torque and MPC experiments already exist, but robust stopping and the
transfer between the simulation and Webots still need work.

A central difficulty is that the effective friction parameters of the Webots
plant have not been identified. The friction coefficients used in the Python
simulation and MPC model are currently fixed assumptions rather than calibrated
values. Good tracking in the Python simulation therefore does not establish
that the same parameters accurately describe the digital twin. Further work will
focus on identifying these differences and improving controller robustness.

## Project structure

```text
configs/
  experiment.yaml       Experiment, reference, plant, and MPC settings
  robots/               Example robot parameters and limits
src/
  mpc/                  Prediction models, MPC controller, reference generation
  sim/                  Python plant dynamics, integration, simulation controller
  utils/                Shared configuration, recording, and evaluation
experiments/            Saved runs grouped by plant, robot, and trajectory
scripts/                Project environment activation
external/acados/        acados Git submodule
docs/                   Installation instructions and result analysis
```

Solver-generated build and code export directories are runtime artifacts.

## Installation and documentation

Follow the [installation guide](docs/installation.md) to build `acados`, install
Python dependencies, and activate the project environment.

From the project root, start a Python simulation with:

```bash
source scripts/activate.sh
python src/sim/control_sim.py
```

The default experiment is configured in `configs/experiment.yaml`. Select another
experiment file with `--exp-config` and a robot configuration with
`--robot-config`. Keep the robot YAML consistent with `robot.name` in the
experiment configuration.

See the [simulation analysis](docs/sim-e-puck-analysis.md) for the current results.
Each saved experiment includes a `report.md`, configuration snapshots, CSV data,
and plots for inspecting the individual run.
