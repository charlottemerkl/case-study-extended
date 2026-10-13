# Experiment 2026-10-13_21-16-27-036528_radius-4.0_heading-3.14_turns-2_clockwise

Started: 2026-10-13T21:16:27.036528+02:00

Plant: `sim`

Status: `completed`

Samples: 400000; last recorded simulation time: 399.999 s

## Results

![Reference and actual trajectory](trajectory.png)

![Reference and actual velocity](velocity.png)

![Heading angle](angle.png)

![Angular velocity](angular_velocity.png)

![Applied motor torques](motor_torques.png)

![MPC horizon objective](mpc_cost.png)

![Position-to-reference distance](position_error.png)

## Tracking evaluation

| Deviation | Samples | MSE | MAE | RMSE | Max |
| --- | ---: | ---: | ---: | ---: | ---: |
| Position-to-reference distance | 400000 | 9.1819e-08 m² | 7.8916e-05 m | 0.000303016 m | 0.00204742 m |
| Linear velocity v − v_ref | 400000 | 4.87377e-08 (m/s)² | 2.46984e-05 m/s | 0.000220766 m/s | 0.00736188 m/s |
| Orientation wrap(θ − θ_ref) | 400000 | 5.06055e-07 rad² | 0.000253192 rad | 0.000711375 rad | 0.0100052 rad |
| Angular velocity ω − ω_ref | 400000 | 2.13747e-07 (rad/s)² | 5.75833e-05 rad/s | 0.000462327 rad/s | 0.0102612 rad/s |

MSE = mean(e²), MAE = mean(abs(e)), RMSE = sqrt(MSE), Max = max(abs(e)). All evaluated samples have equal weight, including partial runs. Position error is the shortest distance to the line segments in reference.csv; this approximates the continuous reference and does not measure timing, lap progress or path completion. Velocity error is v − v_ref at the same recorded time. v_ref is the first velocity of the latest MPC reference window, held between MPC updates. Orientation error is the shortest angular difference atan2(sin(θ − θ_ref), cos(θ − θ_ref)); angular velocity error is ω − ω_ref without angle wrapping. Angular references likewise use the first stage of the latest MPC window, held between updates. During startup rotation this first stage is initialized from the measured orientation and angular velocity. Samples missing a reference are excluded from the corresponding evaluation. Evaluation unavailable (—) indicates missing data.

## Data convention

`data.csv` contains the plant state at time_s before integration, and the control applied over the following plant interval. Angles are in rad, positions in m, velocities in m/s and rad/s, torques in N m. `cost` is the latest MPC horizon objective (not an instantaneous stage cost); it is held between updates. `mpc_updated=1` marks a new MPC solution. `theta_ref` and `omega_ref` are the first-stage heading and angular velocity of the latest MPC reference window, held between updates, including startup rotation. theta_ref uses the angle branch aligned to the plant state by the controller. The last row can precede an interrupted or failed integration step.

## Experiment configuration

```yaml
robot:
  name: e_puck
ref:
  start_position:
  - 0.0
  - -4.0
  - 0.0
  - 0.0
  - 0.0
  traj_type: circular
  spacing: 0.1
  bounds:
    x:
    - -5.0
    - 5.0
    y:
    - -5.0
    - 5.0
  t_final: 400.0
  startup:
    heading_tolerance: 0.02
    angular_speed_tolerance: 0.02
    angular_velocity: 1.0
    angular_acceleration: 1.0
  straight:
    end:
    - -4.0
    - 3.0
  circular:
    radius: 4.0
    heading: 3.14
    turns: 2
    clockwise: true
  sinusoidal:
    length: 4.0
    heading: 0.0
    amplitude: 1.0
    cycles: 2.0
plant:
  dt: 0.001
  twin: null
  sim:
    solver: rk4
mpc:
  dt: 0.01
  vehicle_model: nonlinear
  horizon: 50
  acados_json_file: build/acados_ocp_tracking_diff_drive.json
  cost:
    Q:
    - 20.0
    - 20.0
    - 10.0
    - 10.0
    - 10.0
    R:
    - 0.5
    - 0.5
    Qe:
    - 40.0
    - 40.0
    - 0.0
    - 0.0
    - 0.0
```

## Robot configuration

```yaml
robot:
  name: e_puck
  type: differential_drive
  mass: 0.15
  radius: 0.035
  wheel_radius: 0.02
  wheel_base: 0.053
  linear_friction: 0.1
  angular_friction: 0.0001
  inertia:
    Ixx: 9.78585e-05
    Iyy: 8.64333e-05
    Izz: 8.74869e-05
    Ixy: 0.0
    Ixz: 0.0
    Iyz: 0.0
  state_order:
  - x
  - y
  - theta
  - v
  - omega
  input_order:
  - tau_left
  - tau_right
  max_linear_velocity: 0.15
  max_angular_velocity: 7.536
  max_wheel_angular_velocity: 6.28
  a_max: 0.233
  tau_max: 0.0005
  safety_radius: 0.005
```
