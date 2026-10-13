# e-puck simulation analysis

[Back to the project overview](../README.md)

This analysis summarizes the nine recorded Python simulation runs from
13 October 2026. All nine runs completed without a reported runtime or solver
error. They cover three straight paths, three circular paths, and three
sinusoidal paths; each table entry links to its experiment report and plots.

## Setup

The runs use the nonlinear differential-drive MPC model with a 10 ms control
period and a 50-step prediction horizon. The Python plant uses RK4 with a 1 ms
integration step. The e-puck parameters include a mass of 0.15 kg, a wheel radius
of 0.02 m, a maximum linear velocity of 0.15 m/s, a torque limit of 0.0005 Nm per
motor, and a reference acceleration limit of 0.233 m/s².

Linear and angular friction coefficients are fixed at 0.1 and 0.0001 in the
model. These values are assumptions, not identified Webots or hardware
parameters. The simulation plant and prediction model use the same underlying
dynamics, so these experiments mainly assess the controller and reference
generation under matching model assumptions.

Further experiments are planned with varying plant parameters while keeping the
MPC prediction model fixed. These experiments will assess robustness to model
mismatch and provide a basis for understanding and comparing the later Webots
experiments, particularly the effects of uncertain friction parameters.

## Recorded results

Values below come from the generated experiment reports. Position metrics are
converted from metres to millimetres; velocity RMSE is in m/s.

| Scenario | Position RMSE [mm] | Maximum position distance [mm] | Velocity RMSE [m/s] |
| --- | ---: | ---: | ---: |
| [Circle: radius 4 m, 2 turns, CW](../experiments/sim_e_puck_circular/2026-10-13_21-16-27-036528_radius-4.0_heading-3.14_turns-2_clockwise/report.md) | 0.303 | 2.047 | 0.000221 |
| [Circle: radius 1 m, 4 turns, CW](../experiments/sim_e_puck_circular/2026-10-13_21-27-45-929265_radius-1.0_heading-2.0_turns-4_clockwise/report.md) | 0.372 | 1.959 | 0.000255 |
| [Circle: radius 2 m, 2 turns, CCW](../experiments/sim_e_puck_circular/2026-10-13_21-36-10-748239_radius-2.0_heading-0.0_turns-2_counterclockwise/report.md) | 0.063 | 0.159 | 0.000312 |
| [Sine: length 4 m, amplitude 1 m, 2 cycles](../experiments/sim_e_puck_sinusoidal/2026-10-13_21-42-45-218992_length-4.0_amplitude-1.0_cycles-2.0/report.md) | 1.103 | 2.634 | 0.000393 |
| [Sine: length 6 m, amplitude 1 m, 2 cycles](../experiments/sim_e_puck_sinusoidal/2026-10-13_21-47-27-377527_length-6.0_amplitude-1.0_cycles-2.0/report.md) | 0.566 | 2.055 | 0.000408 |
| [Sine: length 7 m, amplitude 2 m, 4 cycles](../experiments/sim_e_puck_sinusoidal/2026-10-13_21-57-23-035661_length-7.0_amplitude-2.0_cycles-4.0/report.md) | 1.041 | 2.225 | 0.001904 |
| [Straight to [4.0, 0.0]](../experiments/sim_e_puck_straight/2026-10-13_20-49-55-893475_start-0.0_0.0_end-4.0_0.0/report.md) | 0.300 | 0.460 | 0.000442 |
| [Straight to [4.0, 3.0]](../experiments/sim_e_puck_straight/2026-10-13_20-53-07-242842_start-0.0_0.0_end-4.0_3.0/report.md) | 0.570 | 1.673 | 0.000442 |
| [Straight to [-4.0, 3.0]](../experiments/sim_e_puck_straight/2026-10-13_20-57-09-500382_start-0.0_0.0_end--4.0_3.0/report.md) | 0.667 | 2.039 | 0.000442 |

The straight paths include heading alignment toward different target directions.
Their position RMSE ranges from 0.300 to 0.667 mm. Circular paths cover both
rotation directions, different radii, and multiple laps; their position RMSE
ranges from 0.063 to 0.372 mm.

Sinusoidal paths have the largest position errors in this set, with RMSE between
0.566 and 1.103 mm and maximum distance below 2.634 mm. The 7 m path with a 2 m
amplitude and four cycles has the highest velocity RMSE (0.001904 m/s) and maximum
heading error (about 0.052 rad, or 3°). This is consistent with a more demanding
changing-curvature reference, but the runs do not isolate curvature as the sole
cause of the error.

## Interpretation and limitations

The recorded results support accurate geometric tracking for these scenarios
under the selected simulation assumptions. They do not yet validate the friction
model, physical motor limits, or performance in Webots.

Position error measures the shortest distance to the sampled reference path. It
does not measure arrival time, path completion, or completed laps. In particular,
a robot moving slowly along a straight reference can have almost zero geometric
error while failing to follow its velocity profile.

Report averages cover the entire recorded run, including startup and any time
spent near the endpoint. Runs have different durations, and long periods at rest
can lower the aggregate error. Velocity and orientation references use the first
stage of the latest MPC reference window; startup rotation references partly
follow the measured state. The metrics should therefore be read together with
the time plots and trajectory progress, rather than as a standalone ranking.

## Next steps

The next focus is reliable trajectory tracking and stopping in the Webots digital
twin. This requires identifying effective friction and checking the consistency
of speed and torque limits, then evaluating model mismatch with the wheel,
GPS, and compass diagnostics. The current simulation results provide a baseline
for that comparison.
