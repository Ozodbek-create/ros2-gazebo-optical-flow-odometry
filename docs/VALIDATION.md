# Validation evidence and reproduction

The historical 56.033-second run covered 1.350615 m of actual body-origin motion,
with 1,682 camera frames and two speed levels/two repetitions of forward/reverse,
CW/CCW, arcs and stops. Optical position ATE RMSE was 0.003013388 m; final position
error 0.003256753 m and yaw error −1.942421°. All 1,681 clean frame pairs tracked.
These exact numbers are in `validation/reference_summary.json`.

Ground truth was the Gazebo SceneBroadcaster world pose of the robot model,
interpolated to image timestamps with unwrapped yaw. Both trajectories were
aligned once at the starting body pose; no scale fit or per-phase reset was used.
Wheel `/odom` was not treated as ground truth. RPE compares relative SE(2)
motion approximately one second apart. The compact `reference_trajectory.npz`
contains aligned poses, ground truth, time, distance and errors, without raw
images or private workstation paths. The PNG is from that actual run.

Comparisons on identical recordings tested forward/backward LK checks and
80/400-corner settings. Results were mixed across recordings; the original
200-corner algorithm was retained. Blank moving frames caused lost displacement
and a persistent incomplete-pose flag. A high inlier ratio does not establish
global pose accuracy. See the interface specification for admission gates.

The fresh recorder and evaluator use portable paths in this distribution. The
recorder checks the pattern bounds, predicted/live camera position, input streams
and competing ROS command publishers. It sends zero velocity in cleanup, but this
does not cover a host crash. Commands are bounded; a human should remain ready to
stop the simulated drive. It cannot detect every possible direct Gazebo command
publisher. Use an isolated ROS_DOMAIN_ID/GZ_PARTITION for independent sessions.

Raw images from the historical run are deliberately excluded. To reproduce image
processing results, record a new run and evaluate it using README commands; the
simulation is not deterministic enough to promise identical benchmark numbers.
EOF/gaps and source freshness must be considered when interpreting any new run.
