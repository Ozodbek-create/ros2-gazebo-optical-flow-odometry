# Distribution verification — 23 November 2026

## Actually verified

- Copied only the distribution's `src/` into a new independent Linux workspace.
  Sourced `/opt/ros/jazzy/setup.bash`, without the original robotics workspace.
- `colcon build --symlink-install`: both packages built successfully.
- Core `colcon test`: **37 passed, one existing copyright skip, zero failures**.
  The lint runner emitted Python fork/multithreading warnings.
- Ran the README's default `ros2 launch gazebo_differential_drive_robot
  robot.launch.py` command and both documented odometry executables, using a
  separate ROS domain and Gazebo transport partition.
- The default installed custom world loaded; GUI/server processes ran, the robot
  spawned and the observer received **37 320×240 images**, 37 CameraInfo messages,
  37 atomic measurements (30 valid), **30 optical odometry messages** and 76
  diagnostic arrays over approximately 1.2 seconds of image time. Image frame:
  `down_camera_optical_frame`. Intrinsics: fx=160, fy≈160, cx=160, cy=120.
- The launch was stopped after the smoke check. Gazebo required SIGTERM during
  cleanup after SIGINT; successful data flow preceded the intentional shutdown.
  SDF extension/Ogre visibility warnings did not prevent image publication.
- Python syntax, XML/resource presence, deterministic world regeneration, archive
  integrity, nested-Git exclusion and a source-content privacy scan were checked.

This verifies an **independent clean-source build and actual default launch on
the existing Ubuntu 24.04/WSL2 installation**. It is not a fresh OS/container
dependency installation, a manual GUI visual assessment, or a rerun of the entire
historical trajectory benchmark. No physical hardware accuracy was verified.

## Distribution-only changes

The original working workspace was not converted into a repository or modified.
The robot source is vendored as ordinary files, with upstream revision/credits
in NOTICE. No nested `.git`, submodule pointer, build/install/log tree, raw image
recording, credentials, private workstation paths or unrelated TRIK experiments
are shipped. Original package maintainership attribution is retained; the
intern's contact field uses a GitHub noreply address for public distribution.

Packaging fixes install the custom world and generator, make that installed
world the default, expose Gazebo arguments, replace the stale Classic
`gazebo_ros` dependency with Harmonic `ros_gz_sim`, and declare launch/index
dependencies. Validation scripts now accept portable paths. The optical-flow,
geometry, interval-admission and metric algorithms are unchanged.

## Remaining reproduction considerations

The exact APT/rosdep installation instructions were checked against manifests and
available local dependencies, but not installed on a blank operating system.
Gazebo camera rendering needs appropriate graphics support, including for a
server-only process. Evaluation additionally needs matplotlib and the Jazzy
Gazebo Transport/Messages Python vendor modules. Real-time factor, rendering,
friction and feature localization can vary; new trajectories need not reproduce
the historical millimetre values exactly. Read README and VALIDATION.md for
frame/reference conventions, bounded motion and the limited textured area.
