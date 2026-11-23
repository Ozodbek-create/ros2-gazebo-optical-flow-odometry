# Optical flow interface, version 1

Geometry remains: level downward camera, image right=-body Y, down=-body X;
physical camera mounting (-0.15,0) m relative to body_link. No TF is published.

## Motion and compatibility

- `/optical_flow/measurement`: `std_msgs/msg/String`, reliable queue 10. One JSON
  object per decoded image, including invalid frames. This keeps one atomic
  interval/quality record without adding a ROSIDL package or generated build
  dependency. JSON field names and units below are the versioned wire contract;
  consumers must reject unsupported versions. A generated custom interface would
  provide stronger compile-time typing in a future multi-language deployment.
- Required fields: `version`=1; integer `start_ns`, `end_ns` in image/simulation
  clock nanoseconds; Boolean `valid`; string `reason`; `du`,`dv` in pixels;
  `dtheta` in radians; integer `detected`,`tracked`,`inliers`; `inlier_ratio`;
  `residual_px` (RMS Euclidean inlier residual in actual pixels); `scale`
  (similarity linear scale); `coverage` (inlier convex-hull area/image area).
  The publisher also supplies `frame_id` and `processing_ms` (wall time, excluding
  serialization, diagnostics and debug rendering). Initial records have zero
  duration and valid=false. Invalid motion fields are not observations of zero
  displacement. `clock_reset` rebases the interval stream, not the accumulated
  pose. Duplicate image timestamps do not replace the tracker's previous frame.
- `/optical_flow/image_motion`: unchanged `geometry_msgs/msg/Vector3Stamped`;
  valid measurements only, x/y pixels, z image radians, stamp at interval end.
- Metric input defaults to the atomic topic. `legacy_input:=true` instead
  subscribes to the old vector topic and ignores atomic records. Never integrate
  both. Legacy mode cannot guarantee interval correctness or quality; restarting
  both nodes is required when migrating from the original running processes.

## Quality and admission

`valid` is a conjunction of engineering gates, **not a probability of correctness**:
at least 12 successfully tracked features and 8 inliers; inlier ratio >=0.5;
RMS inlier residual <=1.5 px; scale in [0.95,1.05]; spatial coverage >=1%;
|du|,|dv|<=50 px and |dtheta|<=0.25 rad; positive duration <=0.2 s.
Counts, ratios and numeric values are independently checked by the receiver.
These broad initial gates passed both clean recordings; they require tuning
against labeled failures before being considered calibrated operating limits.
Neither low residual nor a high inlier count proves correct correspondences.
Repeated patterns, coherent outliers and incorrect camera geometry can pass.

Failures rebase tracking to the current image so later accepted deltas describe
only their stated local interval. The receiver rejects overlapping, duplicate,
out-of-order, nonfinite, malformed and implausible records. A missing interval
or invalid nonzero-duration observation latches `pose_incomplete=true`.
Subsequent local increments may be integrated, but missing motion is never
invented and the global pose is not declared recovered. Restart the metric node
to start a new origin after an unrecoverable gap. A clock rewind preserves the
pose and latches incomplete rather than silently resetting coordinates.

## Diagnostics and odometry

- `/diagnostics`: standard `diagnostic_msgs/msg/DiagnosticArray`, one tracking
  status per decoded frame and one odometry status per received record.
  Tracking status `optical_flow/tracking` contains raw metrics, reason and
  cumulative failures. It is OK for an admitted visual estimate, WARN for a
  failure/initialization. Image conversion failure is ERROR.
- `optical_flow/odometry` contains admitted/rejected record counts and the
  latched pose flag. Any incomplete pose is ERROR; initialization/rejected
  records without lost motion are WARN. A 1 Hz wall-time watchdog emits STALE
  after one second without input. Paused simulation also produces STALE;
  it is an input freshness warning, not proof of a tracking malfunction.
- `/optical_flow/odom` and `/optical_flow/delta_body` retain their types/units.
  Body origin is default. For axle: `reference_x:=-0.15`,
  `reference_frame:=drive_axle`, keep `camera_x:=-0.15`.
  Delta is finite translation in previous reference axes; Odometry.twist uses
  interval-average translation expressed in current child axes, divided by the
  record's duration. No initial valid increment is discarded in atomic mode.
- After pose loss the x/y/yaw pose covariance diagonal is set to 1e6 as an
  explicit conservative unknown marker. Neither it nor the existing nominal
  covariance is a statistically calibrated uncertainty model. No zero-motion
  odometry is emitted when tracking fails; consumers must watch diagnostics and
  timestamps rather than treating an old pose as a fresh stationary reading.
