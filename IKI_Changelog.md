# IKI Changelog

## 2026-11-24 — ROS 2 Jazzy / Nav2 controller fixes

The existing controller algorithm and pluginlib/lifecycle interface are retained.
No new package dependencies were introduced, and `package.xml` was not read or
modified. No changes were committed or pushed.

### Confirmed failure mechanism

The previous configuration code removed slashes from the ROS node namespace and
concatenated it with TF frame IDs. `/nav` plus `odom` became `navodom`, and
`base_link` became `navbase_link`. Required TF exceptions were caught without
aborting, allowing uninitialized transformations into control calculations.

The limiter could then hide NaN values: `fmin(NaN, previous + acceleration * dt)`
returns the finite acceleration bound. Repeated calls can ramp all affected axes
toward their positive maxima. This mechanism and the source defects were
verified; the exact numerical sequence recorded on the physical MPO-700 was not
replayed, so NaN production on that specific run remains an inference.

### Implementation changes

- Removed automatic TF namespacing. Frame names are used literally; topic
  resolution remains independent. Path transformations use the path header
  instead of a hardcoded `map` frame. Robot poses are transformed from their own
  header frame before costmap calculations.
- Centralized required TF lookups. Failures raise `nav2_core::ControllerTFError`
  and clear stored command, filter, motion-state and direction data before
  propagating to Nav2.
- Added finite-value checks for poses, quaternion normalization, transformations,
  odometry/control inputs, stored state, lookahead/cost/braking intermediates,
  raw/filtered/limited commands, and the final command. Limiting helpers reject
  invalid operands before min/max operations can hide them. Very large finite
  costmap coordinates are checked before integer conversion.
- Invalid paths raise `nav2_core::InvalidPath`; numerical, timing and collision
  failures raise `nav2_core::NoValidControl`. Empty paths, missing lookahead
  direction, inconsistent pose frames and invalid quaternions no longer enter
  ordinary control calculations. Reverse path selection handles fewer than six
  poses safely.
- Removed stale-command early returns. Added Nav2 `reset()` support and reset
  filter state when measured planar motion is zero. Invalid replacement plans
  clear the previous plan. Plan replacement is synchronized with computation.
- Replaced mismatched declarations/retrievals with one numeric parameter table.
  Configured translation maxima and stopped thresholds are no longer
  overwritten. Removed duplicate unused limit members. Dynamic changes validate
  the complete proposed batch and apply only after all node callbacks accept it,
  including when the configured plugin is inactive. Missing dynamic gain and
  lookahead assignments are covered by the same table.
- Replaced the independent system clock with the node's ROS clock. Acceleration
  uses the actual accepted interval. Zero, backward, non-finite or excessively
  large intervals reset control state and reject the calculation.
- Kept acceleration limits positive in reverse motion, corrected reverse speed
  bounds and braking arithmetic, and replaced approximate pi in reverse heading
  calculation. Corrected the impossible backup-distance comparison and scoped
  that guard to non-reversing differential drive.
- Signed axis bounds contain zero, differential drive always commands zero Y,
  and total XY speed is capped with `hypot(vx, vy)`. Coordinated XY interpolation
  preserves both the total cap and per-axis acceleration bounds during direction
  changes. Hard speed-limit reductions take precedence over smooth deceleration.
- Reworked obstacle prediction to include signed X/Y motion and rotation. The
  full convex padded footprint, including filled interior and boundary cells, is
  checked against high, inscribed, lethal and unknown costs and map boundaries.
  Candidate and measured velocities are checked from the current pose through a
  reaction interval and braking trajectory. Existing cost-gradient tracking is
  retained; no alternative avoidance trajectory generator was added.
- Implemented absolute and percentage Nav2 speed restrictions, preserving
  configured maxima and restoring them on `NO_SPEED_LIMIT` (0.0). Invalid
  restrictions are ignored without clearing an active restriction. Angular
  limits remain configured separately.
- Corrected warning throttle intervals and debug-path frame/timestamp handling.
  Debug publication no longer dereferences an absent odometry message.
- Enabled C++17 and restored installation of the example configuration directory.

### Modified and added files

| File | Purpose |
| --- | --- |
| `src/NeoLocalPlanner.cpp` | TF, numerical, state, parameter, timing, collision and speed-limit fixes |
| `include/NeoLocalPlanner.hpp` | Supporting declarations and state; Nav2 reset override |
| `CMakeLists.txt` | C++17 and example configuration installation |
| `config/neo_local_planner.yaml` | Controller configuration fragment for `/nav/controller_server` |
| `IKI_Changelog.md` | Change record, migration guidance and validation scope |

`README.md` is unchanged from its original contents. Package regression-test
files, their CMake targets and test-only friend access were removed at the user's
request. The later integration harnesses and logs are outside the package, in the
workspace's session storage.

### Configuration and migration

Merge `config/neo_local_planner.yaml` into the existing Nav2 configuration. It is
installed under `share/neo_local_planner2/config`. Retain the robot's existing
costmaps, actual dynamic footprint, goal checker and progress checker. Example
limits are not measured MPO-700 braking performance.

- Under `/nav`, configure literal `local_frame: odom` and `base_frame: base_link`
  for an unprefixed TF tree. A prefixed tree requires explicit values such as
  `robot1/odom` and `robot1/base_link` in both controller and costmap. Do not add a
  leading slash to TF frame IDs. Controller frames must match the costmap's
  `global_frame` and `robot_base_frame`; defaults come from the costmap.
- The example selects `/odom` and stamped commands, matching the inspected
  simulator. Keep the command type/remaps appropriate to the installation.
  Relative `odom` resolves under the node namespace; absolute `/odom` does not.
- Use signed `min_vel_x` and `min_vel_y` at or below zero. Positive minima are
  rejected because they prevent zero commands. Reverse X respects `min_vel_x`;
  use the negative of `max_vel_x` for a symmetric range.
- Prefer `max_vel_trans` and `min_vel_trans`. `max_trans_vel` and `min_trans_vel`
  remain deprecated startup-only aliases; conflicting explicit values fail
  configuration. Use canonical names for runtime updates. Alias parameter values
  are startup snapshots, not live mirrors.
- Review omitted parameters. Old declarations used 0.2 for nearly every numeric
  default, hiding distinct retrieval defaults. `numericParameters()` is now the
  authoritative default/member table. `max_vel_trans` and `trans_stopped_vel`
  retain configured values instead of being overwritten.
- Numeric values must be finite. Signed axis ranges contain zero; translation and
  rotation minima and stopped thresholds must not exceed their maxima. X/yaw
  acceleration, lookahead distance and `max_control_dt` must be positive; omni
  drive also requires positive Y acceleration. Gains/distances are nonnegative,
  `low_pass_gain` is in [0,1], and `max_cost` is in (0,1]. Emergency X braking must
  be at least the normal X acceleration magnitude.
- Numeric parameters and drive-mode booleans support atomic runtime updates.
  Frames, topics and aliases require cleanup/reconfiguration.
- Set `max_control_dt` above the normal control period. Its default is 0.5 s;
  intervals are not silently capped at 100 ms. The node clock and `use_sim_time`
  govern acceleration timing and reset recovery.
- Speed limits cap total XY velocity, not each axis independently. Absolute m/s
  and percentage restrictions preserve configured maxima; 0.0 restores them.
  A percentage follows later `max_vel_trans` updates; an absolute cap stays
  absolute. Angular speed retains its configured bound.
- Keep `publish_zero_velocity: true`. Jazzy publishes zero during
  `NoValidControl` failure tolerance; TF/path errors abort immediately, and
  terminal zero publication depends on that setting. Plugin diagnostics are
  throttled; repeated warnings from the server are outside this plugin.
- Legacy `rot_stopped_vel`, `yaw_goal_tolerance` and `goal_tune_time` remain
  accepted and validated. Goal completion and stopped-velocity checks remain the
  Nav2 goal checker's responsibility.

Selected defaults:

| Parameter | Default |
| --- | --- |
| `min_vel_x`, `max_vel_x` | -0.1, 0.5 m/s |
| `min_vel_y`, `max_vel_y` | -0.5, 0.5 m/s |
| `min_vel_trans`, `max_vel_trans` | 0.1, 0.5 m/s |
| `min_rot_vel`, `max_rot_vel` | 0.1, 0.5 rad/s |
| `acc_lim_x`, `acc_lim_y`, `acc_lim_theta` | 0.5 in the respective acceleration units |
| `emergency_acc_lim_x` | 0.5 m/s² |
| `trans_stopped_vel`, `rot_stopped_vel` | 0.05 m/s, 0.05 rad/s |
| `max_control_dt` | 0.5 s |
| `differential_drive`, `allow_reversing` | true, false |

The translation/rotation minima are cost-based floors for situational maxima;
they do not force nonzero commands at zero error.

### Validation

Environment: ROS 2 Jazzy; installed `nav2_controller` and `nav2_core`
1.3.14-1noble.20261109; `nav2_costmap_2d` 1.3.14-1noble.20261105.

- `colcon build --packages-select neo_local_planner2 --symlink-install
  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`: passed.
- `colcon test --packages-select neo_local_planner2`: completed; no package tests
  remain after the requested removal. This command is not the source of the
  integration-case counts below.
- Before removal, 14 in-memory regression cases passed, covering namespace/TF
  handling, invalid inputs, paths, timing, parameter transactions, speed limits,
  collision prediction, coordinated acceleration, differential/reverse operation
  and recovery. That suite is no longer shipped with the package.
- After Nav2 installation, 20 actual controller-server/FollowPath integration
  cases passed in isolated ROS domain 217. The harness used a synthetic map,
  odometry, simulation clock and kinematic plant, and verified that the server
  loaded this workspace's built library. Cases covered signed XY/yaw goal
  convergence; empty/NaN/Inf paths; missing TF; failure tolerance and recovery;
  clock pause/backward reset; absolute/percentage/restored speed limits; walls in
  both signs of X/Y; unknown cells; combined translation/rotation; map boundaries;
  prefixed TF frames; differential operation; and inscribed costs.
- In the moving-robot obstacle-injection case, the first zero command arrived
  approximately 49 ms after injection. Seven zero commands were observed over
  about 0.30 s before `PATIENCE_EXCEEDED`; stale nonzero commands were not reused.
- Absolute, 50-percent and removed restrictions produced total XY peaks of
  0.10, 0.15 and 0.30 m/s, respectively, with a configured 0.30 m/s maximum.
- Synthetic lethal/unknown/boundary tests did not cross the tested footprint
  boundary; minimum geometric clearance was about 1 cm without inflation.
  The inscribed/inflation case stopped with about 21 cm geometric clearance.
  These are fixture-specific observations, not hardware stopping guarantees.
- The existing Gazebo simulator in domain 28 was inspected read-only before
  motion: the base command subscriber was the Gazebo bridge, its configuration
  was a simulator configuration, and the robot description contained Gazebo
  velocity-control and hardware plugins.
- Six bounded 20 cm FollowPath goals passed in that Gazebo simulator, exercising
  both signs of body X and Y at no more than 0.10 m/s. Each completed with a zero
  command; final errors were below 2.5 cm. The robot returned within about 2.4 cm
  of its original position. A separate validation controller loaded this
  workspace's library and used the live TF, odometry, conservative costmap mirror
  and actual 11-vertex padded dynamic footprint. The existing Nav2 configuration
  was not changed. All validation nodes were shut down afterward.
- The first Gazebo preflight conservatively rejected an axis-aligned bounding
  box that included obstacles outside the triangular manipulator extension.
  Costmap inspection confirmed no lethal cell centers inside the actual convex
  footprint. The preflight was corrected to check that convex polygon with a
  5 cm margin, retaining blocked/unknown-cell rejection, before any motion.

Evidence is stored outside the package under
`/root/ros2_ws/codex/codex_storage/01a20dc0-63c7-7442-932d-d4879f79e033/nav2_integration_2026_11_24/`:
`full.result.json`, per-case command traces/server logs, `gazebo.result.json`,
Gazebo command/odometry traces, and simulator inspection snapshots.

### Remaining limitations

The executed runtime checks are simulation/container evidence (L3), not physical
robot commissioning. No physical hardware actuation, measured MPO-700 stopping
trial, Jetson performance benchmark or Debian buildfarm run was performed.

Prediction is sampled, using at most 50 ms and half a costmap cell of maximum
footprint-point travel per step, with a 10,000-step budget. It requires an accurate
nondegenerate convex footprint, costmap, TF, odometry, achievable braking limits
and a working downstream command/stop chain. Inflation and footprint padding
provide additional clearance. The full-footprint policy can be conservative in
narrow inflated spaces. A failed prediction requests a stop; it does not find an
alternative avoidance trajectory. Passing these checks cannot guarantee that a
physical robot will not contact a wall.

## 2026-11-24 — Follow-up: original tuning and full-route stall

The first user-run long-route test exposed behavior not covered by the earlier
short, single-goal Gazebo checks. Those earlier passes do not establish that the
reported route works.

### Restored user tuning

`config/neo_local_planner.yaml` preserves all 34 supplied settings exactly,
including `acc_lim_x/y: 0.25`, `acc_lim_theta: 0.8`, X bounds -0.2/0.8,
Y bounds -0.5/0.5, `max_rot_vel: 0.8`, `max_trans_vel: 0.8`,
`min_trans_vel: 0.1`, `lookahead_time: 0.4`, `lookahead_dist: 1.0`,
`low_pass_gain: 0.2`, `max_cost: 0.95`, and `min_stop_dist: 0.2`.
The historical translation aliases remain supported. The subsection is named
`NeoLocalPlanner`, matching the existing selector. The fragment does not replace
the existing `controller_plugins` list or remove MPPI configuration. The example
controller frequency is 50 Hz, matching the supplied launch log.

Only explicit TF/topic settings and additional safety settings are supplied
alongside that tuning. The README remains unchanged.

### Corrected premature stuck rejection

The old stuck gate rejected the entire calculation from measured-motion
lookahead, negative buffered obstacle distance and a positive X cost gradient,
before checking the proposed command. This can reject a safe lateral escape next
to a forward obstacle. That gate is removed. Existing forward-distance braking
remains, and both measured and proposed XY/yaw stopping trajectories still pass
through the complete footprint collision checker before a command is returned.
Lethal/unknown-cell and boundary rejection remain enforced. Inflation semantics
are corrected separately below.

### Corrected duplicate simulation-clock handling

Nav2's wall-rate loop can execute twice before a new `/clock` timestamp arrives.
Previously each duplicate timestamp raised `NoValidControl` and cleared velocity
state, preventing acceleration when the control loop outran clock publication.
The controller now waits for at most `clock_wait_timeout` (default 0.05 seconds)
for ROS time to advance. Steady time bounds the wait only; control integration
continues to use the node's ROS clock. The costmap mutex is acquired after this
wait. A clock that remains frozen, jumps backward, or advances by an excessive
interval still rejects control and resets state. No stale nonzero command is
returned as a workaround. `clock_wait_timeout` is finite, nonnegative and no
greater than `max_control_dt`; setting it to zero disables the wait.

### Follow-up verification

The package build and source style checks passed. `colcon test` completed with
no package tests, consistent with the earlier requested removal. Four additional
actual controller-server integration cases passed using the supplied tuning:

- A dense multi-pose path with a 50 Hz control loop and approximately 25 Hz clock
  reached its goal, reached about 0.53 m/s, and produced no ordinary time-delta
  failures.
- An inflated forward obstacle with small measured forward motion allowed a
  collision-checked lateral command instead of the premature stuck rejection;
  lateral displacement was about 4.7 cm during the bounded observation.
- Paused and backward-jumping simulation clocks produced zero commands and
  recovered after valid clock progress.
- An invalid numerical path was rejected with `INVALID_PATH` and zero commands.

These checks are stored in `regressions.result.json` and associated traces in the
same session storage as the earlier integration evidence. Startup TF availability,
laser QoS compatibility, initial global-map bounds and keepout warnings in the
supplied log are separate bringup/data issues; these controller changes do not
claim to fix those components.


### Corrected Nav2 inflation semantics and redundant TF lookup

Retracted: the initial implementation treated high/inscribed inflation costs at
ANY footprint cell as hard occupied geometry. That double-counted the robot's
inflation margin and could stall collision-free motion in inflated passages.
The corrected checker rejects lethal and unknown cells throughout the filled
footprint and rejects map-boundary crossings. Inscribed/configured inflation
thresholds apply at the robot center. This follows the distinction used by
[Jazzy Nav2's footprint collision checking](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_regulated_pure_pursuit_controller/src/collision_checker.cpp).
The configured `max_cost: 0.95` was not changed.

A regression with inscribed cells under the footprint, a free robot center and
20 cm physical wall clearance now reaches a lateral goal. Actual lethal/unknown
footprint intersections and map-boundary crossings continue to reject control.

The separate `base_link <- path_frame` lookup was also redundant: Nav2 already
supplies the robot pose. A newly started controller can have no overlapping TF
history for that composite chain even though `odom <- map` and the supplied robot
pose are valid. Robot-relative reverse-path selection now uses the inverse of
the validated supplied local robot pose and the transformed local path. Required
path/local and pose/local TF failures still abort with `ControllerTFError`.

### Full-route Gazebo follow-up

A separate controller server using the current workspace library and original
supplied tuning completed the 146-pose Navfn path toward map goal
`(-2.32, -2.13)` in the running Gazebo simulator. It traveled approximately
3.46 m, returned `NONE`/success, and ended with zero commands. A temporary Nav2
absolute translational speed restriction limited this test to 0.10 m/s without
changing the configured tuning. The test used the live TF, odometry, actual
11-vertex dynamic footprint and a graded mirror of the local costmap. The
mirror preserved lethal/unknown geometry instead of promoting inflated costs to
lethal obstacles. Existing simulator/Nav2 configuration was unchanged, and the
validation controller was shut down afterward.

Initial long-route attempts stopped safely with a redundant-TF-chain error and
then with insufficient progress under the overconservative inflation policy;
those attempts were failures, not passes. The successful follow-up followed the
corrections above. Evidence is in `gazebo_long_route.result.json`, its command and
odometry traces, and its server log in session storage.

Five focused integration checks using the supplied tuning now pass, including
the added inflation-semantics regression. The latest results are in
`regressions_final.result.json`; these supersede the earlier four-case follow-up
result. Build and style checks passed. Physical robot commissioning and stopping
distance remain unverified.

Final follow-up validation passed all 26 isolated integration checks (20 safety,
5 original-tuning regressions and 1 reverse-path check), plus the full Gazebo
route above. The reverse-path check respected the original -0.20 m/s lower X
limit with zero lateral commands in differential mode. Latest synthetic wall
checks retained positive physical clearance (minimum approximately 2 mm); this
supersedes earlier clearance measurements and is not a hardware stopping-margin
claim. `colcon build` and the subsequent `colcon test` completed successfully;
there are no registered package tests after the requested test-file removal.
README remains unchanged. No commit was created.
