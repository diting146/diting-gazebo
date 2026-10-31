# Sentry business workspace

Status: implemented. The initial 0.2 m/s profile passed 10 opening missions and 10 traversals in each direction. The current 0.4 m/s profile passed two full openings and two traversals in each direction with Point-LIO + small_gicp. Physical-robot acceptance remains separate.

This Git repository owns business tasks and motion arbitration. It depends on the sibling `gazebo` and `sentry_nav` workspaces. Localization remains Point-LIO + small_gicp. No z_nav, z-localization or AGV runtime dependency is added.

## Build and launch

From the parent directory, source `source_all.bash`, then build in `sentry_bot` with `colcon build --symlink-install`. Source the parent script again after building. Start with:

```bash
ros2 launch sentry_business_bringup opening_simulation_launch.py
```

Business startup defaults to the measured tunnel entry (-4.1, 3.46), chassis yaw 0 radians, already aligned with the passage. Override `x`, `y`, `yaw` launch arguments if needed. Tunnel cruise is 0.4 m/s; the forward obstacle guard extends to 0.8 m from the chassis origin. Exit deceleration and convergence limits remain enabled.

The explicit debugging option is `localization:=ground_truth`. Do not run a second simulation alongside an existing instance. Launch the existing operator console separately using its launcher.

In the console, load the default three points, adjust them using the existing point list editor, then start the UL opening mission. The first two points define a directed centerline; the third is the center occupation goal. Version one supports only the measured top-left UL passage. Unsupported lines are rejected. The route is frozen in the action goal while running. Cancel/stop latches motion off; there is no automatic restart or reverse recovery.

## Packages and ownership

- `sentry_interfaces`: opening and tunnel actions, motion ownership service and state.
- `sentry_motion`: pure geometry, single chassis velocity writer, tunnel action server.
- `sentry_mission`: validated Nav2 / tunnel / Nav2 sequencing.
- `sentry_business_bringup`: simulation inclusion and business defaults.

In business mode, `nav_velocity_adapter` transforms each `/cmd_vel_nav2_result` command using fresh chassis odometry and publishes `/sentry/cmd_vel_nav`. Upstream `fake_vel_transform` retains virtual-TF ownership but its cached velocity output is diverted to `/sentry/cmd_vel_nav_legacy` and never selected; tunnel commands go to `/sentry/cmd_vel_tunnel`. Only the arbiter publishes `/cmd_vel`. `/sentry/set_motion_owner` accepts `stop`, `nav`, `tunnel`, requires confirmed stationary odometry before enabling, and clears cached commands. Requests carry an expected motion generation; an operator stop invalidates previously queued switches. `/sentry/stop` is a Trigger service. Motion requires `/sentry/mission_heartbeat`; stale odometry, heartbeat or commands latch stop. An owner has up to 3 seconds to produce its first command, during which output remains zero; subsequent commands expire after 0.5 seconds. External publishers must not bypass the arbiter.

The original simulation launch remains compatible and defaults to direct Nav2 control. Business launch disables idle spin in the simulated gateway. Point-LIO and terrain processing keep running through tunnel control. Reference lines are frozen in odom per traversal; local drift remains possible.

## Geometry and limitations

`tools/inspect_ul_geometry.py` reads the installed source field mesh and actual sentry collision primitives. `validation/ul_geometry.json` records initial measurement. The roof underside is approximately z=0.35 m; collision width is approximately 0.532 m. Collision primitives do not fully describe visual lidar/gimbal dimensions. A conservative analytic envelope was initially interpreted as interference; that conclusion was withdrawn after user correction and a successful unchanged-model opening mission. Keep the original lidar mount and transforms. Analytic bounds are not measured collision evidence; runtime acceptance uses the existing simulation model.

The static map uses a conservative 0.40 m navigation radius and pre-dilated obstacles; it may reject the tunnel itself. Tunnel mode does not bypass the need for real physical clearance. Entry/exit remain outside the roof span. No map pixels, robot radius, or upstream algorithms are silently reduced to force passage.

## Validation

Run the pure geometry, routing and ownership tests with the packages on PYTHONPATH. On this host use `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` because a user-installed anyio plugin conflicts with the system pytest. Build/test the custom simulation package as documented in the parent workspace. Runtime acceptance requires fresh odometry, a single map→odom→chassis TF chain, one command writer, blockage/cancel/lost-data tests, then ten LIO runs in each direction and ten full opening missions. Ground truth is diagnostic only. Save failures as well as successes under `validation/`.

### Current verification evidence

`validation/runtime_acceptance_current.json` records the original 0.2 m/s uninterrupted LIO suite: 10/10 complete openings, 10/10 forward and 10/10 reverse traversals. Maximum follow-stage lateral error was 0.01958 m across all 30 runs. Independent reference-pose measurements found at least 0.09636 m side-wall clearance. Reference poses never drive control. The robot model, lidar mounting, sensor coordinates, map radius and upstream navigation algorithms remain unchanged.

Twelve business tests and four simulation smoke/rate tests passed. `colcon test-result` for the business workspace reports 12 tests, no failures or skips. `validation/motion_safety.json` records eight isolated ROS fault-injection checks; `nav_adapter_checks.json` records three adapter checks. `live_stop_checks.json` records action cancellation and a verified physical blocker stopping the robot before collision; fixture removal is checked. `chassis_command_graph_current.txt` confirms the arbiter is the sole chassis writer with the console running.

Short observation delays command immediate zero velocity. The controller resumes only after fresh data returns within two seconds; persistent loss fails the mission. Operator stop, watchdog failure and action failure never automatically restart a mission. Startup waits for the navigation lifecycle before handing it ownership; stop and cancellation invalidate that wait.

Business-mode gimbal TF publication is limited to 100 Hz to avoid transform-listener backlog from 1000 Hz joint updates. Measured angles, sample timestamps and mount coordinates are preserved. Ordinary mode retains its prior unlimited setting.

Earlier failed probes are preserved for diagnosis; they are not passing acceptance evidence. The ground-truth debug run completed but its JSON write failed because of an incorrectly supplied report path; the retained console log is diagnostic only. No real-robot validation or referee/occupation-duration integration is claimed.

`validation/runtime_fast_profile.json` records the current 0.4 m/s profile: 2 complete openings and 2 traversals per direction, all passed. Maximum follow lateral error 0.02339 m. Runtime parameters and spawn pose are embedded in that report. The older 30-run result is not claimed for the faster profile.
