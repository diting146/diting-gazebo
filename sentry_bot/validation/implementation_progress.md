# Implementation and simulation acceptance (2026-10-31)

Implemented and validated the independent `sentry_bot` business workspace. Four packages own interfaces, motion arbitration/tunnel control, opening mission sequencing and business bringup. The console loads and edits the three-point route and starts/cancels tasks. Git is initialized locally; no commit or remote was requested.

Localization stays Point-LIO + small_gicp; no z_nav or AGV runtime dependency. Nav2 controls open areas and a closed-loop line controller handles the tunnel. The original robot model, lidar mount and sensor coordinates remain unchanged. The initial analytic roof-interference conclusion was incorrect and withdrawn.

## Passing evidence

- All four packages build; business colcon tests: 12 passed, zero failures/skips. Simulation focused tests: 4 passed. Simulation workspace aggregate: 467 tests, zero errors/failures, 25 skips (includes historical results).
- `runtime_acceptance_current.json`: one uninterrupted LIO suite, 10 complete openings + 10 forward + 10 reverse traversals, all passed. Maximum follow lateral error 0.019575 m; minimum independently measured side-wall margin 0.096366 m. Reference pose is diagnostic only.
- `motion_safety.json`: eight isolated real-ROS fault-injection checks passed, including wrong source/cache, stop-generation race, command/heartbeat/stale-odometry/persistent-data loss and temporary zero-output hold.
- `nav_adapter_checks.json`: three command conversion/freshness checks passed.
- `live_stop_checks.json`: moving cancellation after 0.0520 m measured travel, stationary confirmed about 1.045 s after cancel; verified physical tunnel blocker caused stop before collision; removal confirmed.
- `chassis_command_graph_current.txt`: exactly one `/cmd_vel` writer, the arbiter, with the console running.
- `operator_console_final.jpg`: final clean business launch ready, stationary and default route visible; `operator_console.jpg`: browser visual check of route controls and point editing. Defaults come from YAML, frozen per mission. Business mode blocks ordinary navigation and spin commands.

## Integration details

Nav commands are converted directly using fresh chassis odometry; the upstream legacy cached converter keeps TF ownership but its velocity output is diverted. Original ordinary-launch routing remains the default. Business gimbal TF is capped at 100 Hz to prevent listener backlog, preserving timestamps, angles and all mount coordinates. Startup waits for active navigation before ownership handoff. Stop invalidates queued handoffs; late child goals are canceled and block reuse until terminated.

Earlier failed probes remain in this directory for diagnosis and do not count toward acceptance. The ground-truth debug action completed, but its report write failed due to an incorrectly supplied filename; its console log is diagnostic evidence only.

Acceptance covers the measured upper-left UL passage in the current simulator. Real hardware, referee integration and three-second occupation timing are outside this implementation. No robot geometry, map radius or upstream planning algorithm was changed to force passage.

Final state: clean integrated business launch and console running, navigation ready, owner stop, stationary confirmed, no leftover isolated safety-test nodes. Final graph in `chassis_command_graph_final.txt` confirms one chassis writer.
