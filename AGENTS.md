# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 Humble / Gazebo Fortress colcon workspace. This repository vendors source snapshots under `src/`; original repositories and revisions are recorded in `UPSTREAM.md`.

- `src/rmu_gazebo_simulator/rmu_gazebo_simulator/`: simulator launch files, YAML configuration, Python referee nodes, and `resource/` worlds/models. The current configuration spawns two rm25 robots in `diting_3v3_field`.
- `src/rmoss_gazebo/`: Gazebo plugins, robot control interfaces, camera integration, and manual test scripts.
- `src/pb2025_sentry_nav/`: Nav2 bringup, controllers, localization, odometry, and terrain processing. Navigation maps and parameters live in `pb2025_nav_bringup/`.
- `src/pb2025_robot_description/`: robot XMacro descriptions and visualization resources.
- `sentry_nav/src/sentry_sim_bringup/`: custom LIO/navigation launches, configuration, model generator, sensor adapters, and map assets; keep project-specific navigation behavior here.
- `build/`, `install/`, and `log/`: generated colcon outputs; edit source files instead.

## Build, Test, and Development Commands

Run from the workspace root in Bash:

```bash
source /opt/ros/humble/setup.bash
rosdep install -r --from-paths src --ignore-src --rosdistro humble -y
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

These commands load ROS, install declared dependencies, build packages, load the workspace, and launch simulation. For focused builds, add `--packages-select <package>` when dependencies are already built. Start Gazebo playback to advance simulation.

Build the navigation overlay separately from `sentry_nav/` with `--base-paths src` after sourcing the root installation. Load both with `source source_all.bash`. Default navigation uses sensor-only `lio`; `ground_truth` is an explicit debugging option. See README for navigation, mapping, and console commands. Do not commit raw mapping captures or generated colcon outputs.

## Coding Style & Naming Conventions

Follow each package's existing conventions and formatter configuration. Use four-space Python indentation; C++ packages with Google-based `.clang-format` generally use two spaces. Keep package/file names in `snake_case`, C++ classes in `PascalCase`, and launch filenames consistent with neighboring files. Run `pre-commit run --all-files` inside the affected Git repository where configured; hooks include Ruff, clang-format, codespell, and ament checks.

## Testing Guidelines

Run `colcon test --packages-select <package>` followed by `colcon test-result --verbose`. Several packages register ament lint checks under `BUILD_TESTING`; no workspace-wide coverage threshold is defined. Follow existing `test/` and `test_*.py` patterns for new tests. RMOSS command scripts are manual simulation checks. For behavior changes, verify robot spawning, namespaced topics, motion, and referee HP behavior as applicable; record reproduction commands.

## Commit & Pull Request Guidelines

Commit workspace changes here and preserve upstream license notices. Upstream history mixes imperative subjects such as `Update lidar pose in sentry_robot` with scoped prefixes such as `ci(mergify): ...`; use a concise, descriptive subject consistent with that project. Keep changes focused. PRs should explain behavior changes, link relevant issues, list validation commands/results, and include screenshots or recordings for visible simulation changes. Follow the affected project's `CONTRIBUTING.md` when present.


## Opening business overlay

`sentry_bot/src/` owns mission sequencing, motion arbitration, tunnel control and business bringup. Build it after the navigation overlay; `source_all.bash` optionally sources its install. `operator_console/` owns the standalone UI/backend. Keep generated build/install/log directories out of Git. The tested opening profile uses Point-LIO + small_gicp and the unchanged sentry model; do not introduce z_nav. See `sentry_bot/validation/implementation_progress.md` for acceptance provenance.
