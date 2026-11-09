# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 Humble / Gazebo Fortress colcon workspace. This repository vendors source snapshots under `src/`; original repositories and revisions are recorded in `UPSTREAM.md`.

Current supported simulation entry: `sentry_sim_bringup dual_sentry_simulation_launch.py`. It creates one autonomous sentry and one keyboard-controlled obstacle robot. Default navigation is Point-LIO + small_gicp + YAstar. The fixed route and simulated HP/low-HP return are enabled by default; spin is disabled by default. Explicit spin is deferred until the sentry has fully exited the tunnel during the fixed route. See `docs/当前构建与依赖说明_2026-11-06.md` for the scoped build and dependency instructions.

- `src/rmu_gazebo_simulator/rmu_gazebo_simulator/`: simulator launch files, YAML configuration, Python referee nodes, and `resource/` worlds/models. The current configuration spawns two rm25 robots in `diting_3v3_field`.
- `src/rmoss_gazebo/`: Gazebo plugins, robot control interfaces, camera integration, and manual test scripts.
- `src/pb2025_sentry_nav/`: Nav2 bringup, controllers, localization, odometry, and terrain processing. Navigation maps and parameters live in `pb2025_nav_bringup/`.
- `src/pb2025_robot_description/`: robot XMacro descriptions and visualization resources.
- `sentry_nav/src/sentry_sim_bringup/`: the current dual scene, fixed-route task, simulation gateways, maps and navigation parameters.
- `sentry_nav/src/sentry_tunnel_bt/`: tunnel behavior-tree plugins. The overlay also contains 15 package names duplicated under the root `src/`; always limit discovery to the intended workspace's `src/`.
- `build/`, `install/`, and `log/`: generated colcon outputs; edit source files instead.

## Build, Test, and Development Commands

Prefer `bash "$WS/tools/build_current.bash" all --check` to verify the current package selection and installed paths, then use `root`, `overlay`, or `all` without `--check` only when an actual build is needed and dependencies are installed. The script creates an isolated environment per layer and never clears existing outputs. See `docs/包归属与受控构建_2026-11-06.md` for ownership and scope; `--check` is not a build or runtime acceptance test. Detailed manual commands remain below.

Use a new Bash terminal with only ROS sourced, then build the root layer and overlay separately. Do not source `source_all.bash` before rebuilding the root layer: that would put the overlay in the root build's parent environment. The current supported entry needs these six root packages; this is not an all-upstream-packages build:

```bash
source /opt/ros/humble/setup.bash
export WS=/home/hui/diting-gazebo-feat-sentry-nav-simulation
cd "$WS"
colcon build --base-paths src --symlink-install \
  --packages-select rmoss_gz_plugins rmu_gazebo_simulator pb2025_robot_description \
  pb_omni_pid_pursuit_controller tdt_yastar_planner sentry_planner_server \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$WS/install/setup.bash"
cd "$WS/sentry_nav"
colcon build --base-paths src --symlink-install \
  --packages-up-to sentry_sim_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$WS/source_all.bash"
ros2 launch sentry_sim_bringup dual_sentry_simulation_launch.py use_rviz:=True
```

Install dependencies first using the scoped commands in `docs/当前构建与依赖说明_2026-11-06.md`; a blanket rosdep/build over all source snapshots includes optional legacy RMOSS dependencies. The overlay selection currently contains 17 local packages. For focused incremental builds, use `--packages-select <package>` in its owning layer only when dependencies are already built. The launcher starts Gazebo running; resume playback if it has been paused manually.

## Coding Style & Naming Conventions

Follow each package's existing conventions and formatter configuration. Use four-space Python indentation; C++ packages with Google-based `.clang-format` generally use two spaces. Keep package/file names in `snake_case`, C++ classes in `PascalCase`, and launch filenames consistent with neighboring files. Run `pre-commit run --all-files` inside the affected Git repository where configured; hooks include Ruff, clang-format, codespell, and ament checks.

## Testing Guidelines

Run `colcon test --packages-select <package>` followed by `colcon test-result --verbose`. Several packages register ament lint checks under `BUILD_TESTING`; no workspace-wide coverage threshold is defined. Follow existing `test/` and `test_*.py` patterns for new tests. RMOSS command scripts are manual simulation checks. For behavior changes, verify robot spawning, namespaced topics, motion, and referee HP behavior as applicable; record reproduction commands.

## Commit & Pull Request Guidelines

Commit workspace changes here and preserve upstream license notices. Upstream history mixes imperative subjects such as `Update lidar pose in sentry_robot` with scoped prefixes such as `ci(mergify): ...`; use a concise, descriptive subject consistent with that project. Keep changes focused. PRs should explain behavior changes, link relevant issues, list validation commands/results, and include screenshots or recordings for visible simulation changes. Follow the affected project's `CONTRIBUTING.md` when present.
