# Repository Guidelines

## Project Structure & Module Organization

This ROS 2 Humble workspace contains six packages under `src/`:

- `jaka_kargo_driver`: C++ controller/AGV bridge, SDK headers in `include/`, and bundled SDK library in `lib/`.
- `jaka_kargo_planner`: MoveIt trajectory action server and SDK integration.
- `jaka_kargo_msgs`: custom interfaces in `msg/` and `srv/`.
- `jaka_kargo_description`: URDF/USD models, STL meshes, and RViz assets.
- `jaka_kargo_moveit_config`: planning groups, controllers, Xacro, and launch configuration.
- `jaka_kargo_isaacsim`: Python simulation scripts and launch files.

Package launch files live in `launch/`; configuration lives in `config/` where present. Treat `build/`, `install/`, and `log/` as generated output.

## Build, Test, and Development Commands

Run from the workspace root with ROS 2 Humble and MoveIt dependencies installed:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install       # Build all packages
source install/setup.bash            # Load workspace packages
colcon test                         # Run registered tests/lint checks
colcon test-result --verbose         # Inspect test results
```

Use `colcon build --symlink-install --packages-up-to jaka_kargo_planner` for a focused build including workspace dependencies.

Start the RViz simulation with:

```bash
ros2 launch jaka_kargo_moveit_config demo.launch.py use_rviz_sim:=true
```

For Isaac Sim, launch `ros2 launch jaka_kargo_isaacsim run_kargo_isaacsim.launch.py`, then launch the MoveIt demo with `use_isaac_sim:=true`.

## Coding Style & Naming Conventions

Use C++17 and four-space indentation in C++/Python; follow existing two-space CMake/YAML indentation. Preserve nearby brace and naming styles. Use `snake_case` for packages, Python functions, parameters, and launch filenames (`*.launch.py`); use PascalCase for interface files such as `AgvGetCurrentMap.srv`. Register new interfaces in `jaka_kargo_msgs/CMakeLists.txt`. No repository formatter configuration exists; the description package uses `ament_lint_auto` and `ament_lint_common`.

## Testing Guidelines

There are no dedicated test files or coverage thresholds. Add regression tests under the affected package's `test/` directory, using `test_*.py` for Python tests, and register them with ament. Run the commands above and validate relevant planning groups in simulation. For motion changes, check joint ordering and unit conversions between ROS and SDK/AGV interfaces.

## Commit & Pull Request Guidelines

History mixes short imperative subjects with scoped messages such as `fix(planner): ...`; prefer concise, scoped subjects. PRs should describe behavior changes, affected packages, linked issues when applicable, and build/test results. State whether validation used RViz, Isaac Sim, or hardware; include screenshots for model or visualization changes. Update `README.md` when launch parameters or public interfaces change.
