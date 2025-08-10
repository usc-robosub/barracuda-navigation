# Repository Guidelines

## Project Structure & Module Organization
- `catkin_ws/src/barracuda_navigation/`: Core ROS package (Python nodes under `scripts/`, launches under `launch/`).
- `catkin_ws/src/rrt_star_planner/`: RRT* planner (C++ in `src/`, headers in `include/`, service in `srv/`, test script in `scripts/`).
- `catkin_ws/src/barracuda_msgs/`: Message definitions used across nodes.
- Tooling: `Dockerfile`, `docker-compose.yaml`, `entrypoint.sh` for ROS Noetic on Ubuntu 20.04.

## Build, Test, and Development Commands
- Build (container): `docker compose up [-d]` — builds the image and launches the stack.
- Stop/clean: `docker compose down --rmi local --remove-orphans` — remove container and local image.
- Build (host): `cd catkin_ws && catkin_make && source devel/setup.bash` — local catkin build.
- Run demo: `roslaunch barracuda_navigation waypoint_interpolator_demo.launch` — interpolator + test publisher + target odom.
- Full stack: `roslaunch barracuda_navigation barracuda_navigation.launch` — planner + target odom.
- Quick check: `rostopic echo /barracuda/target_odometry` — verify outputs.

## Coding Style & Naming Conventions
- Python: 4-space indent, PEP 8; filenames and ROS node scripts in `snake_case.py`.
- C++: consistent brace style, header guards/pragma once; keep functions short and focused.
- ROS topics/params: lower_snake_case; package and launch names match existing patterns (e.g., `barracuda_navigation`, `*_demo.launch`).
- Keep nodes small and composable; prefer parameters over hardcoded constants.

## Testing Guidelines
- Demo-driven testing via `waypoint_interpolator_demo.launch` and topic inspection (RViz optional).
- Planner script: `catkin_ws/src/rrt_star_planner/scripts/test_rrt_star_planner.py` for basic sanity checks.
- Add new tests alongside package code (`scripts/` for Python utilities, `src/` for C++). Aim for deterministic behavior and log key metrics.

## Commit & Pull Request Guidelines
- Commits: short, imperative, present tense (e.g., "add target odom publisher", "update submodules"). Group related changes.
- PRs: clear description, linked issues, what/why summary, launch/test instructions, and screenshots or topic dumps when relevant (e.g., RViz path/odom).
- CI/CD: container builds from `Dockerfile`; ensure `catkin_make` succeeds and key launches run without errors.

## Security & Configuration Tips
- Container runs with `network_mode: host` and `privileged: true`; avoid secrets in code or params.
- Always `source /opt/ros/noetic/setup.bash` and the workspace `devel/setup.bash` before running tools.
