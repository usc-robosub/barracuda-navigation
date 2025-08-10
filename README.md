# barracuda-navigation

## User Guide

Start up docker container: ```docker compose up [-d]```

- ```-d``` flag runs container in detached mode

Gracefully shutdown container & remove image:
```docker compose down --rmi local --remove-orphans```

### Service Interface

The planner provides a ROS service for path planning:

**Service**: `/plan_path`
**Type**: `rrt_star_planner/PlanPath`

**Request**:

```ros
geometry_msgs/PoseStamped goal_pose
```

**Response**:

```ros
barracuda_msgs/Waypoints waypoints
```

### Topics

#### Subscribed Topics

- `/robot_pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  - Current robot position

- `/obstacle_pointcloud` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
  - Point cloud representing obstacles in the environment

#### Published Topics

- `/planned_path` ([barracuda_msgs/Waypoints](https://github.com/usc-robosub/barracuda_msgs/tree/main))
  - The planned path as a series of waypoints

- `/planned_path_viz` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  - Visualization of the planned path for RViz

- `/rrt_tree` ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html))
  - Visualization of the RRT tree structure

### Waypoint Interpolation

The navigation system includes a waypoint interpolation node that converts discrete RRT* waypoints into smooth, high-resolution paths suitable for robot control.

#### Features

- **Smart Interpolation**: Uses linear interpolation for 2 waypoints, cubic spline for 3+ waypoints
- **Speed Control**: Ensures path execution never exceeds maximum safe speeds
- **High Resolution**: Generates dense waypoint sequences from sparse RRT* output
- **Constant Timing**: Provides consistent time intervals for smooth controller operation

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~velocity` | double | 1.0 | Base velocity for time parameterization (m/s) |
| `~max_speed` | double | 2.0 | Maximum allowed speed safety limit (m/s) |
| `~base_dt` | double | 0.1 | Controller time step for speed estimation (s) |
| `~output_dt` | double | 0.02 | High-resolution sampling interval (s) |

#### Topics

**Subscribed:**
- `/rrt_waypoints` ([barracuda_msgs/Waypoints](https://github.com/usc-robosub/barracuda_msgs/tree/main))
  - Sparse waypoints from the RRT* planner

**Published:**
- `/interpolated_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  - High-resolution interpolated path with speed control

#### How It Works

1. **Speed Estimation**: Analyzes waypoint distances to estimate maximum speeds
2. **Global Scaling**: If any segment would exceed `max_speed`, the entire path timing is scaled proportionally
3. **Interpolation**: Creates smooth curves (linear or cubic spline) between waypoints
4. **High-Resolution Sampling**: Generates dense waypoints at constant `output_dt` intervals
5. **Safety**: Guarantees speeds remain within limits while maintaining smooth motion

### Target Odometry Publisher

The `target_odometry_publisher` node converts the `interpolated_path` into a constant-speed target trajectory for downstream controllers (e.g., an LQR). It selects a lookahead waypoint along the path and publishes the corresponding pose and linear velocity as `nav_msgs/Odometry`.

#### Node

- Executable: `barracuda_navigation/scripts/target_odometry_publisher.py`

#### Subscribed Topics

- `interpolated_path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
- `odometry/filtered/global` ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))

#### Published Topics

- `target_odometry` ([nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html))
  - Note: When launched inside the `barracuda` namespace, the fully-qualified topic appears as `/barracuda/target_odometry`.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~path_topic` | string | `interpolated_path` | Input path topic |
| `~odom_topic` | string | `odometry/filtered/global` | Input robot odometry topic |
| `~target_topic` | string | `target_odometry` | Output target odometry topic |
| `~frame_id` | string | `map` | Output frame if path has no frame |
| `~desired_speed` | double | `0.8` | Constant translational speed (m/s) |
| `~lookahead_dist` | double | `1.5` | Lookahead distance along path (m) |
| `~max_lookahead_dist` | double | `3.0` | Cap for lookahead distance (m) |
| `~publish_rate` | double | `30.0` | Publishing rate (Hz) |
| `~hold_at_end` | bool | `true` | Hold final waypoint; publish zero linear velocity within `~hold_distance` |
| `~hold_distance` | double | `0.3` | Distance threshold to trigger final hold (m) |

#### Behavior

1. Finds the nearest path index to the current position and enforces forward progress (no backtracking).
2. Chooses a target index where the along-path distance from the nearest index ≥ `lookahead_dist` (capped by `max_lookahead_dist`).
3. Publishes:
   - Pose: position with orientation aligned to the 3D path tangent (roll=0, pitch from slope, yaw from XY).
   - Twist (linear): constant speed along the local 3D path tangent (vx, vy, vz); zeroed when holding at the final waypoint.
   - Twist (angular): zeros (attitude control handled by your controller).

#### Launch

- `barracuda_navigation/launch/waypoint_interpolator_demo.launch` runs:
  - `waypoint_interpolator`
  - `waypoint_test_publisher`
  - `target_odometry_publisher`

- `barracuda_navigation/launch/barracuda_navigation.launch` runs:
  - `rrt_star_planner`
  - `target_odometry_publisher`

Example (demo):

```
roslaunch barracuda_navigation waypoint_interpolator_demo.launch
```

Then monitor:

```
rostopic echo /barracuda/target_odometry
```
