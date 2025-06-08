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
