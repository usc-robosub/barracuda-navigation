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
