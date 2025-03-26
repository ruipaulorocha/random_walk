# `random_walk`
This ROS package provides a node that makes a robot doing Brownian movements while avoiding obstacles.

#### Description
The package `random_walk` contains a node that makes a differential drive mobile robot executing Brownian movements in a 2D planar workspace using laser rangefinder data. The robot's navigation provided by the node does not involve any kind of path planning and is fully reactive. The algorithm can be briefly summarized as follows:
- Robot goes straight at maximum speed (and null angular speed) equal to `linear_speed` if no obstacles are detected closer than distance `safe_distance_th`.
- If obstacles are detected at a distance `d` such as `critical_distance_th < d < safe_distance_th`, the robot's linear speed is reduced proportionally in ramp such that it would be `alpha * linear_speed` for a distance close to `critical_distance_th`.
- For `d <= critical_distance_th`, the robot's linear speed is null and the robots rotates about itself, clockwise or anti-clockwise depending on nearest obstacle's angle, with angular speed equal to `angular_speed` (absolute value), about a random angle bounded by `max_delta_yaw` (in absolute value).
- The robot resumes straight motion after finding a direction wherein the distance to the nearest obstacle is greater than `critical_distance_th`.


#### Nodes

##### `random_walk`

###### Subscribed Topics
- `laser` (`geometry_msgs::msg::PoseWithCovarianceStamped` message type)
    - Scan provided by a 2D laser rangefinder.

###### Published Topics
- `cmd_vel` (`geometry_msgs::msg::Twist` message type if `stamped` is false / `geometry_msgs::msg::TwistStamped` message type if `stamped` is true)
    - Velocity commands to the robot.

##### Parameters
- `linear_speed` (double, default: 1.0)
    - meter/second
- `angular_speed` (double, default: 1.0)
    - radian/second
- `safe_distance_th` (double, default: 1.0)
    - meter
- `critical_distance_th` (double, default: 0.25)
    - meter
- `alpha` (double, default: 0.25)
    - ratio between 0.0 and 1.0
- `max_delta_yaw` (int, default: 180)
    - Maximum yaw variation
    - degrees
- `lidar_yaw_offset` (double, default: 0.0)
    - Rotation angle about zz axis of the `laser` frame w.r.t. the `base_link` frame
    - degrees
- `stamped` (boolean, default: false)
    - Whether to publish stamped twist messages