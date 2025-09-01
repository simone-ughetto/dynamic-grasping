#ifndef DYNAMIC_GRASPING_CONTROLLER_TRAJECTORY_POINT_HPP
#define DYNAMIC_GRASPING_CONTROLLER_TRAJECTORY_POINT_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace dynamic_grasping_controller
{

struct TrajectoryPoint {
  geometry_msgs::msg::Pose pose;
  geometry_msgs::msg::Twist twist;
};

} // namespace dynamic_grasping_controller

#endif // DYNAMIC_GRASPING_CONTROLLER_TRAJECTORY_POINT_HPP