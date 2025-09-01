// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOCAP4R2_ROBOT_GT__SETGTNODE_HPP_
#define MOCAP4R2_ROBOT_GT__SETGTNODE_HPP_


#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vector>

#include "mocap4r2_msgs/msg/rigid_body.hpp"
#include "mocap4r2_robot_gt_msgs/srv/set_gt_origin.hpp"

#include "rclcpp/rclcpp.hpp"


namespace mocap4r2_robot_gt
{
class SetGTNode : public rclcpp::Node
{
public:
  explicit SetGTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void set_gt(std::vector<double> init_pose);

protected:
  rclcpp::Client<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>::SharedPtr set_gt_origin_cli_;
};

}  // namespace mocap4r2_robot_gt

#endif  // MOCAP4R2_ROBOT_GT__SETGTNODE_HPP_
