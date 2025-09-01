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

#include <vector>

#include "mocap4r2_robot_gt/set_gt_component.hpp"
#include "mocap4r2_robot_gt_msgs/srv/set_gt_origin.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_robot_gt
{

using namespace std::chrono_literals;

SetGTNode::SetGTNode(const rclcpp::NodeOptions & options)
: Node("set_gt", options)
{

  set_gt_origin_cli_ = create_client<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>(
    "/mocap4r2_gt/set_get_origin");
}

void
SetGTNode::set_gt(std::vector<double> init_pose)
{
  auto request = std::make_shared<mocap4r2_robot_gt_msgs::srv::SetGTOrigin::Request>();

  if (init_pose.empty()) {
    request->current_is_origin = true;
  } else {
    if (init_pose.size() == 6u) {
      tf2::Quaternion q;
      q.setEuler(init_pose[3], init_pose[4], init_pose[5]);

      request->origin_pose.position.x = init_pose[0];
      request->origin_pose.position.y = init_pose[1];
      request->origin_pose.position.z = init_pose[2];
      request->origin_pose.orientation.x = q.x();
      request->origin_pose.orientation.y = q.y();
      request->origin_pose.orientation.z = q.z();
      request->origin_pose.orientation.w = q.w();

      RCLCPP_INFO(
        get_logger(),
        "Request for resetting GT origin to (%lf, %lf, %lf) RPY(%lf, %lf, %lf)",
        request->origin_pose.position.x, request->origin_pose.position.y,
        request->origin_pose.position.z, init_pose[3], init_pose[4], init_pose[5]);

    } else {
      RCLCPP_ERROR(get_logger(), "Coordinates vector size is not 6 (x y z roll pitch yaw)");
      return;
    }
  }

  while (!set_gt_origin_cli_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "service not available, waiting again...");
  }

  auto result = set_gt_origin_cli_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(shared_from_this(), result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto resp = *result.get();
    if (resp.success) {
      RCLCPP_INFO(get_logger(), "Set GT origin");
    } else {
      RCLCPP_ERROR(get_logger(), "Could not set GT origin [%s]", resp.error_msg.c_str());
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
}

}  // namespace mocap4r2_robot_gt

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_robot_gt::SetGTNode)
