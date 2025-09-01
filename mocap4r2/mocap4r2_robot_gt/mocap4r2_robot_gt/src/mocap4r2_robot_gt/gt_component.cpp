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

// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>

#include "mocap4r2_robot_gt/gt_component.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"

#include "rclcpp/rclcpp.hpp"

namespace mocap4r2_robot_gt
{

using std::placeholders::_1;
using std::placeholders::_2;

GTNode::GTNode(const rclcpp::NodeOptions & options)
: Node("mocap4r2_gt", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  rigid_body_sub_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::SensorDataQoS(), std::bind(&GTNode::rigid_bodies_callback, this, _1));
  set_gt_origin_srv_ = create_service<mocap4r2_robot_gt_msgs::srv::SetGTOrigin>(
    "~/set_get_origin", std::bind(&GTNode::set_gt_origin_callback, this, _1, _2));

  declare_parameter<std::string>("root_frame", "odom");
  declare_parameter<std::string>("robot_frame", "base_footprint");
  declare_parameter<std::string>("mocap_frame", "base_mocap");
  declare_parameter<std::vector<double>>("init_mocap4r2_xyzrpy", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  get_parameter("root_frame", root_frame_);
  get_parameter("robot_frame", robot_frame_);
  get_parameter("mocap_frame", mocap_frame_);

  std::vector<double> init_mocap4r2_coordinates;
  get_parameter("init_mocap4r2_xyzrpy", init_mocap4r2_coordinates);

  if (init_mocap4r2_coordinates.size() == 6u) {
    tf2::fromMsg(get_pose_from_vector(init_mocap4r2_coordinates), offset_);
  } else {
    RCLCPP_ERROR(get_logger(), "Error in init_mocap xyzrpy coordinates - setting all values to 0");
    tf2::fromMsg(get_pose_from_vector({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}), offset_);
  }
}

void
GTNode::rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{
  if (!valid_gtbody2robot_) {
    try {
      auto gtbody2robot_msg = tf_buffer_.lookupTransform(
        mocap_frame_, robot_frame_, tf2::TimePointZero);
      tf2::fromMsg(gtbody2robot_msg.transform, gtbody2robot_);
      valid_gtbody2robot_ = true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(
        get_logger(), "Transform %s->%s exception: [%s]",
        mocap_frame_.c_str(), robot_frame_.c_str(), e.what());
    }
  } else {
    mocap2gtbody_.setOrigin(
      tf2::Vector3(
        msg->rigidbodies[0].pose.position.x, msg->rigidbodies[0].pose.position.y,
        msg->rigidbodies[0].pose.position.z));
    mocap2gtbody_.setRotation(
      tf2::Quaternion(
        msg->rigidbodies[0].pose.orientation.x, msg->rigidbodies[0].pose.orientation.y,
        msg->rigidbodies[0].pose.orientation.z,
        msg->rigidbodies[0].pose.orientation.w));

    tf2::Transform root2robotgt;
    root2robotgt = offset_ * mocap2gtbody_ * gtbody2robot_;

    geometry_msgs::msg::TransformStamped root2robotgt_msg;
    root2robotgt_msg.header.frame_id = root_frame_;
    root2robotgt_msg.header.stamp = msg->header.stamp;
    root2robotgt_msg.child_frame_id = robot_frame_ + "_gt";
    root2robotgt_msg.transform = tf2::toMsg(root2robotgt);

    tf_broadcaster_->sendTransform(root2robotgt_msg);
  }
}

void
GTNode::set_gt_origin_callback(
  const std::shared_ptr<mocap4r2_robot_gt_msgs::srv::SetGTOrigin::Request> req,
  std::shared_ptr<mocap4r2_robot_gt_msgs::srv::SetGTOrigin::Response> resp)
{
  if (!valid_gtbody2robot_) {
    resp->success = false;
    resp->error_msg = "Pose still not valid setting origin";
    RCLCPP_ERROR(get_logger(), "%s", resp->error_msg.c_str());
    return;
  }

  tf2::Transform wish_gt;
  if (req->current_is_origin) {
    wish_gt.setOrigin({0.0, 0.0, 0.0});
    wish_gt.setRotation({0.0, 0.0, 0.0, 1.0});
  } else {
    wish_gt.setOrigin(
      tf2::Vector3(
        req->origin_pose.position.x, req->origin_pose.position.y, req->origin_pose.position.z));
    wish_gt.setRotation(
      tf2::Quaternion(
        req->origin_pose.orientation.x, req->origin_pose.orientation.y,
        req->origin_pose.orientation.z,
        req->origin_pose.orientation.w));
  }

  tf2::Transform root2robotgt;
  root2robotgt = mocap2gtbody_ * gtbody2robot_;

  offset_ = root2robotgt.inverse() * wish_gt;

  resp->success = true;
}


geometry_msgs::msg::Pose
GTNode::get_pose_from_vector(const std::vector<double> & init_pos)
{
  geometry_msgs::msg::Pose ret;

  if (init_pos.size() == 6u) {
    tf2::Quaternion q;
    q.setEuler(init_pos[3], init_pos[4], init_pos[5]);

    ret.position.x = init_pos[0];
    ret.position.y = init_pos[1];
    ret.position.z = init_pos[2];
    ret.orientation.x = q.x();
    ret.orientation.y = q.y();
    ret.orientation.z = q.z();
    ret.orientation.w = q.w();

    return ret;
  } else {
    RCLCPP_WARN(get_logger(), "Trying to get Pose for a wrong vector");
    return ret;
  }
}

}  // namespace mocap4r2_robot_gt

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mocap4r2_robot_gt::GTNode)
