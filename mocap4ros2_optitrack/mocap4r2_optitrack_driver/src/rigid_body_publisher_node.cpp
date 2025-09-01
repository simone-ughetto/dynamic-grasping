// Copyright 2024 Intelligent Robotics Lab
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

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class RigidBodyPublisherNode : public rclcpp::Node
{
public:
  RigidBodyPublisherNode()
  : Node("rigid_body_publisher_node")
  {
    // Subscribe to rigid bodies topic
    subscription_ = create_subscription<mocap4r2_msgs::msg::RigidBodies>(
      "rigid_bodies", 10,
      std::bind(&RigidBodyPublisherNode::rigid_bodies_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "RigidBodyPublisherNode initialized");
    RCLCPP_INFO(get_logger(), "Subscribing to /rigid_bodies");
    RCLCPP_INFO(get_logger(), "Publishing poses to /<rigid_body_name>/pose");
  }

private:
  void rigid_bodies_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
  {
    // Process each rigid body
    for (const auto & rb : msg->rigidbodies) {
      const std::string & rb_name = rb.rigid_body_name;
      
      // Create publisher if it doesn't exist for this rigid body
      if (publishers_.find(rb_name) == publishers_.end()) {
        RCLCPP_INFO(get_logger(), "Creating publisher for rigid body: %s", rb_name.c_str());
        std::string topic_name = "/" + rb_name + "/pose";
        publishers_[rb_name] = create_publisher<geometry_msgs::msg::PoseStamped>(
          topic_name, 10);
      }
      
      // Create and publish PoseStamped message
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = msg->header;  // Use the same header from RigidBodies message
      pose_msg.pose = rb.pose;
      
      publishers_[rb_name]->publish(pose_msg);
    }
  }

  // Subscription to RigidBodies
  rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr subscription_;
  
  // Map of publishers (one for each rigid body)
  std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> publishers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RigidBodyPublisherNode>());
  rclcpp::shutdown();
  return 0;
}