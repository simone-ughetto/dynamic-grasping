// Copyright 2021 Institute for Robotics and Intelligent Machines,
//                Georgia Institute of Technology
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
//
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Author: David Vargas Frutos <david.vargas@urjc.es>
// Author: Francisco Martín <fmrico@urjc.es>


#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

#include "mocap4r2_dummy_driver/mocap4r2_dummy_driver.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace mocap4r2_dummy_driver
{

using namespace std::chrono_literals;
using std::placeholders::_1;

DummyDriverNode::DummyDriverNode()
: ControlledLifecycleNode("mocap4r2_dummy_driver_node")
{
}

DummyDriverNode::~DummyDriverNode()
{
}

void
DummyDriverNode::control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}

void
DummyDriverNode::control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  (void)msg;
}


void
DummyDriverNode::publish_data()
{
  frame_number_++;

  // Markers
  if (mocap4r2_markers_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::Markers msg;
    msg.header.stamp = now();
    msg.header.frame_id = "mocap";
    msg.frame_number = frame_number_;

    mocap4r2_msgs::msg::Marker marker;
    marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
    marker.marker_index = 0;
    marker.translation.x = 0.0;
    marker.translation.y = 0.0;
    marker.translation.z = 0.0;
    msg.markers.push_back(marker);

    marker.marker_index = 1;
    marker.translation.x = 0.1;
    marker.translation.y = 0.1;
    marker.translation.z = 0.0;
    msg.markers.push_back(marker);

    marker.marker_index = 2;
    marker.translation.x = 0.1;
    marker.translation.y = -0.1;
    marker.translation.z = 0.0;
    msg.markers.push_back(marker);

    mocap4r2_markers_pub_->publish(msg);
  }

  if (mocap4r2_rigid_body_pub_->get_subscription_count() > 0) {
    mocap4r2_msgs::msg::RigidBodies msg_rb;
    msg_rb.header.stamp = now();
    msg_rb.header.frame_id = "mocap";
    msg_rb.frame_number = frame_number_;

    mocap4r2_msgs::msg::RigidBody rb;
    rb.rigid_body_name = "rigid_body_0";
    rb.pose.position.x = 0.0;
    rb.pose.position.y = 0.0;
    rb.pose.position.z = 0.0;
    rb.pose.orientation.x = 0.0;
    rb.pose.orientation.y = 0.0;
    rb.pose.orientation.z = 0.0;
    rb.pose.orientation.w = 1.0;

    mocap4r2_msgs::msg::Marker marker;
    marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
    marker.marker_index = 0;
    marker.translation.x = 0.0;
    marker.translation.y = 0.0;
    marker.translation.z = 0.0;
    rb.markers.push_back(marker);

    marker.marker_index = 1;
    marker.translation.x = 0.1;
    marker.translation.y = 0.1;
    marker.translation.z = 0.0;
    rb.markers.push_back(marker);

    marker.marker_index = 2;
    marker.translation.x = 0.1;
    marker.translation.y = -0.1;
    marker.translation.z = 0.0;
    rb.markers.push_back(marker);

    msg_rb.rigidbodies.push_back(rb);

    mocap4r2_rigid_body_pub_->publish(msg_rb);
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;


// The next Callbacks are used to manage behavior in the different states of the lifecycle node.
CallbackReturnT
DummyDriverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  mocap4r2_markers_pub_ = create_publisher<mocap4r2_msgs::msg::Markers>(
    "markers", rclcpp::QoS(1000));
  mocap4r2_rigid_body_pub_ = create_publisher<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", rclcpp::QoS(1000));

  // Connect with the mocap system

  RCLCPP_INFO(get_logger(), "Configured!\n");

  return ControlledLifecycleNode::on_configure(state);
}

CallbackReturnT
DummyDriverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  mocap4r2_markers_pub_->on_activate();
  mocap4r2_rigid_body_pub_->on_activate();

  timer_ = create_wall_timer(30ms, std::bind(&DummyDriverNode::publish_data, this));

  RCLCPP_INFO(get_logger(), "Activated!\n");

  return ControlledLifecycleNode::on_activate(state);
}

CallbackReturnT
DummyDriverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  mocap4r2_markers_pub_->on_deactivate();
  mocap4r2_rigid_body_pub_->on_deactivate();
  timer_ = nullptr;

  RCLCPP_INFO(get_logger(), "Deactivated!\n");

  return ControlledLifecycleNode::on_deactivate(state);
}

CallbackReturnT
DummyDriverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Cleaned up!\n");

  // Disconnect with the mocap system

  return ControlledLifecycleNode::on_cleanup(state);
}

CallbackReturnT
DummyDriverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutted down!\n");

  // Disconnect with the mocap system

  return ControlledLifecycleNode::on_shutdown(state);
}

CallbackReturnT
DummyDriverNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "State id [%d]", get_current_state().id());
  RCLCPP_INFO(get_logger(), "State label [%s]", get_current_state().label().c_str());

  // Disconnect with the mocap system

  return ControlledLifecycleNode::on_error(state);
}


}  // namespace mocap4r2_dummy_driver
