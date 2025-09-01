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

#ifndef MOCAP4R2_DUMMY_DRIVER__MOCAP4R2_DUMMY_DRIVER_HPP_
#define MOCAP4R2_DUMMY_DRIVER__MOCAP4R2_DUMMY_DRIVER_HPP_

#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"
#include "mocap4r2_msgs/msg/rigid_body.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include "mocap4r2_control/ControlledLifecycleNode.hpp"


namespace mocap4r2_dummy_driver
{

class DummyDriverNode : public mocap4r2_control::ControlledLifecycleNode
{
public:
  DummyDriverNode();
  ~DummyDriverNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  void publish_data();

protected:
  void control_start(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;
  void control_stop(const mocap4r2_control_msgs::msg::Control::SharedPtr msg) override;

  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::Markers>::SharedPtr
    mocap4r2_markers_pub_;
  rclcpp_lifecycle::LifecyclePublisher<mocap4r2_msgs::msg::RigidBodies>::SharedPtr
    mocap4r2_rigid_body_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  uint32_t frame_number_{0};
};

}  // namespace mocap4r2_dummy_driver

#endif  // MOCAP4R2_DUMMY_DRIVER__MOCAP4R2_DUMMY_DRIVER_HPP_
