// Copyright 2020 Intelligent Robotics Lab
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

#include <string>
#include <memory>
#include <set>
#include <vector>

#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "mocap4r2_control_msgs/msg/control.hpp"
#include "mocap4r2_control_msgs/msg/mocap_info.hpp"
#include "mocap4r2_control/ControlledLifecycleNode.hpp"
#include "mocap4r2_control/ControllerNode.hpp"
#include "mocap4r2_control/AuxiliarNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "gtest/gtest.h"


TEST(Controltests, test_sync)
{
  auto node_1 = std::make_shared<mocap4r2_control::AuxiliarNode>("example_mocap4r2_1");
  auto control_node = std::make_shared<mocap4r2_control::ControllerNode>();

  std::vector<mocap4r2_control_msgs::msg::MocapInfo> env_infos;
  auto env_sub = control_node->create_subscription<mocap4r2_control_msgs::msg::MocapInfo>(
    "mocap4r2_environment",
    rclcpp::QoS(1000).reliable().transient_local().keep_all(),
    [&env_infos](const mocap4r2_control_msgs::msg::MocapInfo::SharedPtr msg) {
      env_infos.push_back(*msg);
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node_1->get_node_base_interface());
  executor.add_node(control_node);

  ASSERT_TRUE(env_infos.empty());

  auto param = rclcpp::Parameter(
    "topics", std::vector<std::string> {
    "system1",
    "system1/sensor1",
    "system1/sensor2"});

  node_1->set_parameter(param);


  node_1->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  auto start = control_node->now();
  while ((control_node->now() - start).seconds() < 0.2) {
    executor.spin_some();
  }

  ASSERT_FALSE(env_infos.empty());
  ASSERT_EQ(env_infos.size(), 1u);
  ASSERT_EQ(env_infos[0].mocap4r2_source, "example_mocap4r2_1");
  ASSERT_EQ(env_infos[0].topics.size(), 3u);
  ASSERT_EQ(env_infos[0].topics[0], "system1");
  ASSERT_EQ(env_infos[0].topics[1], "system1/sensor1");
  ASSERT_EQ(env_infos[0].topics[2], "system1/sensor2");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  rclcpp::Rate(1).sleep();
  return result;
}
