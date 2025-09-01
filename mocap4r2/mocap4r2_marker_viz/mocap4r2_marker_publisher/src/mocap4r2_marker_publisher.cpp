// Copyright 2019 Intelligent Robotics Lab
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
// Author: David Vargas Frutos <david.vargas@urjc.es>

#include <chrono>
#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/marker.hpp"
#include "mocap4r2_msgs/msg/markers.hpp"

using namespace std::chrono_literals;

class MarkerPublisher : public rclcpp::Node
{
public:
  MarkerPublisher()
  : Node("mocap4r2_marker_publisher")
  {
    publisher_ = this->create_publisher<mocap4r2_msgs::msg::Markers>("markers", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&MarkerPublisher::timer_callback, this));
  }

  void timer_callback()
  {
    mocap4r2_msgs::msg::Markers markers;
    for (int i = 0; i < 10; i++) {
      mocap4r2_msgs::msg::Marker marker;
      marker.id_type = mocap4r2_msgs::msg::Marker::USE_INDEX;
      marker.marker_index = i;
      marker.translation.x = 0;
      marker.translation.y = 0;
      marker.translation.z = 0.1 * i;
      markers.markers.push_back(marker);
    }
    publisher_->publish(markers);
  }

private:
  rclcpp::Publisher<mocap4r2_msgs::msg::Markers>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
