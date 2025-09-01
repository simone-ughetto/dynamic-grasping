// Copyright 2021 Intelligent Robotics Lab
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

#ifndef RQT_MOCAP4R2_CONTROL__UTILS__HPP_
#define RQT_MOCAP4R2_CONTROL__UTILS__HPP_

#include <memory>
#include <string>
#include <iomanip>
#include "sensor_msgs/msg/imu.hpp"

namespace rqt_mocap4r2_control
{

std::ostream & operator<<(std::ostream & os, const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // ToDo: Complete tranformation
  os << std::setprecision(10) << std::fixed <<
    rclcpp::Time(msg->header.stamp).seconds() << "," <<
    msg->linear_acceleration.x << "," <<
    msg->linear_acceleration.y << "," <<
    msg->linear_acceleration.z;
  return os;
}

}  // namespace rqt_mocap4r2_control

#endif  // RQT_MOCAP4R2_CONTROL__UTILS__HPP_
