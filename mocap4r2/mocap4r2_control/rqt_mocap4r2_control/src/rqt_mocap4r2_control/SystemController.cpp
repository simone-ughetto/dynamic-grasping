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

#include <sys/stat.h>
#include <sys/types.h>

#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include <QTreeWidgetItem>
#include <QCheckBox>
#include <QString>

#include "mocap4r2_control/ControllerNode.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "rqt_mocap4r2_control/SystemController.hpp"
#include "rqt_mocap4r2_control/utils.hpp"

namespace rqt_mocap4r2_control
{

SystemController::SystemController(
  rclcpp::Node::SharedPtr node, const std::string & system_name)
: QTreeWidgetItem(), node_(node), system_name_(system_name)
{
  setCheckState(0, Qt::Checked);
  setText(1, QString(system_name.c_str()));
}

void
SystemController::add_topic(const std::string & topic)
{
  if (topics_.find(topic) == topics_.end()) {
    topics_[topic] = {nullptr, nullptr};
    update_topics();
  }
}

const std::vector<std::string>
SystemController::get_topics()
{
  std::vector<std::string> ret;
  for (const auto & topic : topics_) {
    ret.push_back(topic.first);
  }
  return ret;
}

void
SystemController::set_active(bool active)
{
  setCheckState(0, active ? Qt::Checked : Qt::Unchecked);
}

void
SystemController::set_log_all(bool log)
{
  for (auto & topic : topics_) {
    topic.second.item->setCheckState(2, log ? Qt::Checked : Qt::Unchecked);
  }
}

void
SystemController::update_topics()
{
  for (auto & topic : topics_) {
    // Initialize new entries
    if (topic.second.item == nullptr) {
      topic.second.item = new QTreeWidgetItem();
      topic.second.item->setText(1, QString(topic.first.c_str()));
      topic.second.item->setCheckState(2, Qt::Unchecked);
      addChild(topic.second.item);
    }
  }
}

void
SystemController::update_elapsed_ts(double elapsed)
{
  setText(3, QString(std::to_string(elapsed).c_str()) + " secs");
}

void
SystemController::start_capture(const std::string & output_dir, CaptureMode mode)
{
  if (mode == CSV) {
    for (auto & topic : topics_) {
      if (topic.second.item->checkState(2) == Qt::Checked) {
        auto topics_info = node_->get_publishers_info_by_topic(topic.first);

        if (!topics_info.empty()) {
          topic.second.sub_ = create_csv_writer(
            topic.first, topics_info.front().topic_type(),
            output_dir);
        }
      }
    }
  } else {  // ROSBAG2
    RCLCPP_ERROR_STREAM(node_->get_logger(), "ROSBAG outout not implemented yet");
  }
}

rclcpp::SubscriptionBase::SharedPtr
SystemController::create_csv_writer(
  const std::string & topic, const std::string & msg_type, const std::string & output_dir)
{
  auto output_file = std::make_shared<std::ofstream>();
  mkdir((output_dir + "/" + get_name()).c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

  std::string output_file_name = topic + ".csv";
  std::replace(output_file_name.begin(), output_file_name.end(), '/', '_');
  output_file_name = output_dir + "/" + get_name() + "/" + output_file_name;

  output_file->open(output_file_name);

  if (msg_type == "sensor_msgs/msg/Imu") {
    return node_->create_subscription<sensor_msgs::msg::Imu>(
      topic, rclcpp::SensorDataQoS().keep_all(),
      [ = ](const sensor_msgs::msg::Imu::SharedPtr msg) {
        *output_file << msg << std::endl;
      });
  } else {  // ToDo(fmrico): Repeat previous block for each supported msg type
    RCLCPP_WARN_STREAM(node_->get_logger(), "Type [" << msg_type << "] not supported yet");
  }

  return nullptr;
}

void
SystemController::stop_mocap()
{
  for (auto & topic : topics_) {
    topic.second.sub_ = nullptr;
  }
}

}  // namespace rqt_mocap4r2_control
