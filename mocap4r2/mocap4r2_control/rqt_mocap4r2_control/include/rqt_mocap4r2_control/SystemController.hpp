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

#ifndef RQT_MOCAP4R2_CONTROL__SYSTEM_CONTROLLER__HPP_
#define RQT_MOCAP4R2_CONTROL__SYSTEM_CONTROLLER__HPP_

#include <memory>
#include <string>
#include <vector>

#include <QTreeWidgetItem>

namespace rqt_mocap4r2_control
{

struct TopicInfo
{
  QTreeWidgetItem * item;
  rclcpp::SubscriptionBase::SharedPtr sub_;
};

class SystemController : public QTreeWidgetItem
{
public:
  SystemController(rclcpp::Node::SharedPtr node_, const std::string & system_name);

  void add_topic(const std::string & topic);
  const std::vector<std::string> get_topics();

  const std::string & get_name() {return system_name_;}

  void set_active(bool active);
  bool is_active() {return checkState(0) == Qt::Checked;}

  void set_log_all(bool log);
  void update_elapsed_ts(double elapsed);

  enum CaptureMode {ROSBAG, CSV};

  void start_capture(const std::string & output_dir, CaptureMode mode);
  void stop_mocap();

private:
  rclcpp::Node::SharedPtr node_;
  std::string system_name_;
  std::map<std::string, TopicInfo> topics_;

  void update_topics();
  rclcpp::SubscriptionBase::SharedPtr create_csv_writer(
    const std::string & topic, const std::string & msg_type,
    const std::string & output_dir);
};

}  // namespace rqt_mocap4r2_control

#endif  // RQT_MOCAP4R2_CONTROL__mocap4r2_CONTRO__HPP_
