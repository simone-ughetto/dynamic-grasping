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

#include <unistd.h>

#include "rqt_mocap4r2_control/mocap4r2_control.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>

#include <QDebug>
#include <QTime>
#include <QTimer>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>
#include <QCheckBox>

#include "mocap4r2_control_msgs/msg/mocap_info.hpp"

#include "rqt_mocap4r2_control/SystemController.hpp"

namespace rqt_mocap4r2_control
{

MocapControl::MocapControl()
: rqt_gui_cpp::Plugin()
  , widget_(0)
{
  setObjectName("MocapControl");

  current_output_dir_ = "/tmp";
}

void MocapControl::initPlugin(qt_gui_cpp::PluginContext & context)
{
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(
      widget_->windowTitle() + " (" + QString::number(
        context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  ui_.treeWidget->setColumnCount(3);
  ui_.treeWidget->setHeaderLabels({"Active", "Topic", "Record", "Elapsed"});

  controller_node_ = std::make_shared<mocap4r2_control::ControllerNode>(
    std::bind(&MocapControl::control_callback, this, std::placeholders::_1));
  controller_spin_timer_ = new QTimer(this);
  connect(controller_spin_timer_, SIGNAL(timeout()), this, SLOT(spin_loop()));
  controller_spin_timer_->start(10);

  // Add tf and tf_static
  auto system_item = new SystemController(controller_node_, "System");
  system_item->add_topic("tf");
  system_item->add_topic("tf_static");
  mocap4r2_env_[system_item->get_name()] = system_item;
  ui_.treeWidget->addTopLevelItem(system_item);

  ui_.treeWidget->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
  ui_.treeWidget->header()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
  ui_.treeWidget->header()->setSectionResizeMode(2, QHeaderView::ResizeToContents);

  mocap4r2_env_sub_ = node_->create_subscription<mocap4r2_control_msgs::msg::MocapInfo>(
    "mocap4r2_environment", rclcpp::QoS(1000).reliable().transient_local().keep_all(),
    [this](const mocap4r2_control_msgs::msg::MocapInfo::SharedPtr msg)
    {
      update_tree(msg);
    });

  connect(ui_.startButton, SIGNAL(clicked()), this, SLOT(start_capture()));
  connect(ui_.selectOutputDirpushButton, SIGNAL(clicked()), this, SLOT(select_output_dir()));
  connect(ui_.recordAllCheckBox, SIGNAL(clicked(bool)), this, SLOT(select_record_all(bool)));
  connect(ui_.activeAllCheckBox, SIGNAL(clicked(bool)), this, SLOT(select_active_all(bool)));
  connect(ui_.enableROS1checkBox, SIGNAL(stateChanged(int)), this, SLOT(enable_ros1(int)));
}

void MocapControl::shutdownPlugin()
{
}

void MocapControl::spin_loop()
{
  rclcpp::spin_some(controller_node_);
}

void MocapControl::saveSettings(
  qt_gui_cpp::Settings & plugin_settings,
  qt_gui_cpp::Settings & instance_settings) const
{
  (void)plugin_settings;
  (void)instance_settings;
}

void MocapControl::restoreSettings(
  const qt_gui_cpp::Settings & plugin_settings,
  const qt_gui_cpp::Settings & instance_settings)
{
  (void)plugin_settings;
  (void)instance_settings;
}

void
MocapControl::update_tree(const mocap4r2_control_msgs::msg::MocapInfo::SharedPtr msg)
{
  SystemController * current_system;
  if (mocap4r2_env_.find(msg->mocap4r2_source) == mocap4r2_env_.end()) {
    current_system = new SystemController(node_, msg->mocap4r2_source);
    mocap4r2_env_[current_system->get_name()] = current_system;
    ui_.treeWidget->addTopLevelItem(current_system);

  } else {
    current_system = mocap4r2_env_[msg->mocap4r2_source];
  }

  auto current_topics = current_system->get_topics();
  for (const auto & msg_topic : msg->topics) {
    if (std::find(current_topics.begin(), current_topics.end(), msg_topic) ==
      current_topics.end())
    {
      current_system->add_topic(msg_topic);
    }
  }
}

void
MocapControl::select_output_dir()
{
  auto filename = QFileDialog::getExistingDirectory(
    widget_,
    tr("Select Output Dir"), tr(current_output_dir_.c_str()));

  if (!filename.isEmpty()) {
    current_output_dir_ = filename.toUtf8().constData();
    ui_.selectOutputDirpushButton->setText(filename);
  }
}

void
MocapControl::start_capture()
{
  if (!capturing_) {
    std::vector<std::string> mocap4r2_systems {""};
    for (auto & system : mocap4r2_env_) {
      if (system.second->is_active()) {
        mocap4r2_systems.push_back(system.second->get_name());
      }
    }

    controller_node_->start_system(
      ui_.sessionTextEdit->toPlainText().toUtf8().constData(), mocap4r2_systems);

    SystemController::CaptureMode mode;
    if (ui_.csvModeRadioButton->isChecked()) {
      mode = SystemController::CSV;
    } else {
      mode = SystemController::ROSBAG;
    }

    for (auto & system : mocap4r2_env_) {
      system.second->start_capture(current_output_dir_, mode);
    }
    capturing_ = true;

    ui_.startButton->setText("Stop");
    QPalette pal = ui_.startButton->palette();
    pal.setColor(QPalette::Button, QColor(Qt::red));
    ui_.startButton->setAutoFillBackground(true);
    ui_.startButton->setPalette(pal);
    ui_.startButton->update();
  } else {
    controller_node_->stop_system();
    for (auto & system : mocap4r2_env_) {
      system.second->stop_mocap();
    }
    capturing_ = false;

    ui_.startButton->setText("Start");
    QPalette pal = ui_.startButton->palette();
    pal.setColor(QPalette::Button, QColor(Qt::lightGray));
    ui_.startButton->setAutoFillBackground(true);
    ui_.startButton->setPalette(pal);
    ui_.startButton->update();
  }
}

void
MocapControl::control_callback(const mocap4r2_control_msgs::msg::Control::SharedPtr msg)
{
  if (msg->control_type == mocap4r2_control_msgs::msg::Control::ACK_START ||
    msg->control_type == mocap4r2_control_msgs::msg::Control::ACK_STOP)
  {
    double elapsed = (node_->now() - msg->stamp).seconds();

    if (mocap4r2_env_.find(msg->mocap4r2_source) != mocap4r2_env_.end()) {
      mocap4r2_env_[msg->mocap4r2_source]->update_elapsed_ts(elapsed);
    }
  }
}

void
MocapControl::select_record_all(bool checked)
{
  for (auto & system : mocap4r2_env_) {
    system.second->set_log_all(checked);
  }
}

void
MocapControl::select_active_all(bool checked)
{
  for (auto & system : mocap4r2_env_) {
    system.second->set_active(checked);
  }
}

void MocapControl::enable_ros1(int state)
{
  if (state == 2) {  // QT::Checked
    start_roscore_bridges();
  } else {
    if (pid_roscore_ != 0) {
      kill(pid_roscore_, SIGTERM);
    }
    if (pid_bridge1_ != 0) {
      kill(pid_bridge1_, SIGTERM);
    }
    if (pid_bridge2_ != 0) {
      kill(pid_bridge2_, SIGTERM);
    }
  }
}

void
MocapControl::start_roscore_bridges()
{
  pid_roscore_ = fork();

  if (pid_roscore_ == 0) {
    execlp("/usr/bin/xterm", "xterm", "-e", "source /opt/ros/noetic/setup.bash && roscore", NULL);
    RCLCPP_ERROR(node_->get_logger(), "roscore finished");
    exit(0);
  }

  sleep(1);   // avoid message for waiting roscore

  pid_bridge1_ = fork();

  if (pid_bridge1_ == 0) {
    execlp(
      "/opt/ros/foxy/bin/ros2", "ros2", "run", "ros1_bridge", "dynamic_bridge",
      "--bridge-all-1to2-topics", NULL);
    RCLCPP_ERROR(node_->get_logger(), "error executing bridge");
    sleep(1);
    exit(0);
  }

  pid_bridge2_ = fork();

  if (pid_bridge2_ == 0) {
    execlp("/opt/ros/foxy/bin/ros2", "ros2", "run", "ros1_bridge", "mocap4ros2_info_1_to_2", NULL);
    RCLCPP_ERROR(node_->get_logger(), "error executing bridge");
    sleep(1);
    exit(0);
  }
}

}  // namespace rqt_mocap4r2_control

PLUGINLIB_EXPORT_CLASS(rqt_mocap4r2_control::MocapControl, rqt_gui_cpp::Plugin)
