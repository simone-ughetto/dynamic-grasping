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
// Author: Jose Miguel Guerrero Hernandez <josemiguel.guerrero@urjc.es>

#include <string>

#include "mocap4r2_marker_viz/mocap4r2_marker_viz_node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

MarkerVisualizer::MarkerVisualizer()
: Node("marker_visualizer")
{
  publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization_marker", 1000);

  declare_parameter<float>("default_marker_color_r", 0.0f);
  declare_parameter<float>("default_marker_color_g", 1.0f);
  declare_parameter<float>("default_marker_color_b", 0.0f);
  declare_parameter<float>("default_marker_color_a", 1.0f);
  declare_parameter<double>("marker_scale_x", 0.014f);
  declare_parameter<double>("marker_scale_y", 0.014f);
  declare_parameter<double>("marker_scale_z", 0.014f);
  declare_parameter<float>("marker_lifetime", 0.01f);
  declare_parameter<std::string>("namespace", "mocap4r2_markers");
  declare_parameter<std::string>("mocap4r2_system", "optitrack");

  get_parameter<float>("default_marker_color_r", default_marker_color_.r);
  get_parameter<float>("default_marker_color_g", default_marker_color_.g);
  get_parameter<float>("default_marker_color_b", default_marker_color_.b);
  get_parameter<float>("default_marker_color_a", default_marker_color_.a);
  get_parameter<double>("marker_scale_x", marker_scale_.x);
  get_parameter<double>("marker_scale_y", marker_scale_.y);
  get_parameter<double>("marker_scale_z", marker_scale_.z);
  get_parameter<float>("marker_lifetime", marker_lifetime_);
  get_parameter<std::string>("namespace", namespace_);
  get_parameter<std::string>("mocap4r2_system", mocap4r2_system_);

  markers_subscription_ = this->create_subscription<mocap4r2_msgs::msg::Markers>(
    "markers", 1000, std::bind(&MarkerVisualizer::marker_callback, this, _1));

  // Rigid bodies
  markers_subscription_rb_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", 1000, std::bind(&MarkerVisualizer::rb_callback, this, _1));

  publisher_rb_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization_marker_rb", 1000);
}


// This function change mocap axis to match with rviz axis
geometry_msgs::msg::Pose MarkerVisualizer::mocap2rviz(const geometry_msgs::msg::Pose mocap4r2_pose)
const
{
  geometry_msgs::msg::Pose rviz_pose;
  if (mocap4r2_system_ == "optitrack") {
    rviz_pose.position.x = -mocap4r2_pose.position.y;
    rviz_pose.position.y = mocap4r2_pose.position.x;
    rviz_pose.position.z = mocap4r2_pose.position.z;
    rviz_pose.orientation.x = mocap4r2_pose.orientation.x;
    rviz_pose.orientation.y = -mocap4r2_pose.orientation.y;
    rviz_pose.orientation.z = mocap4r2_pose.orientation.z;
    rviz_pose.orientation.w = mocap4r2_pose.orientation.w;
  } else if (mocap4r2_system_ == "vicon") {
    // TO-DO:
    rviz_pose = mocap4r2_pose;
  } else if (mocap4r2_system_ == "qualisys") {
    // TO-DO:
    rviz_pose = mocap4r2_pose;
  } else {
    rviz_pose = mocap4r2_pose;
  }
  return rviz_pose;
}


void
MarkerVisualizer::marker_callback(const mocap4r2_msgs::msg::Markers::SharedPtr msg) const
{
  if (publisher_->get_subscription_count() == 0) {
    return;
  }

  static int counter = 0;
  visualization_msgs::msg::MarkerArray visual_markers;
  for (const mocap4r2_msgs::msg::Marker & marker : msg->markers) {
    visual_markers.markers.push_back(marker2visual(counter++, marker.translation, msg->header));
  }
  publisher_->publish(visual_markers);
}

visualization_msgs::msg::Marker
MarkerVisualizer::marker2visual(
  int index, const geometry_msgs::msg::Point & translation,
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::Marker viz_marker;
  viz_marker.header = header;
  viz_marker.ns = namespace_;
  viz_marker.color = default_marker_color_;
  viz_marker.id = index;
  viz_marker.type = visualization_msgs::msg::Marker::SPHERE;
  viz_marker.action = visualization_msgs::msg::Marker::ADD;
  // Change mocap system axis to rviz axis
  geometry_msgs::msg::Pose marker_pose;
  marker_pose.position = translation;
  marker_pose.orientation.x = 0.0f;
  marker_pose.orientation.y = 0.0f;
  marker_pose.orientation.z = 0.0f;
  marker_pose.orientation.w = 1.0f;
  viz_marker.pose = mocap2rviz(marker_pose);
  viz_marker.scale = marker_scale_;
  viz_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
  return viz_marker;
}


void
MarkerVisualizer::rb_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) const
{
  if (publisher_rb_->get_subscription_count() == 0) {
    return;
  }

  static int counter_rb = 0;
  static int counter_markers_rb = 0;
  visualization_msgs::msg::MarkerArray visual_markers_rb;

  for (const mocap4r2_msgs::msg::RigidBody & rb : msg->rigidbodies) {
    visual_markers_rb.markers.push_back(rb2visual(counter_rb++, rb.pose, msg->header));

    for (const mocap4r2_msgs::msg::Marker & marker : rb.markers) {
      visual_markers_rb.markers.push_back(
        marker2visual(
          counter_markers_rb++,
          marker.translation, msg->header));
    }
  }

  publisher_rb_->publish(visual_markers_rb);
}


visualization_msgs::msg::Marker
MarkerVisualizer::rb2visual(
  int index, const geometry_msgs::msg::Pose & poserb,
  const std_msgs::msg::Header & header) const
{
  visualization_msgs::msg::Marker viz_marker;
  viz_marker.header = header;
  viz_marker.ns = namespace_;
  viz_marker.color = default_marker_color_;
  viz_marker.id = index;
  viz_marker.type = visualization_msgs::msg::Marker::ARROW;
  viz_marker.action = visualization_msgs::msg::Marker::ADD;

  // Change mocap system axis to rviz axis
  viz_marker.pose = mocap2rviz(poserb);

  geometry_msgs::msg::Vector3 marker_scale_;
  marker_scale_.x = 0.5f;
  marker_scale_.y = 0.014f;
  marker_scale_.z = 0.014f;
  viz_marker.scale = marker_scale_;
  viz_marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_);
  return viz_marker;
}
