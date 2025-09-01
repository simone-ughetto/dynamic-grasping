#ifndef DYNAMIC_GRASPING_MANAGER__DYNAMIC_GRASPING_MANAGER_HPP_
#define DYNAMIC_GRASPING_MANAGER__DYNAMIC_GRASPING_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <wx250s_bringup/msg/object_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <Eigen/Dense> // Include Eigen
#include <dynamic_grasping_controller/trajectory_utils.hpp> 
#include <dynamic_grasping_controller/widowx_kinematics_library.hpp>
#include <deque>
#include <memory>
#include <vector>
#include <optional> // Include optional

class DynamicGraspingManager : public rclcpp::Node
{
public:
  DynamicGraspingManager();

private:
  // -- parameters
  double monitoring_rate_;
  double outer_radius_;
  double inner_radius_;
  double stability_time_;
  double grasp_distance_;
  int trajectory_steps_;
  bool execute_rest_pose_;
  int stabilization_samples_;
  double velocity_diff_threshold_;
  double basic_offset_;
  double max_velocity_;      // Added parameter
  double max_acceleration_;  // Added parameter
  std::string hardware_type_;
  std::string object_name_;
  std::string gripper_close_value_;

  // -- state
  std::optional<Eigen::Vector3d> object_position_;
  std::optional<Eigen::Vector3d> object_velocity_;
  std::optional<Eigen::Vector3d> ee_position_;

  std::deque<Eigen::Vector3d> vel_buffer_;
  Eigen::Vector3d last_avg_velocity_;
  bool has_last_avg_velocity_ = false;

  bool can_retrigger_{false};
  bool waiting_for_stabilization_{false};
  int stable_velocity_count_{0};
  bool velocity_change_signaled_{false};

  bool in_graspable_zone_{false};
  bool grasping_in_progress_{false};
  bool gripper_closed_{false};
  bool rest_pose_{false};
  bool attachment_requested_{false};
  bool started_{false};  // Tracks whether system is started
  bool gripper_close_sent_{false}; // Flag to track if close command has been sent during current sequence

  // Subscriptions
  rclcpp::Subscription<wx250s_bringup::msg::ObjectState>::SharedPtr object_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gripper_closed_sub_;
  
  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr start_grasping_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gripper_command_pub_;
  
  // Service clients
  rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_link_client_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr delayed_command_timer_;

  // -- callbacks
  void object_state_callback(const wx250s_bringup::msg::ObjectState::SharedPtr msg);
  void ee_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void start_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void trajectory_completed_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void gripper_closed_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void monitor_callback();

  // -- internals
  void detect_direction_change();
  void request_link_attachment();
  void start_grasping_sequence();
  void reset_state();
};

#endif  // DYNAMIC_GRASPING_MANAGER__DYNAMIC_GRASPING_MANAGER_HPP_
