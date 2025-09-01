/**
 * @file gripper_planner_node.cpp
 * @brief C++ implementation of gripper planner for WidowX robot
 * 
 * This node provides functionality to control the WidowX gripper through
 * preset commands or specific width values. It sends trajectory goals to
 * the gripper controller and reports gripper state.
 */

#include "dynamic_grasping_controller/gripper_planner_node.hpp"
#include <chrono>
#include <memory>
#include <string>
#include <cstdlib>
#include <utility>
#include <functional>
#include <algorithm>  // for std::transform, std::find
#include <cctype>     // for std::tolower, std::isspace
#include <cmath>      // for std::abs

using namespace std::chrono_literals;

namespace dynamic_grasping_controller
{

GripperPlannerNode::GripperPlannerNode()
: Node("gripper_planner"),
  state_(State::IDLE),
  current_finger_position_(0.037),  // Default to released position
  target_position_(0.0),
  current_command_("")
{
  // --- Declare parameters ---
  this->declare_parameter("preset.grasping", 0.015);
  this->declare_parameter("preset.released", 0.037);
  this->declare_parameter("preset.home", 0.0195);
  this->declare_parameter("max_width", 0.053);
  this->declare_parameter("position_tolerance", 0.002);

  // --- Load parameters ---
  preset_grasping_ = this->get_parameter("preset.grasping").as_double();
  preset_released_ = this->get_parameter("preset.released").as_double();
  preset_home_ = this->get_parameter("preset.home").as_double();
  max_width_ = this->get_parameter("max_width").as_double();
  position_tolerance_ = this->get_parameter("position_tolerance").as_double();

  // --- Initialize current position from parameter ---
  current_finger_position_ = preset_released_;

  // --- Create action client ---
  action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/wx250s/gripper_controller/follow_joint_trajectory");

  // --- Create subscribers ---
  joint_state_sub_ = this->create_subscription<control_msgs::msg::DynamicJointState>(
    "/wx250s/dynamic_joint_states", 10,
    std::bind(&GripperPlannerNode::jointStateCallback, this, std::placeholders::_1));

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "gripper_command", 10,
    std::bind(&GripperPlannerNode::commandCallback, this, std::placeholders::_1));

  // --- Create publishers ---
  gripper_state_pub_ = this->create_publisher<std_msgs::msg::Bool>("gripper_closed", 10);

  RCLCPP_INFO(this->get_logger(), "GripperPlanner initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  preset.grasping: %.4f", preset_grasping_);
  RCLCPP_INFO(this->get_logger(), "  preset.released: %.4f", preset_released_);
  RCLCPP_INFO(this->get_logger(), "  preset.home: %.4f", preset_home_);
  RCLCPP_INFO(this->get_logger(), "  max_width: %.4f", max_width_);
  RCLCPP_INFO(this->get_logger(), "  position_tolerance: %.4f", position_tolerance_);
}

void GripperPlannerNode::jointStateCallback(
  const control_msgs::msg::DynamicJointState::SharedPtr msg)
{
  // Find left_finger joint
  auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), "left_finger");
  if (it == msg->joint_names.end()) {
    return;
  }

  // Get the index
  size_t idx = std::distance(msg->joint_names.begin(), it);
  if (idx >= msg->interface_values.size()) {
    return;
  }

  // Find position interface
  auto& interfaces = msg->interface_values[idx].interface_names;
  auto pos_it = std::find(interfaces.begin(), interfaces.end(), "position");
  if (pos_it == interfaces.end()) {
    return;
  }

  // Get position value
  size_t pos_idx = std::distance(interfaces.begin(), pos_it);
  if (pos_idx >= msg->interface_values[idx].values.size()) {
    return;
  }

  current_finger_position_ = msg->interface_values[idx].values[pos_idx];
}

void GripperPlannerNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // Ignore commands when not in IDLE state
  if (state_ != State::IDLE) {
    RCLCPP_INFO(this->get_logger(), "Busy, command ignored.");
    return;
  }

  // Parse command
  auto [cmd, width] = parseCommand(msg->data);
  if (cmd.empty()) {
    return;
  }

  // Store command type
  current_command_ = cmd;

  // Handle quit command
  if (cmd == "quit") {
    state_ = State::SHUTTING_DOWN;
    executeTrajectory(width);
    createOneshotTimer(2.0, []() { rclcpp::shutdown(); });
    return;
  }

  // Normal commands
  state_ = State::EXECUTING;
  executeTrajectory(width);
  createOneshotTimer(2.0, std::bind(&GripperPlannerNode::onExecutionComplete, this));

  // Safety timeout - force return to IDLE if action doesn't complete in 3 seconds
  safety_timer_ = this->create_wall_timer(
    3s, std::bind(&GripperPlannerNode::safetyTimeoutCallback, this));
}

std::pair<std::string, double> GripperPlannerNode::parseCommand(
  const std::string &data)
{
  std::string text = data;
  
  // Remove leading and trailing whitespace
  text.erase(0, text.find_first_not_of(" \t\n\r\f\v"));
  text.erase(text.find_last_not_of(" \t\n\r\f\v") + 1);

  // Convert to lowercase
  std::transform(text.begin(), text.end(), text.begin(), 
                 [](unsigned char c) { return std::tolower(c); });

  // Check preset commands
  if (text == "grasping") {
    return {"grasping", preset_grasping_};
  } else if (text == "released") {
    return {"released", preset_released_};
  } else if (text == "home") {
    return {"home", preset_home_};
  } else if (text == "quit") {
    return {"quit", preset_released_};
  }

  // Try parsing as numeric width
  try {
    double width = std::stod(text);
    
    // Validate width
    if (width > max_width_) {
      RCLCPP_ERROR(this->get_logger(), "Width too large (>%f m).", max_width_);
      return {"", 0.0};
    }
    
    // Convert width to finger position
    double pos = (width + 0.018) / 2.0;
    return {"custom", pos};
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), 
      "Invalid command; use 'grasping', 'released', 'home', 'quit', or a numeric width.");
    return {"", 0.0};
  }
}

void GripperPlannerNode::executeTrajectory(double target_pos)
{
  // Check action server availability
  if (!action_client_->wait_for_action_server(3s)) {
    RCLCPP_ERROR(this->get_logger(), "Action server unavailable after timeout!");
    state_ = State::IDLE;
    return;
  }

  // Store target position for later verification
  target_position_ = target_pos;
  RCLCPP_INFO(this->get_logger(), "Executing gripper movement to position: %.4f", target_pos);

  // Build trajectory
  auto traj = std::make_shared<trajectory_msgs::msg::JointTrajectory>();
  traj->joint_names = {"left_finger"};

  // Add points (start, mid, end)
  trajectory_msgs::msg::JointTrajectoryPoint start_point;
  start_point.positions = {current_finger_position_};
  start_point.time_from_start.sec = 0;
  start_point.time_from_start.nanosec = 2500000; // 0.025 seconds

  trajectory_msgs::msg::JointTrajectoryPoint mid_point;
  mid_point.positions = {(current_finger_position_ + target_pos) / 2.0};
  mid_point.time_from_start.sec = 0;
  mid_point.time_from_start.nanosec = 350000000; // 0.35 seconds

  trajectory_msgs::msg::JointTrajectoryPoint end_point;
  end_point.positions = {target_pos};
  end_point.time_from_start.sec = 0;
  end_point.time_from_start.nanosec = 700000000; // 0.7 seconds

  traj->points = {start_point, mid_point, end_point};

  // Create goal
  auto goal_msg = FollowJointTrajectory::Goal();
  goal_msg.trajectory = *traj;

  // Send goal
  auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&GripperPlannerNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&GripperPlannerNode::resultCallback, this, std::placeholders::_1);

  action_client_->async_send_goal(goal_msg, send_goal_options);
}

void GripperPlannerNode::goalResponseCallback(
  const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr &goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "Goal rejected.");
    state_ = State::IDLE;
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Gripper goal accepted, waiting for result");
}

void GripperPlannerNode::resultCallback(
  const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult &result)
{
  // Cancel safety timer if it's still active
  if (safety_timer_) {
    safety_timer_->cancel();
    safety_timer_.reset();
  }

  try {
    // Check if the action was successful
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_ERROR(this->get_logger(), "Action failed with code: %d", static_cast<int>(result.code));
    } else {
      // Simply determine if gripper is closed based on whether command is "released"
      bool is_closed = (current_command_ != "released");
      
      // Publish gripper state
      auto closed_msg = std_msgs::msg::Bool();
      closed_msg.data = is_closed;
      gripper_state_pub_->publish(closed_msg);

      RCLCPP_INFO(this->get_logger(), "Gripper action successful, gripper_closed=%s",
        is_closed ? "true" : "false");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Error handling gripper result: %s", e.what());
  }

  // Reset state unless shutting down
  if (state_ == State::EXECUTING) {
    state_ = State::IDLE;
  }
}

void GripperPlannerNode::safetyTimeoutCallback()
{
  if (state_ == State::EXECUTING) {
    RCLCPP_WARN(this->get_logger(), "Action taking too long - forcing return to IDLE state");
    state_ = State::IDLE;
  }
  safety_timer_.reset();
}

void GripperPlannerNode::createOneshotTimer(double seconds, std::function<void()> callback)
{
  auto timer = this->create_wall_timer(std::chrono::duration<double>(seconds), 
    [callback, this]() {
      callback();
      // Timer will be cancelled after firing once
    });
  
  // Store timer to keep it alive until it fires
  rclcpp::TimerBase::SharedPtr shared_timer = timer;
  this->create_wall_timer(std::chrono::duration<double>(seconds + 0.1), 
    [shared_timer]() {
      // Cancel timer after it has fired
      shared_timer->cancel();
    });
}

void GripperPlannerNode::onExecutionComplete()
{
  RCLCPP_INFO(this->get_logger(), "Ready for next command.");
}

} // namespace dynamic_grasping_controller

// Main function
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dynamic_grasping_controller::GripperPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}