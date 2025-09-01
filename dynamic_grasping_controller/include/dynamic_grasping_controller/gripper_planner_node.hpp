#ifndef DYNAMIC_GRASPING_CONTROLLER__GRIPPER_PLANNER_NODE_HPP_
#define DYNAMIC_GRASPING_CONTROLLER__GRIPPER_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <string>
#include <memory>
#include <utility>  // for std::pair

namespace dynamic_grasping_controller
{

/**
 * @brief Node for controlling the gripper of a WidowX robot arm
 *
 * This node subscribes to gripper commands and translates them to
 * FollowJointTrajectory actions to control the gripper position.
 * It supports preset positions ('grasping', 'released', 'home') as well
 * as direct width specifications, and publishes the current gripper state.
 */
class GripperPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the GripperPlannerNode
   */
  explicit GripperPlannerNode();

private:
  /**
   * @brief Possible states of the gripper planner state machine
   */
  enum class State {
    IDLE,          ///< Ready to accept new commands
    EXECUTING,     ///< Currently executing a command
    SHUTTING_DOWN  ///< In the process of shutting down
  };

  // Action client type aliases
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  /**
   * @brief Update current finger position from incoming joint states
   * @param msg Joint state message containing finger position
   */
  void jointStateCallback(const control_msgs::msg::DynamicJointState::SharedPtr msg);

  /**
   * @brief Parse and execute gripper commands
   * @param msg Command message with command string
   */
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief Parse command text into a command type and target position
   * @param data Command string
   * @return Pair of command name and target position, or {empty, 0.0} if invalid
   */
  std::pair<std::string, double> parseCommand(const std::string &data);

  /**
   * @brief Execute gripper movement to target position
   * @param target_pos Target finger position in meters
   */
  void executeTrajectory(double target_pos);

  /**
   * @brief Goal response callback for action client
   * @param goal_handle The goal handle from the server
   */
  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr &goal_handle);

  /**
   * @brief Result callback for action client
   * @param result Action result including success/failure status
   */
  void resultCallback(
    const rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult &result);

  /**
   * @brief Safety timeout callback to prevent getting stuck in EXECUTING state
   */
  void safetyTimeoutCallback();

  /**
   * @brief Create a one-shot timer that executes a callback once
   * @param seconds Delay in seconds
   * @param callback Function to call after delay
   */
  void createOneshotTimer(double seconds, std::function<void()> callback);

  /**
   * @brief Called after execution is complete to log readiness
   */
  void onExecutionComplete();

  // Current state
  State state_;
  double current_finger_position_;
  double target_position_;
  std::string current_command_;

  // ROS interfaces
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_state_pub_;

  // Timer for safety timeout
  rclcpp::TimerBase::SharedPtr safety_timer_;

  // Parameters
  double preset_grasping_;
  double preset_released_;
  double preset_home_;
  double max_width_;
  double position_tolerance_;
};

} // namespace dynamic_grasping_controller

#endif // DYNAMIC_GRASPING_CONTROLLER__GRIPPER_PLANNER_NODE_HPP_