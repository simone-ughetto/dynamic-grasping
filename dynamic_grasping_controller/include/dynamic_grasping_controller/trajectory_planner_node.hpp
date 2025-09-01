/**
 * @file trajectory_planner_node.hpp
 * @brief ROS2 node for trajectory planning to grasp moving objects
 */

#ifndef DYNAMIC_GRASPING_CONTROLLER__TRAJECTORY_PLANNER_NODE_HPP_
#define DYNAMIC_GRASPING_CONTROLLER__TRAJECTORY_PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int8.hpp"  // Add Int8 import
#include "wx250s_bringup/msg/object_state.hpp"
#include "dynamic_grasping_controller/trajectory_utils.hpp"
#include "dynamic_grasping_controller/widowx_kinematics_library.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ruckig/ruckig.hpp>
#include <vector>
#include <string>

namespace dynamic_grasping_controller
{

/**
 * @brief Node for planning trajectories to grasp moving objects
 * 
 * This node generates trajectories for the WidowX 250 robot to grasp moving objects.
 * It uses a multi-phase approach:
 * 1. Approach phase with Z offset
 * 2. Direct approach to object
 * 3. Optional constant velocity matching (only for moving objects)
 * 4. Optional deceleration phase (only for moving objects)
 */
class TrajectoryPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * 
   * Initializes the node, parameters, publishers and subscribers
   */
  TrajectoryPlannerNode();

private:
  /**
   * @brief Object state callback
   * @param msg Object state message (position and velocity)
   */
  void objectStateCallback(const wx250s_bringup::msg::ObjectState::SharedPtr msg);
  
  /**
   * @brief Current end effector pose callback
   * @param msg Current pose message
   */
  void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  
  /**
   * @brief Current end effector velocity callback
   * @param msg Current velocity message
   */
  void currentVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  
  /**
   * @brief Start signal callback for dynamic grasping
   * @param msg Int8 message to control trajectory planning
   *        0: Disable trajectory planning
   *        1: Start normal trajectory planning
   *        2: Execute emergency stop trajectory
   */
  void startDynamicGraspingCallback(const std_msgs::msg::Int8::SharedPtr msg);
  
  /**
   * @brief Timer callback for trajectory planning
   */
  void timerCallback();

  /**
   * @brief Main trajectory planning function
   * @return True if planning succeeded, false otherwise
   */
  bool mainTrajectory();
  
  /**
   * @brief Plans and executes an emergency stop trajectory
   * @return True if emergency stop was successful, false otherwise
   */
  bool stopTrajectory();

  /**
   * @brief Plans a trajectory to go back to the deafault position
   * @return True if planning succeeded, false otherwise
   */
  bool goToRestTrajectory();

  // ROS publishers and subscribers
  rclcpp::Subscription<wx250s_bringup::msg::ObjectState>::SharedPtr object_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_ee_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr current_ee_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr start_dynamic_grasping_sub_;  // Change to Int8
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr direct_traj_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  std::vector<std::string> joint_names_;  ///< Joint names in the correct order
  geometry_msgs::msg::PoseStamped ee_pose_;  ///< Current end-effector pose
  Eigen::Vector3d ee_velocity_;              ///< Current end-effector linear velocity
  Eigen::Vector3d object_pos_;               ///< Object position
  Eigen::Vector3d object_vel_;               ///< Object velocity
  Eigen::Vector3d ee_euler_angles_;          ///< Current end-effector orientation as Euler angles
  Eigen::Vector3d ee_euler_rates_;           ///< Current end-effector angular velocities

  // Configuration variables
  double dt_;                        ///< Time step between trajectory points
  double max_velocity_;              ///< Maximum linear velocity
  double max_acceleration_;          ///< Maximum linear acceleration
  double max_jerk_;                  ///< Maximum linear jerk
  double max_angular_velocity_;      ///< Maximum angular velocity
  double max_angular_acceleration_;  ///< Maximum angular acceleration
  double max_angular_jerk_;          ///< Maximum angular jerk
  double approach_offset_;         ///< Z-offset for approach phase
  double basic_offset_;              ///< Small offset to avoid collision
  double t2_;                        ///< Second trajectory phase duration
  double t3_;                        ///< Third trajectory phase duration
  bool enable_trajectory_logging_;   ///< Whether to log trajectory data to CSV
  std::string trajectory_log_path_;  ///< Path for trajectory log CSV file
  bool log_cartesian_trajectory_;    ///< Whether to log trajectory to console
  double stationary_velocity_threshold_; ///< Threshold to consider object stationary
  double approach_velocity_; ///< Approach velocity
  bool include_velocity_;  ///< Whether to include velocity in trajectory points
  bool predict_ahead_;     ///< Whether to predict slightly ahead (0.1s) of the target position
  double prediction_offset_; ///< Time offset for ahead prediction (default 0.1s)

  // Flags
  bool object_detected_;             ///< Whether an object has been detected
  bool trajectory_executed_;         ///< Whether a trajectory has been executed

  // Grasp prediction
  bool show_prediction_marker_;  // Add the new member variable
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pred_pub_;
  std::string hardware_type_;
  std::string grasp_marker_xacro_;
  bool has_spawned_visual_marker_;
};

}  // namespace dynamic_grasping_controller

#endif  // DYNAMIC_GRASPING_CONTROLLER__TRAJECTORY_PLANNER_NODE_HPP_