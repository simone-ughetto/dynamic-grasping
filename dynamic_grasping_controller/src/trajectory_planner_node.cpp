/**
 * @file trajectory_planner_node.cpp
 * @brief Implementation of the trajectory planner node for grasping a moving object
 *        This version bypasses the FollowJointTrajectory action client and publishes the
 *        computed JointTrajectory directly on the "trajectory_command" topic.
 */

#pragma region Includes

#include "dynamic_grasping_controller/trajectory_planner_node.hpp"
#include "dynamic_grasping_controller/trajectory_utils.hpp"
#include "dynamic_grasping_controller/widowx_kinematics_library.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "std_msgs/msg/bool.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <ruckig/ruckig.hpp>
#include <chrono>
#include <memory>
#include <vector>
#include "sensor_msgs/msg/joint_state.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <thread>
#include <mutex>

#pragma endregion Includes

namespace dynamic_grasping_controller
{

#pragma region Constructor
//=============================================================================
// Constructor
//=============================================================================
TrajectoryPlannerNode::TrajectoryPlannerNode()
: Node("trajectory_planner_node"),
  ee_velocity_(Eigen::Vector3d::Zero()),
  object_pos_(Eigen::Vector3d::Zero()),
  object_vel_(Eigen::Vector3d::Zero()),
  ee_euler_angles_(Eigen::Vector3d::Zero()),
  ee_euler_rates_(Eigen::Vector3d::Zero()),
  object_detected_(false),
  trajectory_executed_(false),
  has_spawned_visual_marker_(false)
{
  // Declare and get ROS parameters from the config file
  this->declare_parameter("dt", 0.05);
  this->declare_parameter("max_velocity", 0.3);
  this->declare_parameter("max_acceleration", 0.40);
  this->declare_parameter("max_jerk", 1e3);
  this->declare_parameter("max_angular_velocity", 2.0);
  this->declare_parameter("max_angular_acceleration", 1.5);
  this->declare_parameter("max_angular_jerk", 1e3);
  this->declare_parameter("enable_trajectory_logging", false);
  this->declare_parameter("trajectory_log_path", std::string("/home/sughetto/my_robot_ws/trajectory_data.csv"));
  this->declare_parameter("stationary_velocity_threshold", 5e-3);
  this->declare_parameter("approach_z_offset", 0.1);
  this->declare_parameter("t2_duration", 1.0);
  this->declare_parameter("t3_duration", 0.8);
  this->declare_parameter("include_velocity", false);
  this->declare_parameter("predict_ahead", false);
  this->declare_parameter("prediction_offset", 0.0);
  this->declare_parameter("hardware_type", "gz_classic");
  this->declare_parameter("basic_offset", 0.03);
  this->declare_parameter("approach_velocity", 0.05);
  this->declare_parameter("show_prediction_marker", true);  // Add the new parameter
  this->declare_parameter("grasp_marker_xacro", ament_index_cpp::get_package_share_directory("wx250s_bringup") + "/urdf/visual_cylinder.urdf.xacro");

  // Get parameters
  dt_ = this->get_parameter("dt").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();
  max_jerk_ = this->get_parameter("max_jerk").as_double();
  max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
  max_angular_acceleration_ = this->get_parameter("max_angular_acceleration").as_double();
  max_angular_jerk_ = this->get_parameter("max_angular_jerk").as_double();
  enable_trajectory_logging_ = this->get_parameter("enable_trajectory_logging").as_bool();
  trajectory_log_path_ = this->get_parameter("trajectory_log_path").as_string();
  stationary_velocity_threshold_ = this->get_parameter("stationary_velocity_threshold").as_double();
  include_velocity_ = this->get_parameter("include_velocity").as_bool();
  predict_ahead_ = this->get_parameter("predict_ahead").as_bool();
  prediction_offset_ = this->get_parameter("prediction_offset").as_double();
  approach_offset_ = this->get_parameter("approach_z_offset").as_double();
  basic_offset_ = this->get_parameter("basic_offset").as_double();
  approach_velocity_ = this->get_parameter("approach_velocity").as_double();
  t2_ = this->get_parameter("t2_duration").as_double();
  t3_ = this->get_parameter("t3_duration").as_double();
  hardware_type_ = this->get_parameter("hardware_type").as_string();
  show_prediction_marker_ = this->get_parameter("show_prediction_marker").as_bool();  // Get the new parameter
  grasp_marker_xacro_ = this->get_parameter("grasp_marker_xacro").as_string();

  // Create publisher for grasp prediction
  grasp_pred_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/grasp_prediction", 10);
  
  // Setup subscribers
  object_state_sub_ = this->create_subscription<wx250s_bringup::msg::ObjectState>(
    "/object_state", 10, std::bind(&TrajectoryPlannerNode::objectStateCallback, this, std::placeholders::_1));
    
  current_ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wx250s/end_effector_pose", 10, std::bind(&TrajectoryPlannerNode::currentPoseCallback, this, std::placeholders::_1));
    
  current_ee_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/wx250s/end_effector_velocities_euler", 10, std::bind(&TrajectoryPlannerNode::currentVelocityCallback, this, std::placeholders::_1));
  
  // Change Bool to Int8 for the start_dynamic_grasping subscription
  start_dynamic_grasping_sub_ = this->create_subscription<std_msgs::msg::Int8>(
    "/start_dynamic_grasping", 10, std::bind(&TrajectoryPlannerNode::startDynamicGraspingCallback, this, std::placeholders::_1));
  
  // Create a publisher for the JointTrajectory - publish directly to the controller
  direct_traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/wx250s/arm_controller/joint_trajectory", 10);
  
  // Setup timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(dt_)), 
    std::bind(&TrajectoryPlannerNode::timerCallback, this));
  
  // Initialize joint names from parameter server
  try {
    this->declare_parameter("robot.joint_names", std::vector<std::string>());
    joint_names_ = this->get_parameter("robot.joint_names").as_string_array();
    
    if (joint_names_.empty()) {
      // Fallback to hardcoded values if parameter is empty
      RCLCPP_WARN(this->get_logger(), "Joint names parameter is empty, using default names");
      joint_names_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Error getting joint names: %s. Using default names.", e.what());
    joint_names_ = {"waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};
  }
  
  RCLCPP_INFO(this->get_logger(), "Trajectory Planner Node initialized with parameters:");
  RCLCPP_INFO(this->get_logger(), "  dt: %.3f s", dt_);
  RCLCPP_INFO(this->get_logger(), "  max_velocity: %.3f m/s", max_velocity_);
  RCLCPP_INFO(this->get_logger(), "  max_acceleration: %.3f m/s²", max_acceleration_);
  RCLCPP_INFO(this->get_logger(), "  stationary_velocity_threshold: %.3e m/s", stationary_velocity_threshold_);
  RCLCPP_INFO(this->get_logger(), "  enable_trajectory_logging: %s", enable_trajectory_logging_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  include_velocity: %s", include_velocity_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  predict_ahead: %s", predict_ahead_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "  prediction_offset: %.3f s", prediction_offset_);
  RCLCPP_INFO(this->get_logger(), "  approach_z_offset: %.3f m", approach_offset_);
  RCLCPP_INFO(this->get_logger(), "  basic_offset: %.3f m", basic_offset_);
}
#pragma endregion

#pragma region Callbacks
//=============================================================================
// Callback Methods
//=============================================================================
void TrajectoryPlannerNode::objectStateCallback(const wx250s_bringup::msg::ObjectState::SharedPtr msg)
{
  object_pos_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
  object_vel_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
}

void TrajectoryPlannerNode::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  ee_pose_ = *msg;
  
  // Update current Euler angles from quaternion
  Eigen::Quaterniond current_quat(
    msg->pose.orientation.w,
    msg->pose.orientation.x,
    msg->pose.orientation.y,
    msg->pose.orientation.z
  );
  ee_euler_angles_ = quaternionToEuler(current_quat);
}

void TrajectoryPlannerNode::currentVelocityCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  ee_velocity_ = Eigen::Vector3d(
    msg->twist.linear.x,
    msg->twist.linear.y,
    msg->twist.linear.z);
  // Assuming Euler angle rates are provided in the angular part of the twist message
  ee_euler_rates_ = Eigen::Vector3d(
    msg->twist.angular.x,
    msg->twist.angular.y,
    msg->twist.angular.z);
}

void TrajectoryPlannerNode::startDynamicGraspingCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  switch (msg->data) {
    case 0:
      // Disable trajectory planning, similar to false in the old Bool version
      object_detected_ = false;
      break;
      
    case 1:
      // Enable normal trajectory planning, similar to true in the old Bool version
      if (!object_detected_) {
        object_detected_ = true;
        trajectory_executed_ = false;
        RCLCPP_INFO(this->get_logger(), "Normal trajectory planning requested");
      }
      break;
      
    case 2:
      // Execute stop trajectory
      RCLCPP_INFO(this->get_logger(), "stop trajectory requested");
      stopTrajectory();
      break;

    case 3:
      // Go to rest pose
      RCLCPP_INFO(this->get_logger(), "Going to rest pose");
      goToRestTrajectory();
      break;

    default:
      RCLCPP_WARN(this->get_logger(), "Unknown start_dynamic_grasping value: %d", msg->data);
      break;

  }
}

void TrajectoryPlannerNode::timerCallback()
{
  if (object_detected_ && !trajectory_executed_) {
    if (ee_pose_.header.stamp.sec == 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for complete pose and object data...");
      return;
    }
    
    bool success = mainTrajectory();
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Trajectory planned successfully!");
      trajectory_executed_ = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan trajectory!");
    }
    
    object_detected_ = false;
  }
}

#pragma endregion

#pragma region Trajectory planning methods
//=============================================================================
// Trajectory Planning Methods
//=============================================================================
#pragma region Main trajectory
bool TrajectoryPlannerNode::mainTrajectory()
{
  // Start timing the overall trajectory planning
  auto overall_start_time = std::chrono::high_resolution_clock::now();

  // Check if the object is stationary (velocity magnitude below threshold)
  const double object_velocity_magnitude = object_vel_.norm();
  const bool is_object_stationary = (object_velocity_magnitude < stationary_velocity_threshold_);
   
  Eigen::Vector3d current_position(
    ee_pose_.pose.position.x,
    ee_pose_.pose.position.y,            
    ee_pose_.pose.position.z
  );
  
  // Use adaptive t1 calculation
  double t1 = computeApproachDuration(object_pos_, object_vel_, current_position, approach_offset_, max_velocity_, dt_);
  RCLCPP_INFO(this->get_logger(), "Initial adaptive t1: %.2f s", t1);

  // Variables for trajectory planning
  ruckig::Ruckig<6> otg;
  ruckig::InputParameter<6> input;
  ruckig::Trajectory<6> trajectory1;
  std::array<double, 6> end_position, end_velocity, end_acceleration;
  bool recompute_trajectory = false;
  int max_iterations = 5; // Limit the number of recomputation attempts
  int iteration = 0;
  
  // Declare these variables outside of the loop so they're in scope for the entire function
  Eigen::Vector3d target_pos_1;
  Eigen::Quaterniond target_ori;
  Eigen::Vector3d target_ori_euler;

  // Start timer for first trajectory segment
  auto start_time = std::chrono::high_resolution_clock::now();

  //=============================================================================
  // PHASE 1: APPROACH TRAJECTORY (with Z offset) - with adaptive timing

  do {
    recompute_trajectory = false;
    iteration++;
    
    // Update target position for phase 1 using current t1 value (no redeclaration)
    // Apply prediction offset if predict_ahead is enabled
    double prediction_time_1 = predict_ahead_ ? t1 + prediction_offset_ : t1;
    target_pos_1 = predictObjectPosition(object_pos_, object_vel_, prediction_time_1);
    target_ori = computeDesiredOrientation(object_pos_, object_vel_);
    target_ori_euler = quaternionToEuler(target_ori);

    // // Create current pose for adaptive velocity calculation
    // Eigen::VectorXd ee_start_pose_1(6);
    // ee_start_pose_1 << 
    //   current_position.x(),
    //   current_position.y(),
    //   current_position.z(),
    //   ee_euler_angles_.x(),
    //   ee_euler_angles_.y(),
    //   ee_euler_angles_.z();

    // // Create target pose for phase 1
    // Eigen::VectorXd target_ee_pose_1(6);
    // target_ee_pose_1 << 
    //   target_pos_1.x(),
    //   target_pos_1.y(),
    //   target_pos_1.z() + approach_z_offset_,
    //   target_ori_euler.x(),
    //   target_ori_euler.y(),
    //   target_ori_euler.z();
    
    // // Compute adaptive max velocity for first phase
    // double adaptive_max_vel_1 = computeAdaptiveMaxVelocity(
    //   ee_start_pose_1,
    //   target_ee_pose_1,
    //   max_velocity_,
    //   this->get_logger()
    // );
  
    RCLCPP_INFO(this->get_logger(), "Planning first trajectory with t1=%.2f (iteration %d)", t1, iteration);
    
    input.current_position = {
      current_position.x(), current_position.y(), current_position.z(),
      ee_euler_angles_.x(), ee_euler_angles_.y(), ee_euler_angles_.z()
    };
    
    input.current_velocity = {
      ee_velocity_.x(), ee_velocity_.y(), ee_velocity_.z(),
      ee_euler_rates_.x(), ee_euler_rates_.y(), ee_euler_rates_.z()
    };
    
    input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    input.target_position = {
      target_pos_1.x(), target_pos_1.y(), target_pos_1.z() + approach_offset_,
      target_ori_euler.x(), target_ori_euler.y(), target_ori_euler.z()
    };
    
    // If object is stationary, target velocity should be zero
    if (is_object_stationary) {
      input.target_velocity = {0.0, 0.0, - approach_velocity_, 0.0, 0.0, 0.0};
    } else {
      input.target_velocity = {
        object_vel_.x(), object_vel_.y(), object_vel_.z() - approach_velocity_,
        0.0, 0.0, 0.0
      };
    }
    
    input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    input.max_velocity = { 
      max_velocity_, max_velocity_, max_velocity_, 
      max_angular_velocity_, max_angular_velocity_, max_angular_velocity_ 
    };
    input.max_acceleration = { max_acceleration_, max_acceleration_, max_acceleration_, max_angular_acceleration_, max_angular_acceleration_, max_angular_acceleration_ };
    input.max_jerk = { max_jerk_, max_jerk_, max_jerk_, max_angular_jerk_, max_angular_jerk_, max_angular_jerk_ };
    
    input.minimum_duration = t1;
    
    auto result = otg.calculate(input, trajectory1);
    if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
      RCLCPP_ERROR(this->get_logger(), "First trajectory calculation failed with result: %d", static_cast<int>(result));
      return false;
    }
    
    // Compare the actual trajectory duration with our estimated t1
    double actual_duration = trajectory1.get_duration();
    RCLCPP_INFO(this->get_logger(), "First trajectory planned duration: %.2f s (estimated: %.2f s)", actual_duration, t1);
    
    // If actual duration is larger than our estimate, update t1 and recompute
    if (actual_duration > t1 && iteration < max_iterations) {
      double previous_t1 = t1;
      t1 = actual_duration * 1.1; // Use 1.1 of the actual duration
      RCLCPP_INFO(this->get_logger(), "Updating t1 from %.2f to %.2f for better timing", previous_t1, t1);
      recompute_trajectory = true;
    }
    
  } while (recompute_trajectory);
  
  // Round up the final t1 to the nearest multiple of dt_
  t1 = std::ceil(t1 / dt_) * dt_;
  trajectory1.at_time(trajectory1.get_duration(), end_position, end_velocity, end_acceleration);
  RCLCPP_INFO(this->get_logger(), "Final first trajectory calculated with rounded t1: %.2f s", t1);

  // End timer for first trajectory segment
  auto segment1_end_time = std::chrono::high_resolution_clock::now();
  auto segment1_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      segment1_end_time - start_time).count();
  RCLCPP_INFO(this->get_logger(), "First trajectory segment planning completed in %ld μs", segment1_duration);

  auto prediction_start_time = std::chrono::high_resolution_clock::now();

  /* ---------- grasp‑point prediction ---------- */
  geometry_msgs::msg::Pose obj_pose_msg;
  obj_pose_msg.position.x = object_pos_.x();
  obj_pose_msg.position.y = object_pos_.y();
  obj_pose_msg.position.z = object_pos_.z();
  obj_pose_msg.orientation.w = 1.0;   // (unused – keep identity)

  geometry_msgs::msg::Twist obj_twist_msg;
  obj_twist_msg.linear.x = object_vel_.x();
  obj_twist_msg.linear.y = object_vel_.y();
  obj_twist_msg.linear.z = object_vel_.z();

  // End timer for prediction calculation (before spawning)
  auto prediction_end_time = std::chrono::high_resolution_clock::now();
  auto prediction_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      prediction_end_time - prediction_start_time).count();
  RCLCPP_INFO(this->get_logger(), "Prediction step completed in %ld μs", prediction_duration);

  // Also add cumulative timing
  auto cumulative_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      prediction_end_time - overall_start_time).count();
  RCLCPP_INFO(this->get_logger(), "Trajectory planning + prediction completed in %ld μs (before spawning)",
              cumulative_duration);

  /* ---------- Launch visualization in separate thread ---------- */
  // Only show prediction marker if enabled in parameters
  if (show_prediction_marker_) {
    // Create a copy of variables needed for the thread
    auto node_ptr = this->shared_from_this();
    auto grasp_pred_pub_copy = grasp_pred_pub_;
    auto obj_pose_msg_copy = obj_pose_msg;
    auto obj_twist_msg_copy = obj_twist_msg;
    double t1_copy = t1;
    std::string hardware_type_copy = hardware_type_;
    std::string grasp_marker_xacro_copy = grasp_marker_xacro_;
    
    // Make sure this variable is properly protected for thread safety
    std::mutex marker_mutex;
    bool* has_spawned_marker_ptr = &has_spawned_visual_marker_;

    // Launch the visualization in a separate thread
    std::thread visualization_thread([node_ptr, grasp_pred_pub_copy, obj_pose_msg_copy, obj_twist_msg_copy, 
                                    t1_copy, hardware_type_copy, grasp_marker_xacro_copy, has_spawned_marker_ptr,
                                    &marker_mutex]() {
      std::lock_guard<std::mutex> lock(marker_mutex);
      predictGrasp(
        node_ptr,
        grasp_pred_pub_copy,
        obj_pose_msg_copy,
        obj_twist_msg_copy,
        t1_copy,
        hardware_type_copy,
        grasp_marker_xacro_copy,
        *has_spawned_marker_ptr
      );
    });
    
    // Detach the thread so it runs independently
    visualization_thread.detach();
    
    RCLCPP_INFO(this->get_logger(), "Prediction marker visualization launched");
  } else {
    RCLCPP_INFO(this->get_logger(), "Prediction marker visualization disabled by parameter");
  }
  
  // Continue with trajectory planning immediately
  auto segment2_start_time = std::chrono::high_resolution_clock::now();

  //=============================================================================
  // PHASE 2: SECOND TRAJECTORY
  
  // Calculate new target position for phase 2 based on final t1 value
  // Apply prediction offset if predict_ahead is enabled
  double prediction_time_2 = predict_ahead_ ? t1 + t2_ + prediction_offset_ : t1 + t2_;
  Eigen::Vector3d target_pos_2 = predictObjectPosition(object_pos_, object_vel_, prediction_time_2);
  target_ori = computeDesiredOrientation(object_pos_, object_vel_);
  target_ori_euler = quaternionToEuler(target_ori);
  
  // // Create end pose of first trajectory as starting point for second
  // Eigen::VectorXd ee_start_pose_2(6);
  // ee_start_pose_2 << 
  //   end_position[0], end_position[1], end_position[2],
  //   end_position[3], end_position[4], end_position[5];
  
  // // Create target pose for second phase
  // Eigen::VectorXd target_ee_pose_2(6);
  // target_ee_pose_2 << 
  //   target_pos_2.x(),
  //   target_pos_2.y(),
  //   target_pos_2.z() + basic_offset_,
  //   target_ori_euler.x(),
  //   target_ori_euler.y(),
  //   target_ori_euler.z();
  
  // // Compute adaptive max velocity for second phase
  // double adaptive_max_vel_2 = computeAdaptiveMaxVelocity(
  //   ee_start_pose_2,
  //   target_ee_pose_2,
  //   max_velocity_,
  //   this->get_logger()
  // );

  ruckig::Ruckig<6> otg2;
  ruckig::InputParameter<6> input2;
  ruckig::Trajectory<6> trajectory2;
  
  input2.current_position = end_position;
  input2.current_velocity = end_velocity;
  input2.current_acceleration = end_acceleration;
  
  input2.target_position = {
    target_pos_2.x(), target_pos_2.y(), target_pos_2.z() + basic_offset_,
    target_ori_euler.x(), target_ori_euler.y(), target_ori_euler.z()
  };
  
  // If object is stationary, target velocity should be zero
  if (is_object_stationary) {
    input2.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  } else {
    input2.target_velocity = {
      object_vel_.x(), object_vel_.y(), object_vel_.z(),
      0.0, 0.0, 0.0
    };
  }
  
  input2.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  input2.max_velocity = { 
    max_velocity_, max_velocity_, max_velocity_, 
    max_angular_velocity_, max_angular_velocity_, max_angular_velocity_ 
  };
  input2.max_acceleration = { max_acceleration_, max_acceleration_, max_acceleration_, max_angular_acceleration_, max_angular_acceleration_, max_angular_acceleration_ };
  input2.max_jerk = { max_jerk_, max_jerk_, max_jerk_, max_angular_jerk_, max_angular_jerk_, max_angular_jerk_ };
  
  input2.minimum_duration = t2_;
  
  auto result2 = otg2.calculate(input2, trajectory2);
  if (result2 != ruckig::Result::Working && result2 != ruckig::Result::Finished) {
    RCLCPP_ERROR(this->get_logger(), "Second trajectory calculation failed with result: %d", static_cast<int>(result2));
    return false;
  }
  
  // End timer for second trajectory segment
  auto segment2_end_time = std::chrono::high_resolution_clock::now();
  auto segment2_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      segment2_end_time - segment2_start_time).count();
  RCLCPP_INFO(this->get_logger(), "Second trajectory segment planning completed in %ld μs", segment2_duration);
  
  RCLCPP_INFO(this->get_logger(), "Second trajectory calculated (duration: %.2f s)", trajectory2.get_duration());
  
  trajectory2.at_time(trajectory2.get_duration(), end_position, end_velocity, end_acceleration);

  // If object is stationary, skip phases 3 and 4
  ruckig::Trajectory<6> trajectory3;
  ruckig::Trajectory<6> trajectory4;
  
  if (!is_object_stationary) {
    // Start timer for third trajectory segment
    auto segment3_start_time = std::chrono::high_resolution_clock::now();
    
    //=============================================================================
    // PHASE 3: CONSTANT VELOCITY TRAJECTORY - Only for moving objects
    // Create end pose of second trajectory for velocity computation
    // Eigen::VectorXd ee_start_pose_3(6);
    // ee_start_pose_3 << 
    //   end_position[0], end_position[1], end_position[2],
    //   end_position[3], end_position[4], end_position[5];
    
    // // For phase 3, compute adaptive velocity by creating a future position
    // // We'll create a pose a short distance ahead along object velocity direction
    // Eigen::Vector3d future_pos = Eigen::Vector3d(end_position[0], end_position[1], end_position[2]) + 
    //                             object_vel_.normalized() * 0.05;  // 5cm ahead
    
    // Eigen::VectorXd future_ee_pose(6);
    // future_ee_pose << 
    //   future_pos.x(), future_pos.y(), future_pos.z(),
    //   end_position[3], end_position[4], end_position[5];
    
    // // Compute adaptive max velocity for third phase
    // double adaptive_max_vel_3 = computeAdaptiveMaxVelocity(
    //   ee_start_pose_3,
    //   future_ee_pose,
    //   max_velocity_,
    //   this->get_logger()
    // );
    
    ruckig::Ruckig<6> otg3;
    ruckig::InputParameter<6> input3;
    
    input3.current_position = end_position;
    input3.current_velocity = end_velocity;
    input3.current_acceleration = end_acceleration;
    
    // Set target velocity to match object's velocity (will be maintained constant)
    input3.target_velocity = {
      object_vel_.x(), object_vel_.y(), object_vel_.z(),
      0.0, 0.0, 0.0
    };
    
    input3.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    input3.max_velocity = { 
      max_velocity_, max_velocity_, max_velocity_, 
      max_angular_velocity_, max_angular_velocity_, max_angular_velocity_ 
    };
    input3.max_acceleration = { max_acceleration_, max_acceleration_, max_acceleration_, max_angular_acceleration_, max_angular_acceleration_, max_angular_acceleration_ };
    input3.max_jerk = { max_jerk_, max_jerk_, max_jerk_, max_angular_jerk_, max_angular_jerk_, max_angular_jerk_ };
    
    // Set control interface to velocity mode
    input3.control_interface = ruckig::ControlInterface::Velocity;
    
    // Keep minimum duration at t3 (1 second)
    input3.minimum_duration = t3_;
    
    auto result3 = otg3.calculate(input3, trajectory3);
    if (result3 != ruckig::Result::Working && result3 != ruckig::Result::Finished) {
      RCLCPP_ERROR(this->get_logger(), "Third trajectory calculation failed with result: %d", static_cast<int>(result3));
      return false;
    }
    
    // End timer for third trajectory segment
    auto segment3_end_time = std::chrono::high_resolution_clock::now();
    auto segment3_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        segment3_end_time - segment3_start_time).count();
    RCLCPP_INFO(this->get_logger(), "Third trajectory segment planning completed in %ld μs", segment3_duration);
    
    RCLCPP_INFO(this->get_logger(), "Third trajectory calculated (duration: %.2f s, velocity control mode)", trajectory3.get_duration());
    
    //=============================================================================
    // PHASE 4: DECELERATION PHASE - Using velocity control mode (Only for moving objects)
    
    // Start timer for fourth trajectory segment
    auto segment4_start_time = std::chrono::high_resolution_clock::now();
    
    trajectory3.at_time(trajectory3.get_duration(), end_position, end_velocity, end_acceleration);
    
    ruckig::Ruckig<6> otg4;
    ruckig::InputParameter<6> input4;
    
    // Configure start state
    input4.current_position = end_position;
    input4.current_velocity = end_velocity;
    input4.current_acceleration = end_acceleration;
    
    // Set target velocity to zero (full stop)
    input4.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    input4.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    input4.max_velocity = { 
      max_velocity_, max_velocity_, max_velocity_, 
      max_angular_velocity_, max_angular_velocity_, max_angular_velocity_ 
    };
    input4.max_acceleration = { max_acceleration_, max_acceleration_, max_acceleration_, max_angular_acceleration_, max_angular_acceleration_, max_angular_acceleration_ };
    input4.max_jerk = { max_jerk_, max_jerk_, max_jerk_, max_angular_jerk_, max_angular_jerk_, max_angular_jerk_ };
    
    // Set control interface to velocity mode
    input4.control_interface = ruckig::ControlInterface::Velocity;
    
    auto result4 = otg4.calculate(input4, trajectory4);
    if (result4 != ruckig::Result::Working && result4 != ruckig::Result::Finished) {
      RCLCPP_ERROR(this->get_logger(), "Fourth trajectory calculation failed with result: %d", static_cast<int>(result4));
      return false;
    }
    
    // End timer for fourth trajectory segment
    auto segment4_end_time = std::chrono::high_resolution_clock::now();
    auto segment4_duration = std::chrono::duration_cast<std::chrono::microseconds>(
        segment4_end_time - segment4_start_time).count();
    RCLCPP_INFO(this->get_logger(), "Fourth trajectory segment planning completed in %ld μs", segment4_duration);
    
    RCLCPP_INFO(this->get_logger(), "Fourth trajectory calculated (duration: %.2f s)", trajectory4.get_duration());
  }
  
  // Start timer for trajectory combining and publishing
  auto publish_start_time = std::chrono::high_resolution_clock::now();
  
  //=============================================================================
  // Combine all trajectories into one joint trajectory
  
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names_;
  
  // Set header timestamp for immediate execution only for actual hardware
  if (hardware_type_ == "actual") {
    joint_trajectory.header.stamp = this->now();
  }
  
  std::vector<Eigen::Vector3d> cartesian_positions;
  std::vector<Eigen::Vector3d> cartesian_orientations;
  std::vector<Eigen::Vector3d> cartesian_velocities;
  std::vector<Eigen::Vector3d> angular_velocities;
  
  double total_time = 0.0;
  
  // Sample all trajectory segments using the utility functions
  total_time = sampleTrajectorySegment(
      trajectory1, 1, 0.0, joint_trajectory, 
      cartesian_positions, cartesian_orientations,
      cartesian_velocities, angular_velocities,
      dt_, joint_names_, this->get_logger(), 
      false, include_velocity_);

  total_time = sampleTrajectorySegment(
      trajectory2, 2, total_time, joint_trajectory,
      cartesian_positions, cartesian_orientations,
      cartesian_velocities, angular_velocities,
      dt_, joint_names_, this->get_logger(), 
      true, include_velocity_);

  // Only include phases 3 and 4 if the object is moving
  if (!is_object_stationary) {
    total_time = sampleTrajectorySegment(
        trajectory3, 3, total_time, joint_trajectory,
        cartesian_positions, cartesian_orientations,
        cartesian_velocities, angular_velocities,
        dt_, joint_names_, this->get_logger(), 
        true, include_velocity_);
    
    total_time = sampleTrajectorySegment(
        trajectory4, 4, total_time, joint_trajectory,
        cartesian_positions, cartesian_orientations,
        cartesian_velocities, angular_velocities,
        dt_, joint_names_, this->get_logger(), 
        true, include_velocity_);
  }
  
  // Check if we have a minimum viable trajectory
  if (joint_trajectory.points.size() < 3) {
    RCLCPP_ERROR(this->get_logger(), "Not enough valid points in trajectory (%zu points). Minimum 3 required.", 
                joint_trajectory.points.size());
    return false;
  }
  
  if (enable_trajectory_logging_) {
    saveTrajectoryToCSV(
        joint_trajectory, 
        cartesian_positions, cartesian_orientations,
        cartesian_velocities, angular_velocities,
        joint_names_, trajectory_log_path_,
        object_pos_, object_vel_,
        this->get_logger());
  }
  
  direct_traj_pub_->publish(joint_trajectory);
  
  // End timer for trajectory publishing
  auto publish_end_time = std::chrono::high_resolution_clock::now();
  auto publish_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      publish_end_time - publish_start_time).count();
  RCLCPP_INFO(this->get_logger(), "Trajectory combining and publishing completed in %ld μs", publish_duration);
  
  // End timer for overall process
  auto overall_end_time = std::chrono::high_resolution_clock::now();
  auto overall_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      overall_end_time - overall_start_time).count();
  RCLCPP_INFO(this->get_logger(), "Total trajectory planning process completed in %ld μs", overall_duration);
  
  RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu points", joint_trajectory.points.size());
  
  return true;
}
#pragma endregion Main trajectory

#pragma region stop
bool TrajectoryPlannerNode::stopTrajectory()
{
  // Check if we have current pose data
  if (ee_pose_.header.stamp.sec == 0) {
    RCLCPP_ERROR(this->get_logger(), "No valid end-effector pose data for stop");
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Planning stop trajectory");

  // Initialize trajectory message
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names_;
  
  // Set header timestamp for immediate execution only for actual hardware
  if (hardware_type_ == "actual") {
    joint_trajectory.header.stamp = this->now();
  }

  // Extract current position and velocity
  Eigen::Vector3d current_position(
    ee_pose_.pose.position.x,
    ee_pose_.pose.position.y,
    ee_pose_.pose.position.z
  );

  // Setup Ruckig for deceleration planning
  ruckig::Ruckig<6> otg;
  ruckig::InputParameter<6> input;
  ruckig::Trajectory<6> trajectory;

  // Configure start state
  input.current_position = {
    current_position.x(), current_position.y(), current_position.z(),
    ee_euler_angles_.x(), ee_euler_angles_.y(), ee_euler_angles_.z()
  };
  
  input.current_velocity = {
    ee_velocity_.x(), ee_velocity_.y(), ee_velocity_.z(),
    ee_euler_rates_.x(), ee_euler_rates_.y(), ee_euler_rates_.z()
  };

  std::cout << "Current velocity: " << input.current_velocity[0] << ", " 
            << input.current_velocity[1] << ", " 
            << input.current_velocity[2] << ", "
            << input.current_velocity[3] << ", "
            << input.current_velocity[4] << ", "
            << input.current_velocity[5] << std::endl;
  
  input.current_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Set target velocity to zero (full stop)
  input.target_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input.target_acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
  // Configure limits
  input.max_velocity = { max_velocity_, max_velocity_, max_velocity_, 
                         max_angular_velocity_, max_angular_velocity_, max_angular_velocity_ };
  input.max_acceleration = { max_acceleration_, max_acceleration_, max_acceleration_, 
                             max_angular_acceleration_, max_angular_acceleration_, max_angular_acceleration_ };
  input.max_jerk = { max_jerk_, max_jerk_, max_jerk_, 
                     max_angular_jerk_, max_angular_jerk_, max_angular_jerk_ };
  
  // Set control interface to velocity mode (we want to control velocity to bring it to zero)
  input.control_interface = ruckig::ControlInterface::Velocity;
  
  // Calculate the trajectory
  auto result = otg.calculate(input, trajectory);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
    RCLCPP_ERROR(this->get_logger(), "stop trajectory calculation failed with result: %d", static_cast<int>(result));
    return false;
  }
  
  // Get total deceleration time
  double stop_time = trajectory.get_duration();
  RCLCPP_INFO(this->get_logger(), "stop trajectory calculated (duration: %.2f s)", stop_time);

  // Get current joint positions using inverse kinematics
  Eigen::VectorXd current_pose(6);
  current_pose << 
    current_position.x(), current_position.y(), current_position.z(),
    ee_euler_angles_.x(), ee_euler_angles_.y(), ee_euler_angles_.z();
  
  Eigen::VectorXd current_joint_positions = analyticIK(current_pose);
  
  if (current_joint_positions.size() != 0) {
    // Create current Cartesian velocities
    Eigen::VectorXd cartesian_vel(6);
    cartesian_vel << 
        ee_velocity_.x(), ee_velocity_.y(), ee_velocity_.z(),
        ee_euler_rates_.x(), ee_euler_rates_.y(), ee_euler_rates_.z();
    
    // Get the Jacobian to convert Cartesian to joint velocities
    Eigen::MatrixXd jacobian = analyticJacobian(current_joint_positions);
    
    // Convert Cartesian velocities to joint velocities
    Eigen::VectorXd joint_vel = cartesianToJointVel(jacobian, cartesian_vel);
    
    // Create first trajectory point (current position with current velocities at t=0)
    trajectory_msgs::msg::JointTrajectoryPoint first_point;
    first_point.positions = {
      current_joint_positions(0), current_joint_positions(1), current_joint_positions(2),
      current_joint_positions(3), current_joint_positions(4), current_joint_positions(5)
    };
    first_point.velocities = {
      joint_vel(0), joint_vel(1), joint_vel(2), 
      joint_vel(3), joint_vel(4), joint_vel(5)
    };
    first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
    
    // Add the first point to the trajectory
    joint_trajectory.points.push_back(first_point);
    
    RCLCPP_INFO(this->get_logger(), "Added current position point at t=0.0 s");
  } else {
    RCLCPP_WARN(this->get_logger(), "Failed to compute IK for current position in stop trajectory");
  }

  // Create the final point at the end time
  std::array<double, 6> final_position, final_velocity, final_acceleration;
  trajectory.at_time(stop_time, final_position, final_velocity, final_acceleration);
  
  // Convert Cartesian to joint angles using inverse kinematics
  Eigen::VectorXd target_pose(6);
  target_pose << 
    final_position[0], final_position[1], final_position[2],
    final_position[3], final_position[4], final_position[5];
  Eigen::VectorXd joint_positions = analyticIK(target_pose);

  if (joint_positions.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to compute inverse kinematics for stop");
    return false;
  }

  // Create final trajectory point
  trajectory_msgs::msg::JointTrajectoryPoint final_point;
  final_point.positions = { 
    joint_positions(0), joint_positions(1), joint_positions(2),
    joint_positions(3), joint_positions(4), joint_positions(5) 
  };
  final_point.velocities.resize(6, 0.0);  // Zero velocities at final point
  final_point.accelerations.resize(6, 0.0);  // Zero accelerations at final point

  final_point.time_from_start = rclcpp::Duration::from_seconds(stop_time);
  
  // Add the final point to the trajectory
  joint_trajectory.points.push_back(final_point);
  
  // Publish the trajectory
  direct_traj_pub_->publish(joint_trajectory);
  RCLCPP_INFO(this->get_logger(), "Published stop trajectory with %zu points", joint_trajectory.points.size());
  
  return true;
}
#pragma endregion stop

#pragma region To-rest-pose trajectory
bool TrajectoryPlannerNode::goToRestTrajectory()
{
  // Check if we have current pose data
  if (ee_pose_.header.stamp.sec == 0) {
    RCLCPP_WARN(this->get_logger(), "No valid end-effector pose data. Using single-point trajectory.");
    
    // Define the rest pose in joint space
    std::vector<double> rest_position = {0.0, -1.2, 1.2, 0.0, 0.5, 0.0}; // rest position
    std::vector<double> rest_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // zero velocity
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = rest_position;
    point.velocities = rest_velocity;
    point.time_from_start = rclcpp::Duration::from_seconds(3.0); // Duration of 3 second
    
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names_;
    
    // Set header timestamp for immediate execution only for actual hardware
    if (hardware_type_ == "actual") {
      joint_trajectory.header.stamp = this->now();
    }
    
    joint_trajectory.points.push_back(point);
    
    direct_traj_pub_->publish(joint_trajectory);
    RCLCPP_INFO(this->get_logger(), "Published single-point rest pose trajectory");
    
    return true;
  }
  
  // Create a JointTrajectory message
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  joint_trajectory.joint_names = joint_names_;
  
  // Set header timestamp for immediate execution only for actual hardware
  if (hardware_type_ == "actual") {
    joint_trajectory.header.stamp = this->now();
  }
  
  // Extract current position and orientation
  Eigen::Vector3d current_position(
    ee_pose_.pose.position.x,
    ee_pose_.pose.position.y,
    ee_pose_.pose.position.z
  );
  
  // Create pose vector for current position
  Eigen::VectorXd current_pose(6);
  current_pose << 
    current_position.x(), current_position.y(), current_position.z(),
    ee_euler_angles_.x(), ee_euler_angles_.y(), ee_euler_angles_.z();
  
  // Calculate joint positions for the current pose
  Eigen::VectorXd current_joint_solution = analyticIK(current_pose);
  if (current_joint_solution.size() == 0) {
    RCLCPP_WARN(this->get_logger(), "Failed to compute IK for current position. Using single-point trajectory.");
    
    std::vector<double> rest_position = {0.0, -1.2, 1.2, 0.0, 0.5, 0.0};
    std::vector<double> rest_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = rest_position;
    point.velocities = rest_velocity;
    point.time_from_start = rclcpp::Duration::from_seconds(3.0);
    
    joint_trajectory.points.push_back(point);
  } else {
    // Calculate projected position after 0.6 seconds
    const double projection_time = 0.6;  // seconds
    Eigen::Vector3d projected_position = current_position + ee_velocity_ * projection_time;
    projected_position.z() += approach_offset_ / 2; // Adjust Z position for approach offset
    
    // Create pose vector for projected position
    Eigen::VectorXd projected_pose(6);
    projected_pose << 
      projected_position.x(), projected_position.y(), projected_position.z(),
      ee_euler_angles_.x(), ee_euler_angles_.y(), ee_euler_angles_.z();
    
    // Calculate joint positions for the projected pose
    Eigen::VectorXd projected_joint_solution = analyticIK(projected_pose);
    if (projected_joint_solution.size() == 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to compute IK for projected position. Using two-point trajectory.");
      
      // Create current Cartesian velocities
      Eigen::VectorXd cartesian_vel(6);
      cartesian_vel << 
          ee_velocity_.x(), ee_velocity_.y(), ee_velocity_.z(),
          ee_euler_rates_.x(), ee_euler_rates_.y(), ee_euler_rates_.z();
      
      // Get the Jacobian at the current configuration
      Eigen::MatrixXd jacobian = analyticJacobian(current_joint_solution);
      
      // Convert Cartesian velocities to joint velocities
      Eigen::VectorXd joint_vel = cartesianToJointVel(jacobian, cartesian_vel);
      
      // Create first trajectory point (current position with current velocities)
      trajectory_msgs::msg::JointTrajectoryPoint first_point;
      first_point.positions = {
        current_joint_solution[0], current_joint_solution[1], current_joint_solution[2],
        current_joint_solution[3], current_joint_solution[4], current_joint_solution[5]
      };
      first_point.velocities = {
      joint_vel(0), joint_vel(1), joint_vel(2), 
        joint_vel(3), joint_vel(4), joint_vel(5)
      };
      first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
      
      // Create second trajectory point (rest pose)
      trajectory_msgs::msg::JointTrajectoryPoint second_point;
      second_point.positions = {0.0, -1.2, 1.2, 0.0, 0.5, 0.0};
      second_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      second_point.time_from_start = rclcpp::Duration::from_seconds(3.0);
      
      joint_trajectory.points.push_back(first_point);
      joint_trajectory.points.push_back(second_point);
    } else {
      // Create current Cartesian velocities
      Eigen::VectorXd cartesian_vel(6);
      cartesian_vel << 
          ee_velocity_.x(), ee_velocity_.y(), ee_velocity_.z(),
          ee_euler_rates_.x(), ee_euler_rates_.y(), ee_euler_rates_.z();
      
      // Get the Jacobian at the current configuration
      Eigen::MatrixXd jacobian = analyticJacobian(current_joint_solution);
      
      // Convert Cartesian velocities to joint velocities
      Eigen::VectorXd joint_vel = cartesianToJointVel(jacobian, cartesian_vel);
      
      // Create first trajectory point (current position with current velocities at t=0)
      trajectory_msgs::msg::JointTrajectoryPoint first_point;
      first_point.positions = {
        current_joint_solution[0], current_joint_solution[1], current_joint_solution[2],
        current_joint_solution[3], current_joint_solution[4], current_joint_solution[5]
      };
      first_point.velocities = {
        joint_vel(0), joint_vel(1), joint_vel(2), 
        joint_vel(3), joint_vel(4), joint_vel(5)
      };
      first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
      
      // Create modified Cartesian velocities for projected position (with Z adjustment)
      Eigen::VectorXd modified_cartesian_vel(6);
      modified_cartesian_vel << 
          ee_velocity_.x(), ee_velocity_.y(), ee_velocity_.z() + approach_velocity_ * 2,
          ee_euler_rates_.x(), ee_euler_rates_.y(), ee_euler_rates_.z();
      
      // Get the Jacobian at the projected configuration
      Eigen::MatrixXd projected_jacobian = analyticJacobian(projected_joint_solution);
      
      // Convert modified Cartesian velocities to joint velocities
      Eigen::VectorXd projected_joint_vel = cartesianToJointVel(projected_jacobian, modified_cartesian_vel);
      
      // Create second trajectory point (projected position)
      trajectory_msgs::msg::JointTrajectoryPoint second_point;
      second_point.positions = {
        projected_joint_solution[0], projected_joint_solution[1], projected_joint_solution[2],
        projected_joint_solution[3], projected_joint_solution[4], projected_joint_solution[5]
      };
      second_point.velocities = {
        projected_joint_vel(0), projected_joint_vel(1), projected_joint_vel(2), 
        projected_joint_vel(3), projected_joint_vel(4), projected_joint_vel(5)
      };
      second_point.time_from_start = rclcpp::Duration::from_seconds(projection_time);
      
      // Create third trajectory point (rest pose)
      trajectory_msgs::msg::JointTrajectoryPoint third_point;
      third_point.positions = {0.0, -1.2, 1.2, 0.0, 0.5, 0.0};
      third_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      third_point.time_from_start = rclcpp::Duration::from_seconds(3.0);
      
      // Add all three points to the trajectory
      joint_trajectory.points.push_back(first_point);
      joint_trajectory.points.push_back(second_point);
      joint_trajectory.points.push_back(third_point);
      
      RCLCPP_INFO(this->get_logger(), "Created three-point trajectory: current (t=0), projected (t=0.2s), rest (t=3.0s)");
    }
  }
  
  // Publish the trajectory
  direct_traj_pub_->publish(joint_trajectory);
  RCLCPP_INFO(this->get_logger(), "Published trajectory with %zu points", joint_trajectory.points.size());
  
  return true;
}
#pragma endregion To-rest-pose trajectory
#pragma endregion Trajectory planning methods

} // namespace dynamic_grasping_controller

//=============================================================================
// Main function using MultiThreadedExecutor
//=============================================================================
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<dynamic_grasping_controller::TrajectoryPlannerNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}