#include "dynamic_grasping_controller/dynamic_grasping_manager_node.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <cstdlib>   // for std::system
#include <chrono>
#include <numeric>   // for std::accumulate

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace dynamic_grasping_controller;  // Add this line

//=============================================================================
// Constructor
//=============================================================================
DynamicGraspingManager::DynamicGraspingManager()
: Node("dynamic_grasping_manager")
{
  // --- Parameter Declaration -------------------------------------------
  this->declare_parameter("monitoring_rate", 30.0);
  this->declare_parameter("inner_radius", 0.08);
  this->declare_parameter("stabilization_samples", 6);
  this->declare_parameter("velocity_diff_threshold", 0.01);
  this->declare_parameter("gripper_close_value", "0.03");
  this->declare_parameter("hardware_type", "gz_classic");
  this->declare_parameter("object_name", "cylinder");
  this->declare_parameter("basic_offset", 0.01);
  this->declare_parameter("max_velocity", 0.3);      // Declare max_velocity parameter
  this->declare_parameter("max_acceleration", 0.4);  // Declare max_acceleration parameter

  // --- Get Parameters -------------------------------------------------
  monitoring_rate_ = this->get_parameter("monitoring_rate").as_double();
  inner_radius_ = this->get_parameter("inner_radius").as_double();
  stabilization_samples_ = this->get_parameter("stabilization_samples").as_int();
  velocity_diff_threshold_ = this->get_parameter("velocity_diff_threshold").as_double();
  gripper_close_value_ = this->get_parameter("gripper_close_value").as_string();
  hardware_type_ = this->get_parameter("hardware_type").as_string();
  object_name_ = this->get_parameter("object_name").as_string();
  basic_offset_ = this->get_parameter("basic_offset").as_double();
  max_velocity_ = this->get_parameter("max_velocity").as_double();      // Get max_velocity parameter
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();  // Get max_acceleration parameter

  // --- Initialization -------------------------------------------------
  // Auto-start only for the first run in simulation
  started_ = (hardware_type_ != "actual");
  // vel_buffer_ is initialized by deque's default constructor

  // Initialize tracking flags
  gripper_close_sent_ = false;

  // --- Subscribers ----------------------------------------------------
  object_state_sub_ = this->create_subscription<wx250s_bringup::msg::ObjectState>(
    "/object_state", 10, std::bind(&DynamicGraspingManager::object_state_callback, this, _1));
  
  ee_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/wx250s/end_effector_pose", 10, std::bind(&DynamicGraspingManager::ee_pose_callback, this, _1));

  // Create start subscription for ALL hardware types
  start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/start", 10, std::bind(&DynamicGraspingManager::start_callback, this, _1));
  
  // Only display waiting message if we need a start command
  if (!started_) {
    RCLCPP_INFO(get_logger(), "Waiting for start command on /start");
  } else {
    RCLCPP_INFO(get_logger(), "Auto-started (simulation first run). Future runs will require start command.");
  }

  gripper_closed_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/gripper_closed", 10, std::bind(&DynamicGraspingManager::gripper_closed_callback, this, _1));

  // --- Publishers -----------------------------------------------------
  start_grasping_pub_ = this->create_publisher<std_msgs::msg::Int8>("/start_dynamic_grasping", 10);
  gripper_command_pub_ = this->create_publisher<std_msgs::msg::String>("/gripper_command", 10);

  // --- Service Clients ------------------------------------------------
  attach_link_client_ = this->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");

  // --- Timers --------------------------------------------------------
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / monitoring_rate_),
    std::bind(&DynamicGraspingManager::monitor_callback, this));

  RCLCPP_INFO(get_logger(), "Dynamic Grasping Manager initialized with parameters:");
  RCLCPP_INFO(get_logger(), "  monitoring_rate: %.1f Hz", monitoring_rate_);
  RCLCPP_INFO(get_logger(), "  inner_radius: %.2f m", inner_radius_);
  RCLCPP_INFO(get_logger(), "  hardware_type: %s", hardware_type_.c_str());
  RCLCPP_INFO(get_logger(), "  object_name: %s", object_name_.c_str());
  RCLCPP_INFO(get_logger(), "  max_velocity: %.2f m/s", max_velocity_);
  RCLCPP_INFO(get_logger(), "  max_acceleration: %.2f m/s²", max_acceleration_);
}

//=============================================================================
// Subscription Callbacks
//=============================================================================
void DynamicGraspingManager::start_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && !started_) {
    RCLCPP_INFO(get_logger(), "Start command received");
    started_ = true;
  }
}

void DynamicGraspingManager::object_state_callback(const wx250s_bringup::msg::ObjectState::SharedPtr msg) {
  object_position_ = Eigen::Vector3d{msg->position.x, msg->position.y, msg->position.z};
  object_velocity_ = Eigen::Vector3d{msg->velocity.x, msg->velocity.y, msg->velocity.z};
}

void DynamicGraspingManager::ee_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  ee_position_ = Eigen::Vector3d{
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  };
}

void DynamicGraspingManager::gripper_closed_callback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && grasping_in_progress_ && !gripper_closed_) {
    RCLCPP_INFO(get_logger(), "Gripper closed signal received");
    gripper_closed_ = true;
    std::cout << "Gripper closed recognized" << std::endl;
    
    // Request link attachment immediately (only for simulation)
    if (hardware_type_ != "actual") {
      RCLCPP_INFO(get_logger(), "Requesting link attachment");
      request_link_attachment();
    }
    
    // Create a one-shot timer to delay command 3 by 0.5 seconds
    delayed_command_timer_ = this->create_wall_timer(
      250ms, // 0.1 seconds
      [this]() {
        // Send command 3 after the delay
        auto start_msg = std_msgs::msg::Int8();
        start_msg.data = 3;
        start_grasping_pub_->publish(start_msg);
        
        // Reset state after sending the command
        RCLCPP_INFO(get_logger(), "Resetting manager state");
        reset_state();
        
        // Cancel the timer - only execute once
        if (delayed_command_timer_) {
          delayed_command_timer_->cancel();
        }
      }
    );
  }
}

//=============================================================================
// Trajectory Analysis Functions
//=============================================================================

//=============================================================================
// Trajectory Analysis Functions
//=============================================================================
void DynamicGraspingManager::detect_direction_change() {
  if (!object_velocity_) return;

  // Add latest velocity to rolling buffer
  vel_buffer_.push_back(*object_velocity_);
  // Ensure buffer does not exceed capacity
  while (vel_buffer_.size() > static_cast<size_t>(stabilization_samples_)) {
      vel_buffer_.pop_front();
  }

  if (vel_buffer_.size() < static_cast<size_t>(stabilization_samples_)) return;

  // Compute average velocity vector from buffer samples
  Eigen::Vector3d avg = std::accumulate(vel_buffer_.begin(), vel_buffer_.end(), 
                         Eigen::Vector3d::Zero().eval()) / vel_buffer_.size();

  if (!has_last_avg_velocity_) {
    last_avg_velocity_ = avg;
    has_last_avg_velocity_ = true;
    return;
  }

  double diff_norm = (avg - last_avg_velocity_).norm();
  
  // Only plan stops when velocity changes significantly AND gripper close command hasn't been sent yet
  if (diff_norm >= velocity_diff_threshold_ && !gripper_close_sent_) {
    if (grasping_in_progress_ && !velocity_change_signaled_) {
      RCLCPP_INFO(get_logger(), "Velocity change detected (Δ‖v‖ = %.3f) - planning stop", diff_norm);
      auto msg = std_msgs::msg::Int8();
      msg.data = 2;  // Signal stop
      start_grasping_pub_->publish(msg);
      velocity_change_signaled_ = true;
    }
    waiting_for_stabilization_ = true;
    stable_velocity_count_ = 0;
    vel_buffer_.clear(); // Reset buffer to wait for new stabilization
    last_avg_velocity_ = avg; // Update reference velocity
    return;
  } else if (diff_norm >= velocity_diff_threshold_ && gripper_close_sent_) {
    // Log that we detected a change but ignoring it because gripper close command was already sent
    RCLCPP_INFO(get_logger(), "Velocity change detected (Δ‖v‖ = %.3f) but grip command already sent - ignoring", diff_norm);
    last_avg_velocity_ = avg; // Still update reference velocity
    return;
  }

  if (waiting_for_stabilization_ && !gripper_close_sent_) {
    if (diff_norm < velocity_diff_threshold_) {
      stable_velocity_count_++;
      if (stable_velocity_count_ >= stabilization_samples_) {
        RCLCPP_INFO(get_logger(), "Velocity has stabilized after direction change");
        can_retrigger_ = true;
        waiting_for_stabilization_ = false;
        stable_velocity_count_ = 0;
        velocity_change_signaled_ = false; // Reset signal flag
      }
    } else {
      stable_velocity_count_ = 0; // Reset stability counter if velocity changes significantly again
      last_avg_velocity_ = avg; // Update reference velocity to current average
    }
  }
}

//=============================================================================
// Grasping Action Functions
//=============================================================================
void DynamicGraspingManager::request_link_attachment() {
  if (!attach_link_client_->wait_for_service(1s)) {
    RCLCPP_WARN(get_logger(), "AttachLink service not available after waiting");
    return;
  }
  
  auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
  request->model1_name = "wx250s"; // Make sure this matches your robot model name in Gazebo
  request->link1_name  = "wx250s/ee_gripper_link"; // Make sure this matches the link name
  request->model2_name = object_name_; // Use the parameter for object name
  request->link2_name  = object_name_ + "_link"; // Assuming object link is named like this

  // Asynchronous call
  auto future_result = attach_link_client_->async_send_request(request,
    [this, request](rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedFuture future) {
        try {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(get_logger(), "Successfully attached link %s to %s",
                            request->link2_name.c_str(), request->link1_name.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Failed to attach link: %s", response->message.c_str());
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
    });

  RCLCPP_INFO(get_logger(), "AttachLink request sent for %s", object_name_.c_str());
}

void DynamicGraspingManager::start_grasping_sequence() {
  // Ensure gripper is open before starting
  auto open_msg = std_msgs::msg::String();
  open_msg.data = "released"; // Command to open gripper
  gripper_command_pub_->publish(open_msg);

  // Signal trajectory planner to start
  auto start_msg = std_msgs::msg::Int8();
  start_msg.data = 1; // Signal normal trajectory planning
  start_grasping_pub_->publish(start_msg);

  // Update state variables
  grasping_in_progress_ = true;
  gripper_closed_ = false; // Gripper starts open
  rest_pose_ = false; // No longer in rest pose
  attachment_requested_ = false; // Reset attachment request flag
  can_retrigger_ = false; // Reset retrigger flag
  velocity_change_signaled_ = false; // Reset velocity change signal flag
  has_last_avg_velocity_ = false; // Reset velocity tracking for the new sequence
  vel_buffer_.clear();
  gripper_close_sent_ = false; // Reset flag for tracking gripper close commands

  RCLCPP_INFO(get_logger(), "Grasping sequence initiated.");
}


//=============================================================================
// Reset State Function
//=============================================================================
void DynamicGraspingManager::reset_state() {
  RCLCPP_INFO(get_logger(), "Resetting manager state");
  
  // Reset all state variables
  grasping_in_progress_ = false;
  gripper_closed_ = false;
  rest_pose_ = false;
  attachment_requested_ = false;
  can_retrigger_ = false;
  waiting_for_stabilization_ = false;
  stable_velocity_count_ = 0;
  velocity_change_signaled_ = false;
  in_graspable_zone_ = false;
  gripper_close_sent_ = false; // Reset the gripper close command tracking flag
  
  // Clear buffers
  vel_buffer_.clear();
  has_last_avg_velocity_ = false;
  
  // After reset, ALWAYS require a new start command regardless of hardware type
  started_ = false;
  RCLCPP_INFO(get_logger(), "Waiting for new start command on /start");
}

//=============================================================================
// Main Monitoring Logic
//=============================================================================
void DynamicGraspingManager::monitor_callback() {
  // Only proceed if started (regardless of hardware type)
  if (!started_) return;
  
  if (!object_position_ || !ee_position_ || !object_velocity_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Waiting for object/EE data...");
    return;
  }

  // Run velocity change detection (keep original function)
  detect_direction_change();

  /* ---------- rich trajectory prediction ---------- */
  const auto pred = objectTrajectoryEstimator(
      *object_position_, 
      *object_velocity_, 
      inner_radius_);

  const bool object_inside_now = (pred.idx_enter == 0);  // First sample already in workspace

  /* ---------- skip if never enters reachable workspace ---------- */
  if (pred.idx_enter < 0) return;

  const double entrance_horizon = pred.t[pred.idx_enter];   // τₑ
  const double inside_duration  = pred.inside_window;       // Δτₚ
  const double obj_speed        = object_velocity_->norm(); // ‖v‖

  /* ---------- time robot really needs ---------- */
  const Eigen::Vector3d& entrance_point = pred.pos[pred.idx_enter];
  const double d_ee_to_entry = (*ee_position_ - entrance_point).norm();
  // Updated to use the member variables instead of global constants
  const double approach_time = estimateEETravelTime(d_ee_to_entry, max_velocity_, max_acceleration_);  // τᵣ

  /* ---------- decision logic ---------- */
  bool fire = false;
  
  // Check if object is stationary (very low velocity)
  const bool is_stationary = (obj_speed < 1e-4);
  
  if (!object_inside_now) {
    const double lower = std::max(0.0, approach_time - (inside_duration - 2.0));  // τᵣ - (Δτₚ-2)
    const bool timing_ok = (entrance_horizon >= lower) && 
                          (entrance_horizon <= approach_time);
    
    if (is_stationary) {
      // For stationary objects, only check if it's reachable and has sufficient duration
      fire = (inside_duration >= 2.0);
    } else {
      // For moving objects, use original logic
      fire = (obj_speed < 0.9 * max_velocity_) &&
            (inside_duration >= 2.0) &&
            timing_ok;
    }
  } else {  // already inside workspace
    if (is_stationary) {
      // For stationary objects already inside, can grasp immediately
      fire = true;
    } else {
      // For moving objects, use original logic
      fire = (inside_duration >= 5.0);
    }
  }

  /* ---------- Handle ongoing grasping sequence ---------- */
  if (grasping_in_progress_) {
    // Only proceed if gripper is not yet closed
    if (!gripper_closed_) {
      // Handle retriggering or closing gripper
      if (can_retrigger_) {
        // Retrigger trajectory planning
        RCLCPP_INFO(get_logger(), "Retriggering trajectory planning due to stabilized velocity change");
        start_grasping_pub_->publish(std_msgs::msg::Int8().set__data(1));
        can_retrigger_ = false;
      } else if (d_ee_to_entry < basic_offset_ + 0.03 && !gripper_close_sent_) {
        // Send gripper close command (only once)
        RCLCPP_INFO(get_logger(), "Object within grasp distance (%.3f m). Closing gripper.", d_ee_to_entry);
        gripper_command_pub_->publish(std_msgs::msg::String().set__data(gripper_close_value_));
        gripper_close_sent_ = true;
      }
    }
    return; // Skip rest of function while grasping is active
  }

  /* ---------- launch grasp if conditions are met & not in progress ---------- */
  if (fire) {
    if (is_stationary) {
      RCLCPP_INFO(get_logger(),
        "Starting grasp for STATIONARY object: τᵣ=%.2f s, Δτₚ=%.2f s, |v|=%.4f m/s",
        approach_time, inside_duration, obj_speed);
    } else {
      RCLCPP_INFO(get_logger(),
        "Starting grasp for MOVING object: τᵣ=%.2f s, τₑ=%.2f s, Δτₚ=%.2f s, |v|=%.2f m/s",
        approach_time, entrance_horizon, inside_duration, obj_speed);
    }
    start_grasping_sequence();
  }
}

//=============================================================================
// Main Function
//=============================================================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicGraspingManager>());
  rclcpp::shutdown();
  return 0;
}

