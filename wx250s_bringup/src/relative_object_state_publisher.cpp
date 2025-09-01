/**
 * @file relative_object_state_publisher.cpp
 * @brief A node to calculate and publish relative position and velocity between robot and object
 *        using actual hardware data from motion capture system
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "wx250s_bringup/msg/object_state.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <string>
#include <rclcpp/qos.hpp>
#include <Eigen/Geometry> // Add for quaternion/rotation operations
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class RelativeObjectStatePublisher : public rclcpp::Node
{
public:
  RelativeObjectStatePublisher()
  : Node("relative_object_state_publisher"),
    have_robot_pose_(false),
    have_robot_twist_(false),
    have_object_pose_(false),
    have_object_twist_(false)
  {
    // Declare parameters
    this->declare_parameter("object_name", "red_cylinder");
    this->declare_parameter("robot_base_frame", "wx250s/base_link");
    this->declare_parameter("update_rate", 30.0);
    
    // Get parameters
    object_name_ = this->get_parameter("object_name").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    
    // Create publisher for object state
    publisher_ = this->create_publisher<wx250s_bringup::msg::ObjectState>("/object_state", 10);
    
    // Define a QoS profile (SensorData is often suitable for sensor streams)
    auto sensor_qos = rclcpp::SensorDataQoS();

    // Subscribe to robot pose (assuming default QoS is working for this one)
    robot_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/wx250s/pose", 10,
      std::bind(&RelativeObjectStatePublisher::robotPoseCallback, this, std::placeholders::_1));

    // Subscribe to robot twist - Specify QoS
    robot_twist_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/wx250s/twist", sensor_qos, // <<< Use explicit QoS
      std::bind(&RelativeObjectStatePublisher::robotTwistCallback, this, std::placeholders::_1));

    // Subscribe to object pose using parameter-based topic name
    std::string object_pose_topic = "/" + object_name_ + "/pose";
    object_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic, 10,
      std::bind(&RelativeObjectStatePublisher::objectPoseCallback, this, std::placeholders::_1));

    // Subscribe to object twist using parameter-based topic name - Specify QoS
    std::string object_twist_topic = "/" + object_name_ + "/twist";
    object_twist_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      object_twist_topic, sensor_qos, // <<< Use explicit QoS
      std::bind(&RelativeObjectStatePublisher::objectTwistCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Relative object state publisher initialized (hardware mode)");
    RCLCPP_INFO(this->get_logger(), "Tracking object: %s", object_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Object pose topic: %s", object_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Object twist topic: %s", object_twist_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Using robot base frame: %s", robot_base_frame_.c_str());
    
    // Add debug logging to check data arrival
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&RelativeObjectStatePublisher::debugCallback, this));
  }

private:
  void debugCallback()
  {
    // Log what data we're still waiting for
    std::string missing_data = "";
    if (!have_robot_pose_) missing_data += "robot_pose ";
    if (!have_robot_twist_) missing_data += "robot_twist ";
    if (!have_object_pose_) missing_data += "object_pose ";
    if (!have_object_twist_) missing_data += "object_twist ";

    if (!missing_data.empty()) {
      RCLCPP_WARN(this->get_logger(), "Waiting for data: %s", missing_data.c_str());
    } else {
      if (timer_) { // Check if timer still exists before cancelling
         RCLCPP_INFO_ONCE(this->get_logger(), "All required data received, cancelling debug timer."); // Log only once
         timer_->cancel();
         timer_.reset(); // Release the timer pointer
      }
    }
  }

  void robotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Store robot position directly from PoseStamped
    robot_pose_ = msg->pose;
    have_robot_pose_ = true;
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Received first robot pose message");
    
    // Try to publish state if we have all necessary data
    tryPublishState();
  }

  // Callback CHANGED to accept TwistStamped
  void robotTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    // Store robot twist from the 'twist' field of TwistStamped
    robot_twist_ = msg->twist; // <<< Extract Twist part
    have_robot_twist_ = true;

    RCLCPP_INFO_ONCE(this->get_logger(), "Received first robot twist message");
    
    // Try to publish state if we have all necessary data
    tryPublishState();
  }
  
  // Changed from nav_msgs/Odometry to geometry_msgs/PoseStamped
  void objectPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    // Store object position directly from PoseStamped
    object_pose_ = msg->pose;
    have_object_pose_ = true;
    
    RCLCPP_INFO_ONCE(this->get_logger(), "Received first object pose message");
    
    // Try to publish state if we have all necessary data
    tryPublishState();
  }
  
  // Callback CHANGED to accept TwistStamped
  void objectTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    // Store object twist from the 'twist' field of TwistStamped
    object_twist_ = msg->twist; // <<< Extract Twist part
    have_object_twist_ = true;

    RCLCPP_INFO_ONCE(this->get_logger(), "Received first object twist message");
    
    // Try to publish state if we have all necessary data
    tryPublishState();
  }
  
  void tryPublishState()
  {
    // Check if we have all necessary data
    if (!have_robot_pose_ || !have_robot_twist_ || !have_object_pose_ || !have_object_twist_) {
      return;
    }
    
    // Create object state message
    auto state_msg = wx250s_bringup::msg::ObjectState();
    
    // Calculate the relative position of the object in the robot's reference frame
    // First, compute the vector from robot to object in world frame
    Eigen::Vector3d object_position_world(
      object_pose_.position.x,
      object_pose_.position.y,
      object_pose_.position.z
    );
    
    Eigen::Vector3d robot_position_world(
      robot_pose_.position.x,
      robot_pose_.position.y,
      robot_pose_.position.z
    );
    
    // Convert robot's orientation quaternion to rotation matrix
    Eigen::Quaterniond robot_orientation(
      robot_pose_.orientation.w,
      robot_pose_.orientation.x,
      robot_pose_.orientation.y,
      robot_pose_.orientation.z
    );
    
    // Rotation matrix from world to robot frame
    Eigen::Matrix3d rotation_world_to_robot = robot_orientation.conjugate().toRotationMatrix();
    
    // Vector from robot to object in world frame
    Eigen::Vector3d vector_to_object_world = object_position_world - robot_position_world;
    
    // Transform this vector to the robot's frame by applying the inverse rotation
    Eigen::Vector3d vector_to_object_robot = rotation_world_to_robot * vector_to_object_world;
    
    // Store the result in the message
    state_msg.position.x = vector_to_object_robot.x();
    state_msg.position.y = vector_to_object_robot.y();
    state_msg.position.z = vector_to_object_robot.z();
    
    // For velocity, we should also transform to the robot's frame
    // First, get the linear velocities in world frame
    Eigen::Vector3d object_vel_world(
      object_twist_.linear.x,
      object_twist_.linear.y,
      object_twist_.linear.z
    );
    
    Eigen::Vector3d robot_vel_world(
      robot_twist_.linear.x,
      robot_twist_.linear.y,
      robot_twist_.linear.z
    );
    
    // Calculate relative velocity in world frame
    Eigen::Vector3d rel_vel_world = object_vel_world - robot_vel_world;
    
    // Transform relative velocity to robot frame
    Eigen::Vector3d rel_vel_robot = rotation_world_to_robot * rel_vel_world;
    
    // Store in message
    state_msg.velocity.x = rel_vel_robot.x();
    state_msg.velocity.y = rel_vel_robot.y();
    state_msg.velocity.z = rel_vel_robot.z();
    
    // Set header
    state_msg.header.stamp = this->now();
    // Update frame_id to indicate it's in the robot's reference frame now
    state_msg.header.frame_id = robot_base_frame_;
    
    // Publish the message
    publisher_->publish(state_msg);
    
    // Log published state (consider using DEBUG level for less noise)
    RCLCPP_DEBUG(this->get_logger(), "Published relative state in robot frame: pos=(%.3f, %.3f, %.3f), vel=(%.3f, %.3f, %.3f)",
              state_msg.position.x, state_msg.position.y, state_msg.position.z,
              state_msg.velocity.x, state_msg.velocity.y, state_msg.velocity.z);
  }
  
  // Member variables
  std::string object_name_;
  std::string robot_base_frame_;
  rclcpp::Publisher<wx250s_bringup::msg::ObjectState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_; // For debug logging

  // Robot subscriptions - Pose is PoseStamped, Twist is TwistStamped
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr robot_twist_subscription_;

  // Object subscriptions - Pose is PoseStamped, Twist is TwistStamped
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_pose_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr object_twist_subscription_;

  // Data storage - Still store Pose and Twist internally
  geometry_msgs::msg::Pose robot_pose_;
  geometry_msgs::msg::Twist robot_twist_;
  geometry_msgs::msg::Pose object_pose_;
  geometry_msgs::msg::Twist object_twist_;
  
  // Flags to track if we have received data
  bool have_robot_pose_;
  bool have_robot_twist_;
  bool have_object_pose_;
  bool have_object_twist_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RelativeObjectStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}