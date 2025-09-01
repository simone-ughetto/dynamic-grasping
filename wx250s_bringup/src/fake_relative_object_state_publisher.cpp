/**
 * @file fake_relative_object_state_publisher.cpp
 * @brief A node to calculate and publish relative position and velocity between robot and object
 *        for simulation using differentiation with linear regression
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "wx250s_bringup/msg/object_state.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include <string>
#include <cmath>
#include <deque>
#include <vector>

// Structure to hold timestamped data for regression
struct TimestampedData {
  double time;
  geometry_msgs::msg::Point position;
  geometry_msgs::msg::Quaternion orientation;
};

class LinearRegressionEstimator {
private:
  std::deque<TimestampedData> buffer_;
  size_t max_size_;
  double min_window_time_;  // Minimum time span for regression

public:
  LinearRegressionEstimator(size_t max_size = 15, double min_window_time = 0.3) 
    : max_size_(max_size), min_window_time_(min_window_time) {}

  void addSample(double time, const geometry_msgs::msg::Point& pos, const geometry_msgs::msg::Quaternion& quat) {
    TimestampedData data;
    data.time = time;
    data.position = pos;
    data.orientation = quat;
    
    buffer_.push_back(data);
    
    // Remove old samples
    while (buffer_.size() > max_size_) {
      buffer_.pop_front();
    }
    
    // Also remove samples outside time window
    double current_time = time;
    while (!buffer_.empty() && (current_time - buffer_.front().time) > min_window_time_ * 3.0) {
      buffer_.pop_front();
    }
  }

  geometry_msgs::msg::Vector3 getLinearVelocity() {
    geometry_msgs::msg::Vector3 vel;
    vel.x = vel.y = vel.z = 0.0;
    
    if (buffer_.size() < 3) return vel;  // Need at least 3 points
    
    // Check if we have sufficient time span
    double time_span = buffer_.back().time - buffer_.front().time;
    if (time_span < min_window_time_) return vel;
    
    // Linear regression: v = (sum(t*p) - sum(t)*sum(p)/n) / (sum(t²) - sum(t)²/n)
    size_t n = buffer_.size();
    double sum_t = 0.0, sum_t2 = 0.0;
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    double sum_tx = 0.0, sum_ty = 0.0, sum_tz = 0.0;
    
    for (const auto& sample : buffer_) {
      double t = sample.time;
      sum_t += t;
      sum_t2 += t * t;
      sum_x += sample.position.x;
      sum_y += sample.position.y;
      sum_z += sample.position.z;
      sum_tx += t * sample.position.x;
      sum_ty += t * sample.position.y;
      sum_tz += t * sample.position.z;
    }
    
    double denominator = n * sum_t2 - sum_t * sum_t;
    if (std::abs(denominator) < 1e-10) return vel;  // Avoid division by zero
    
    vel.x = (n * sum_tx - sum_t * sum_x) / denominator;
    vel.y = (n * sum_ty - sum_t * sum_y) / denominator;
    vel.z = (n * sum_tz - sum_t * sum_z) / denominator;
    
    return vel;
  }

  // Alternative implementation using median filtering:

  geometry_msgs::msg::Vector3 getAngularVelocity() {
    geometry_msgs::msg::Vector3 ang_vel;
    ang_vel.x = ang_vel.y = ang_vel.z = 0.0;
    
    if (buffer_.size() < 5) return ang_vel;  // Need more samples for stability
    
    double time_span = buffer_.back().time - buffer_.front().time;
    if (time_span < min_window_time_) return ang_vel;
    
    // Compute angular velocity using multiple time differences to reduce noise
    std::vector<geometry_msgs::msg::Vector3> velocity_estimates;
    
    // Use different time intervals for robustness
    std::vector<int> intervals = {1, 2, 3};  // Skip 1, 2, or 3 samples
    
    for (int interval : intervals) {
      for (size_t i = interval; i < buffer_.size(); ++i) {
        const auto& prev_sample = buffer_[i - interval];
        const auto& curr_sample = buffer_[i];
        
        double dt = curr_sample.time - prev_sample.time;
        if (dt <= 0.0) continue;
        
        tf2::Quaternion q_prev(prev_sample.orientation.x, prev_sample.orientation.y,
                              prev_sample.orientation.z, prev_sample.orientation.w);
        tf2::Quaternion q_curr(curr_sample.orientation.x, curr_sample.orientation.y,
                              curr_sample.orientation.z, curr_sample.orientation.w);
        
        q_prev.normalize();
        q_curr.normalize();
        
        // Ensure shortest path
        if (q_curr.dot(q_prev) < 0.0) {
          q_prev = tf2::Quaternion(-q_prev.x(), -q_prev.y(), -q_prev.z(), -q_prev.w());
        }
        
        // Compute angular velocity using logarithmic map
        tf2::Quaternion q_rel = q_curr * q_prev.inverse();
        q_rel.normalize();
        
        geometry_msgs::msg::Vector3 w_est;
        
        // Use log map for more stable computation
        double w_component = std::abs(q_rel.w());
        if (w_component > 0.999) {
          // Very small rotation
          w_est.x = 2.0 * q_rel.x() / dt;
          w_est.y = 2.0 * q_rel.y() / dt;
          w_est.z = 2.0 * q_rel.z() / dt;
        } else {
          double theta = 2.0 * std::acos(w_component);
          tf2::Vector3 axis(q_rel.x(), q_rel.y(), q_rel.z());
          double axis_norm = axis.length();
          
          if (axis_norm > 1e-8) {
            if (q_rel.w() < 0.0) theta = -theta;
            double scale = theta / (axis_norm * dt);
            w_est.x = axis.x() * scale;
            w_est.y = axis.y() * scale;
            w_est.z = axis.z() * scale;
          } else {
            w_est.x = w_est.y = w_est.z = 0.0;
          }
        }
        
        velocity_estimates.push_back(w_est);
      }
    }
    
    if (velocity_estimates.empty()) return ang_vel;
    
    // Compute median of estimates to remove outliers
    auto computeMedian = [](std::vector<double>& values) -> double {
      if (values.empty()) return 0.0;
      std::sort(values.begin(), values.end());
      size_t n = values.size();
      if (n % 2 == 0) {
        return (values[n/2 - 1] + values[n/2]) / 2.0;
      } else {
        return values[n/2];
      }
    };
    
    std::vector<double> x_vals, y_vals, z_vals;
    for (const auto& v : velocity_estimates) {
      x_vals.push_back(v.x);
      y_vals.push_back(v.y);
      z_vals.push_back(v.z);
    }
    
    ang_vel.x = computeMedian(x_vals);
    ang_vel.y = computeMedian(y_vals);
    ang_vel.z = computeMedian(z_vals);
    
    // Final noise reduction
    const double noise_threshold = 0.01;  // rad/s
    if (std::abs(ang_vel.x) < noise_threshold) ang_vel.x = 0.0;
    if (std::abs(ang_vel.y) < noise_threshold) ang_vel.y = 0.0;
    if (std::abs(ang_vel.z) < noise_threshold) ang_vel.z = 0.0;
    
    return ang_vel;
  }

  bool hasEnoughData() const {
    if (buffer_.size() < 3) return false;
    double time_span = buffer_.back().time - buffer_.front().time;
    return time_span >= min_window_time_;
  }
};

class FakeRelativeObjectStatePublisher : public rclcpp::Node
{
public:
  FakeRelativeObjectStatePublisher()
  : Node("fake_relative_object_state_publisher"),
    has_robot_data_(false),
    window_size_(15),
    min_window_time_(0.2)
  {
    // Declare parameters
    this->declare_parameter("object_name", "ball");
    this->declare_parameter("robot_base_frame", "wx250s/base_link");
    this->declare_parameter("window_size", static_cast<int>(window_size_));
    this->declare_parameter("min_window_time", min_window_time_);
    
    // Get parameters
    object_name_ = this->get_parameter("object_name").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    window_size_ = this->get_parameter("window_size").as_int();
    min_window_time_ = this->get_parameter("min_window_time").as_double();
    
    // Create estimators
    object_estimator_ = std::make_unique<LinearRegressionEstimator>(window_size_, min_window_time_);
    robot_estimator_ = std::make_unique<LinearRegressionEstimator>(window_size_, min_window_time_);
    
    // Create publisher for object state
    publisher_ = this->create_publisher<wx250s_bringup::msg::ObjectState>("/object_state", 10);
    
    // Set up TF2
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // Subscribe to object pose
    object_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + object_name_ + "/pose", 10, 
      std::bind(&FakeRelativeObjectStatePublisher::objectOdomCallback, this, std::placeholders::_1));
    
    // Subscribe to robot base link pose
    robot_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/" + robot_base_frame_ + "_pose", 10,
      std::bind(&FakeRelativeObjectStatePublisher::robotOdomCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Fake relative object state publisher initialized with linear regression");
    RCLCPP_INFO(this->get_logger(), "Tracking object: %s", object_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using robot base frame: %s", robot_base_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Window size: %zu, Min window time: %.2f s", window_size_, min_window_time_);
  }

private:
  void robotOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    // Add sample to robot estimator
    robot_estimator_->addSample(current_time, msg->pose.pose.position, msg->pose.pose.orientation);
    
    has_robot_data_ = true;
  }

  void objectOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!has_robot_data_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "Waiting for robot position data...");
      return;
    }

    double current_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    
    // Get the object pose from the Odometry message
    geometry_msgs::msg::Pose object_pose = msg->pose.pose;
    
    // Create object state message
    auto state_msg = wx250s_bringup::msg::ObjectState();
    
    try {
      // Create message with world-frame data
      geometry_msgs::msg::PoseStamped world_pose;
      world_pose.header = msg->header;
      world_pose.pose = object_pose;
      
      // Look up the transform from world to robot base
      geometry_msgs::msg::TransformStamped transform = 
        tf_buffer_->lookupTransform(robot_base_frame_, msg->header.frame_id, 
                                    msg->header.stamp);
      
      // Transform the pose
      geometry_msgs::msg::PoseStamped base_pose;
      tf2::doTransform(world_pose, base_pose, transform);
      
      // Add sample to object estimator (in robot base frame)
      object_estimator_->addSample(current_time, base_pose.pose.position, base_pose.pose.orientation);
      
      // Set position (relative to robot base frame)
      state_msg.position = base_pose.pose.position;
      
      // Get velocities from regression estimators
      if (object_estimator_->hasEnoughData()) {
        auto obj_linear_vel = object_estimator_->getLinearVelocity();
        auto obj_angular_vel = object_estimator_->getAngularVelocity();
        
        // Object velocities are already in robot base frame due to TF transform
        state_msg.velocity = obj_linear_vel;
        state_msg.angular_velocity = obj_angular_vel;
      } else {
        // Not enough data yet - set zero velocity
        state_msg.velocity.x = state_msg.velocity.y = state_msg.velocity.z = 0.0;
        state_msg.angular_velocity.x = state_msg.angular_velocity.y = state_msg.angular_velocity.z = 0.0;
      }
      
      // Set header
      state_msg.header.stamp = this->now();
      state_msg.header.frame_id = robot_base_frame_;
      
      // Publish the message
      publisher_->publish(state_msg);
      
    } catch (const tf2::TransformException &ex) {
      // Fallback: compute in world frame then subtract robot velocity
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Could not transform from %s to %s: %s, using fallback method", 
                       msg->header.frame_id.c_str(), robot_base_frame_.c_str(), ex.what());
      
      // Add sample to object estimator (in world frame)
      object_estimator_->addSample(current_time, object_pose.position, object_pose.orientation);
      
      // Compute relative position directly
      if (robot_estimator_->hasEnoughData()) {
        // Get latest robot position (this is approximate but better than nothing)
        auto robot_linear_vel = robot_estimator_->getLinearVelocity();
        auto robot_angular_vel = robot_estimator_->getAngularVelocity();
        
        // Simple relative position computation (not frame-corrected)
        // For better accuracy, store latest robot position
        state_msg.position.x = object_pose.position.x;  // Placeholder
        state_msg.position.y = object_pose.position.y;  // Placeholder
        state_msg.position.z = object_pose.position.z;  // Placeholder
        
        if (object_estimator_->hasEnoughData()) {
          auto obj_linear_vel = object_estimator_->getLinearVelocity();
          auto obj_angular_vel = object_estimator_->getAngularVelocity();
          
          // Compute relative velocities
          state_msg.velocity.x = obj_linear_vel.x - robot_linear_vel.x;
          state_msg.velocity.y = obj_linear_vel.y - robot_linear_vel.y;
          state_msg.velocity.z = obj_linear_vel.z - robot_linear_vel.z;
          
          state_msg.angular_velocity.x = obj_angular_vel.x - robot_angular_vel.x;
          state_msg.angular_velocity.y = obj_angular_vel.y - robot_angular_vel.y;
          state_msg.angular_velocity.z = obj_angular_vel.z - robot_angular_vel.z;
        } else {
          state_msg.velocity.x = state_msg.velocity.y = state_msg.velocity.z = 0.0;
          state_msg.angular_velocity.x = state_msg.angular_velocity.y = state_msg.angular_velocity.z = 0.0;
        }
      } else {
        // No robot data available
        state_msg.position = object_pose.position;
        state_msg.velocity.x = state_msg.velocity.y = state_msg.velocity.z = 0.0;
        state_msg.angular_velocity.x = state_msg.angular_velocity.y = state_msg.angular_velocity.z = 0.0;
      }
      
      // Set header
      state_msg.header.stamp = this->now();
      state_msg.header.frame_id = "world";
      
      // Publish the message
      publisher_->publish(state_msg);
    }
  }

  // Member variables
  std::string object_name_;
  std::string robot_base_frame_;
  rclcpp::Publisher<wx250s_bringup::msg::ObjectState>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr object_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_subscription_;
  
  // TF2 objects
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // Regression estimators
  std::unique_ptr<LinearRegressionEstimator> object_estimator_;
  std::unique_ptr<LinearRegressionEstimator> robot_estimator_;
  
  bool has_robot_data_;
  size_t window_size_;
  double min_window_time_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FakeRelativeObjectStatePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}