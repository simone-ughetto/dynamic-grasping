/**
 * @file vel_estimator_node.cpp
 * @brief Implementation of a linear regression-based velocity estimator from pose data.
 *
 * This node uses a sliding window with linear regression and adaptive smoothing
 * to estimate TwistStamped from PoseStamped messages.
 */

#include "mocap4r2_optitrack_driver/vel_estimator_node.hpp"

// Standard Libraries
#include <memory>
#include <string>
#include <cmath>
#include <algorithm>
#include <deque>
#include <numeric>
#include <chrono>

// Third-Party Libraries
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// ROS Libraries (already included via header)

using std::placeholders::_1;

namespace {
// Numerical stability and filter behavior constants
constexpr double kMaxProcessingLatency = 0.1; // Maximum allowed processing latency (100 ms)

/**
 * @brief Compute angular velocity between two quaternions using SLERP-based approach
 * @param q1 First quaternion
 * @param q2 Second quaternion  
 * @param dt Time difference between quaternions
 * @return Angular velocity vector in rad/s
 */
inline Eigen::Vector3d computeAngularVelocity(const Eigen::Quaterniond& q1, 
                                              const Eigen::Quaterniond& q2, 
                                              double dt) {
    if (dt < 1e-9) {
        return Eigen::Vector3d::Zero();
    }
    
    // Ensure shortest path rotation by checking dot product
    Eigen::Quaterniond q2_corrected = q2;
    if (q1.dot(q2) < 0.0) {
        q2_corrected.coeffs() *= -1.0;  // Flip quaternion for shortest path
    }
    
    // Compute relative rotation: q_rel = q1^(-1) * q2
    Eigen::Quaterniond q_rel = q1.inverse() * q2_corrected;
    q_rel.normalize();
    
    // Convert to axis-angle representation
    Eigen::AngleAxisd aa(q_rel);
    double angle = aa.angle();
    Eigen::Vector3d axis = aa.axis();
    
    // Handle near-zero rotations
    if (std::abs(angle) < 1e-6) {
        return Eigen::Vector3d::Zero();
    }
    
    // Ensure angle is in [-π, π] range
    if (angle > M_PI) {
        angle -= 2.0 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    
    // Angular velocity = rotation_vector / dt
    return (angle * axis) / dt;
}

/**
 * @brief Compute angular velocity using SLERP-based regression
 * @param buffer Sliding window buffer of pose samples
 * @return Angular velocity vector in rad/s
 */
Eigen::Vector3d computeAngularVelocityRegression(const std::deque<TimedSample>& buffer) {
    const size_t N = buffer.size();
    if (N < 2) {
        return Eigen::Vector3d::Zero();
    }
    
    // Use multiple pairwise angular velocity estimates
    std::vector<Eigen::Vector3d> omega_samples;
    std::vector<double> weights;
    
    // Compute angular velocities for different time spans
    for (size_t i = 1; i < N; ++i) {
        double dt = (buffer[i].stamp - buffer[i-1].stamp).seconds();
        if (dt > 1e-6) {  // Valid time difference
            Eigen::Vector3d omega = computeAngularVelocity(buffer[i-1].quat, buffer[i].quat, dt);
            omega_samples.push_back(omega);
            
            // Weight by time span (longer spans are more reliable for low-frequency motion)
            weights.push_back(std::sqrt(dt));
        }
    }
    
    if (omega_samples.empty()) {
        return Eigen::Vector3d::Zero();
    }
    
    // Weighted average of angular velocity estimates
    Eigen::Vector3d omega_avg = Eigen::Vector3d::Zero();
    double total_weight = 0.0;
    
    for (size_t i = 0; i < omega_samples.size(); ++i) {
        omega_avg += weights[i] * omega_samples[i];
        total_weight += weights[i];
    }
    
    if (total_weight > 1e-9) {
        omega_avg /= total_weight;
    }
    
    return omega_avg;
}

} // namespace

/**
 * @brief Constructor: Initializes parameters and ROS publishers/subscribers.
 */
VelocityEstimatorNode::VelocityEstimatorNode(const rclcpp::NodeOptions &opts)
    : Node("velocity_estimator_node", opts),
      has_previous_vel_(false),
      unstable_count_(0),
      stable_count_(0),
      in_stabilization_mode_(false)
{
    RCLCPP_INFO(get_logger(), "Initializing Linear Regression Velocity Estimator Node...");

    // --- Parameters ---
    window_size_ = declare_parameter<int>("window_length", 120);
    in_rate_hz_ = declare_parameter<double>("sampling_rate_hz", 240.0);
    out_rate_hz_ = declare_parameter<double>("publishing_rate_hz", 30.0);
    base_alpha_ = declare_parameter<double>("base_alpha", 1);

    current_alpha_ = base_alpha_;

    if (window_size_ < 3) {
        throw std::runtime_error("window_length must be ≥ 3 for regression");
    }

    RCLCPP_INFO(get_logger(), "Parameters loaded: window_length=%d, sampling_rate=%.1f Hz, publishing_rate=%.1f Hz, base_alpha=%.2f",
                window_size_, in_rate_hz_, out_rate_hz_, base_alpha_);

    // --- ROS Interface ---
    auto sub_topic = declare_parameter("subscription_topic", std::string("/body/pose"));
    auto pub_topic = declare_parameter("publication_topic", std::string("/body/twist"));
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        sub_topic, qos, std::bind(&VelocityEstimatorNode::poseCallback, this, _1));
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(pub_topic, qos.keep_last(10));

    // Timer for periodic publishing
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / out_rate_hz_)),
        std::bind(&VelocityEstimatorNode::publishTwist, this));

    RCLCPP_INFO(get_logger(), "Subscribing to pose topic: %s", sub_topic.c_str());
    RCLCPP_INFO(get_logger(), "Publishing twist topic: %s", pub_topic.c_str());
    RCLCPP_INFO(get_logger(), "Linear Regression Velocity Estimator Node initialized successfully.");
}

/**
 * @brief Callback for incoming pose messages - stores data in sliding window.
 */
void VelocityEstimatorNode::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

    rclcpp::Time current_time = msg->header.stamp;
    rclcpp::Time now = this->now();

    // --- Latency Check ---
    double processing_latency = (now - current_time).seconds();
    if (processing_latency > kMaxProcessingLatency) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "High latency detected: %.3f s, skipping message", processing_latency);
        return; // Skip this message due to high latency
    }

    // Create timed sample
    TimedSample sample;
    sample.stamp = current_time;
    sample.pos = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    sample.quat = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                    msg->pose.orientation.y, msg->pose.orientation.z);
    sample.quat.normalize();
    sample.frame = msg->header.frame_id;

    // Add to buffer and maintain window size
    buffer_.push_back(std::move(sample));
    if (buffer_.size() > static_cast<size_t>(window_size_)) {
        buffer_.pop_front();
    }
}

/**
 * @brief Timer callback for periodic twist publishing using linear regression.
 */
void VelocityEstimatorNode::publishTwist() {
    if (buffer_.size() < 3) {
        return; // Need at least 3 points for regression
    }

    // === Prepare data for LINEAR VELOCITY regression ===
    const size_t N = buffer_.size();
    Eigen::VectorXd t(N);
    Eigen::Matrix<double, Eigen::Dynamic, 3> P(N, 3);

    for (size_t i = 0; i < N; ++i) {
        const auto& s = buffer_[i];
        t(i) = (s.stamp - buffer_.front().stamp).seconds();
        P.row(i) = s.pos;
    }

    const double t_mean = t.mean();
    Eigen::VectorXd t_ctr = t.array() - t_mean;
    const double denom = t_ctr.squaredNorm();

    if (denom < 1e-9) {
        return; // Insufficient time variation
    }

    Eigen::RowVector3d p_mean = P.colwise().mean();

    // === Compute raw LINEAR velocity via regression ===
    Eigen::Vector3d v_raw = (t_ctr.transpose() * (P.rowwise() - p_mean)).transpose() / denom;

    // === Compute raw ANGULAR velocity using SLERP-based approach ===
    Eigen::Vector3d w_raw = computeAngularVelocityRegression(buffer_);

    // === Adaptive alpha logic ===
    if (has_previous_vel_) {
        // Calculate velocity changes
        double linear_change = (v_raw - prev_linear_vel_).norm() / (prev_linear_vel_.norm() + 1e-6);
        double angular_change = (w_raw - prev_angular_vel_).norm() / (prev_angular_vel_.norm() + 1e-6);

        // Different thresholds for instability detection vs stability confirmation
        bool is_unstable = (linear_change > 0.2) || (angular_change > 0.2);  // 20% for instability
        bool is_stable = (linear_change < 0.05) && (angular_change < 0.05);   // 5% for stability

        if (in_stabilization_mode_) {
            // In stabilization mode - use alpha = 1.0 until 5 consecutive stable samples
            current_alpha_ = 1.0;
            if (is_stable) {
                stable_count_++;
                if (stable_count_ >= 5) {  // Require 5 consecutive stable samples
                    unstable_count_ = 0;
                    stable_count_ = 0;
                    in_stabilization_mode_ = false;
                    current_alpha_ = base_alpha_;
                    RCLCPP_DEBUG(get_logger(), "Velocity stabilized after 5 consecutive samples, reverting to alpha=%.2f", base_alpha_);
                }
            } else {
                stable_count_ = 0;  // Reset if not stable
            }
        } else {
            // Normal mode - check for instability
            if (is_unstable) {
                unstable_count_++;
                stable_count_ = 0;  // Reset stable counter
                if (unstable_count_ >= 3) {
                    in_stabilization_mode_ = true;
                    current_alpha_ = 1.0;
                    RCLCPP_DEBUG(get_logger(), "Velocity unstable for 3 iterations, switching to alpha=1.0");
                }
            } else {
                unstable_count_ = 0;  // Reset counter on stable measurement
            }
        }
    } else {
        current_alpha_ = 1.0;  // First measurement, no smoothing
        has_previous_vel_ = true;
    }

    // === Apply adaptive smoothing ===
    Eigen::Vector3d v, w;
    if (has_previous_vel_ && current_alpha_ < 1.0) {
        v = current_alpha_ * v_raw + (1.0 - current_alpha_) * prev_linear_vel_;
        w = current_alpha_ * w_raw + (1.0 - current_alpha_) * prev_angular_vel_;
    } else {
        v = v_raw;
        w = w_raw;
    }

    // === Store for next iteration ===
    prev_linear_vel_ = v;
    prev_angular_vel_ = w;

    // === Publish result ===
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = this->now();
    twist_msg.header.frame_id = buffer_.empty() ? "" : buffer_.back().frame;

    twist_msg.twist.linear.x = v.x();
    twist_msg.twist.linear.y = v.y();
    twist_msg.twist.linear.z = v.z();
    twist_msg.twist.angular.x = w.x();
    twist_msg.twist.angular.y = w.y();
    twist_msg.twist.angular.z = w.z();

    twist_pub_->publish(twist_msg);
}

/**
 * @brief Main function to initialize the ROS node and spin.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityEstimatorNode>(rclcpp::NodeOptions{}));
    rclcpp::shutdown();
    return 0;
}