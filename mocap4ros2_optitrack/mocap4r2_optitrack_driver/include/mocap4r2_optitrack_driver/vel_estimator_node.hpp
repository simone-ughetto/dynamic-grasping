#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>

/**
 * @brief Timed sample structure for storing pose data with timestamp.
 */
struct TimedSample {
    rclcpp::Time stamp;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    std::string frame;
};

/**
 * @brief Estimates linear and angular velocity from PoseStamped messages using linear regression.
 *
 * This node subscribes to PoseStamped messages and publishes TwistStamped messages.
 * It uses a sliding window with linear regression and adaptive smoothing for velocity estimation.
 */
class VelocityEstimatorNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the VelocityEstimatorNode.
     * @param opts Node options for initialization.
     */
    explicit VelocityEstimatorNode(const rclcpp::NodeOptions& opts);

private:
    // === ROS Interface ===
    /**
     * @brief Callback function for incoming PoseStamped messages.
     * @param msg Shared pointer to the received PoseStamped message.
     */
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief Timer callback for periodic twist publishing.
     */
    void publishTwist();

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_; //!< Subscription to pose data.
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_; //!< Publisher for estimated twist data.
    rclcpp::TimerBase::SharedPtr timer_; //!< Timer for periodic publishing.

    // === Data Buffer ===
    std::deque<TimedSample> buffer_; //!< Sliding window buffer for pose samples.

    // === Parameters ===
    int window_size_;        //!< Size of the sliding window.
    double in_rate_hz_;      //!< Expected input sampling rate.
    double out_rate_hz_;     //!< Output publishing rate.
    double base_alpha_;      //!< Base smoothing factor for stable conditions.

    // === Adaptive Smoothing State ===
    double current_alpha_;           //!< Current smoothing factor.
    bool has_previous_vel_;          //!< Flag indicating if previous velocity exists.
    Eigen::Vector3d prev_linear_vel_; //!< Previous linear velocity for smoothing.
    Eigen::Vector3d prev_angular_vel_; //!< Previous angular velocity for smoothing.
    
    // === Stability Tracking ===
    int unstable_count_;             //!< Counter for consecutive unstable measurements.
    int stable_count_;               //!< Counter for consecutive stable measurements.
    bool in_stabilization_mode_;     //!< Flag indicating if in stabilization mode.
};