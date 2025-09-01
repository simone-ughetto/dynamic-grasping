/**
 * @file trajectory_utils.hpp
 * @brief Utility functions for trajectory planning and manipulation
 * 
 * This file contains utility functions to support the trajectory planning
 * operations for the WidowX robot arm, including coordinate transforms,
 * trajectory sampling, and data logging.
 */

#ifndef DYNAMIC_GRASPING_CONTROLLER__TRAJECTORY_UTILS_HPP_
#define DYNAMIC_GRASPING_CONTROLLER__TRAJECTORY_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <ruckig/ruckig.hpp>
#include <vector>
#include <string>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "rclcpp/logger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <rclcpp/rclcpp.hpp> 
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp" 
#include "dynamic_grasping_controller/widowx_kinematics_library.hpp"


namespace dynamic_grasping_controller
{

/*==============================================================================
 * Coordinate System Transformations
 *============================================================================*/

/**
 * @brief Convert quaternion to Euler angles (roll, pitch, yaw)
 * @param quat Input quaternion
 * @return Vector of Euler angles in radians [roll, pitch, yaw]
 */
Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& quat);

/**
 * @brief Convert Euler angles to quaternion
 * @param roll Roll angle in radians
 * @param pitch Pitch angle in radians
 * @param yaw Yaw angle in radians
 * @return Quaternion corresponding to the Euler angles
 */
Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);

/*==============================================================================
 * Object Motion Planning
 *============================================================================*/

/**
 * @brief Compute desired orientation for grasping based on object position and velocity
 * 
 * Calculates an optimal orientation for the gripper based on the object's
 * position and velocity vectors. For moving objects, it aligns with the
 * object's direction of motion. For stationary objects, it aligns with
 * the position vector.
 *
 * @param object_pos Object position vector
 * @param object_vel Object velocity vector
 * @return Quaternion representing the desired orientation
 */
Eigen::Quaterniond computeDesiredOrientation(
    const Eigen::Vector3d& object_pos,
    const Eigen::Vector3d& object_vel);

/**
 * @brief Predict object position based on current position, velocity and time
 * 
 * Simple linear prediction of an object's future position assuming
 * constant velocity motion.
 *
 * @param object_pos Current object position
 * @param object_vel Current object velocity
 * @param time_horizon Time in the future to predict (seconds)
 * @return Predicted position vector
 */
Eigen::Vector3d predictObjectPosition(
    const Eigen::Vector3d& object_pos,
    const Eigen::Vector3d& object_vel,
    double time_horizon);

/**
 * @brief Compute adaptive duration for the first trajectory phase
 * 
 * Calculates an appropriate time duration for the approach phase of
 * the trajectory based on the object's motion and the end-effector's
 * current position. Uses a quadratic equation solver to determine
 * optimal timing.
 *
 * @param p_obj Current object position
 * @param v_obj Current object velocity
 * @param p_ee Current end-effector position
 * @param offset Z-axis offset for approach
 * @param max_vel Maximum allowed end-effector velocity
 * @param dt Time step to align with
 * @return Computed duration adjusted to be a multiple of dt
 */
double computeApproachDuration(
    const Eigen::Vector3d& p_obj,
    const Eigen::Vector3d& v_obj,
    const Eigen::Vector3d& p_ee,
    double offset,
    double max_vel,
    double dt);

/*==============================================================================
 * Trajectory Sampling and Processing
 *============================================================================*/

/**
 * @brief Sample a trajectory segment and convert to joint space
 * 
 * Samples a Ruckig trajectory at regular intervals, converting from
 * Cartesian space to joint space using inverse kinematics. The function
 * populates a ROS joint trajectory message and tracks Cartesian data
 * for visualization and logging.
 *
 * @param trajectory Ruckig trajectory to sample
 * @param phase_number Phase identifier for logging
 * @param time_offset Time offset for this segment
 * @param joint_trajectory Output joint trajectory to populate
 * @param cartesian_positions Output cartesian positions for logging
 * @param cartesian_orientations Output cartesian orientations for logging
 * @param cartesian_velocities Output cartesian velocities for logging
 * @param angular_velocities Output angular velocities for logging
 * @param dt Time step between trajectory points
 * @param joint_names Joint names in the correct order
 * @param logger ROS logger for warnings and info
 * @param add_final_point Whether to add final point for this segment
 * @param skip_first_point Whether to skip the first point (to avoid duplicates)
 * @param include_velocity Whether to include velocity in the trajectory
 * @return Total time after adding this segment
 */
double sampleTrajectorySegment(
    const ruckig::Trajectory<6>& trajectory,
    int phase_number,
    double time_offset,
    trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    std::vector<Eigen::Vector3d>& cartesian_positions,
    std::vector<Eigen::Vector3d>& cartesian_orientations,
    std::vector<Eigen::Vector3d>& cartesian_velocities,
    std::vector<Eigen::Vector3d>& angular_velocities,
    double dt,
    const std::vector<std::string>& joint_names,
    const rclcpp::Logger& logger,
    bool skip_first_point = false,
    bool include_velocity = true);  // New parameter with default=true

/*==============================================================================
 * Data Logging and Visualization
 *============================================================================*/

/**
 * @brief Save trajectory data to CSV file for visualization
 * 
 * Exports trajectory data to a CSV file for later analysis and visualization.
 * Includes joint angles, velocities, Cartesian positions, velocities, 
 * orientations, and object information.
 *
 * @param joint_trajectory Joint trajectory data
 * @param cartesian_positions Cartesian positions
 * @param cartesian_orientations Cartesian orientations
 * @param cartesian_velocities Cartesian velocities
 * @param angular_velocities Angular velocities
 * @param joint_names Joint names in the correct order
 * @param trajectory_log_path Path to save the CSV file
 * @param object_pos Current object position
 * @param object_vel Current object velocity
 * @param logger ROS logger for error reporting
 */
void saveTrajectoryToCSV(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    const std::vector<Eigen::Vector3d>& cartesian_positions,
    const std::vector<Eigen::Vector3d>& cartesian_orientations,
    const std::vector<Eigen::Vector3d>& cartesian_velocities,
    const std::vector<Eigen::Vector3d>& angular_velocities,
    const std::vector<std::string>& joint_names,
    const std::string& trajectory_log_path,
    const Eigen::Vector3d& object_pos,
    const Eigen::Vector3d& object_vel,
    const rclcpp::Logger& logger);

/*==============================================================================
 * Trajectory Velocity Calculation
 *============================================================================*/

/**
 * @brief Compute adaptive maximum velocity for a straight-line path
 * 
 * Samples a straight line between current and target positions at approximately 
 * 1 cm intervals, computes the reduced Jacobian at each point, and finds the
 * minimum of the maximum singular values to determine the maximum safe velocity.
 * 
 * @param start_pose Start pose as 6-element vector [x,y,z,roll,pitch,yaw]
 * @param target_pose Target pose as 6-element vector [x,y,z,roll,pitch,yaw]
 * @param max_default_velocity Default maximum velocity to cap the result
 * @param logger Logger for informational output
 * @return Maximum safe velocity that respects joint limits along the entire path
 */
double computeAdaptiveMaxVelocity(
    const Eigen::VectorXd& start_pose,
    const Eigen::VectorXd& target_pose,
    double max_default_velocity,
    const rclcpp::Logger& logger);

/*==============================================================================
 * Grasp prediction
 *============================================================================*/
/**
 * Predict grasping instant & position, publish it, optionally spawn a Gazebo‑Classic
 * marker.
 * @param node            – node that provides rclcpp context (used for logging & creating clients)
 * @param pub             – already‑created publisher of PoseStamped on /grasp_prediction
 * @param object_pose     – current pose of the object (planning instant)
 * @param object_twist    – current twist of the object (planning instant)
 * @param t1              – duration [s] of the *first* segment of the planned trajectory
 * @param hardware_type   – string parameter; if "gz_classic" a marker is spawned in Gazebo
 * @param urdf_path       – full path to visual_cylinder.urdf.xacro
 * @param has_spawned_marker – reference to a bool tracking if marker is already spawned
 * @return true on success, false otherwise (errors are logged)
 */
bool predictGrasp(const rclcpp::Node::SharedPtr           &node,
    const rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr &pub,
    const geometry_msgs::msg::Pose             &object_pose,
    const geometry_msgs::msg::Twist            &object_twist,
    double                                       t1,
    const std::string                           &hardware_type,
    const std::string                           &urdf_path,
    bool                                        &has_spawned_marker);

// Constants for approach calculation
constexpr double kStepLen = 0.015;         // 1 cm spacing between trajectory points
constexpr double kMaxVel  = 0.25;          // max robot velocity (m/s)
constexpr double kMaxAcc  = 0.40;          // max robot acceleration (m/s²)

/**
 * @brief Struct to hold rich trajectory prediction results
 */
struct PredResult {
    std::vector<Eigen::Vector3d> pos;        // whole trajectory path
    std::vector<double>          t;          // per-point timestamp
    int                          idx_enter;  // -1 → never enters workspace
    double                       inside_window;  // seconds object stays in workspace
};

/**
 * @brief Calculates approach time using trapezoidal/triangular motion profile
 * 
 * Computes the time required to move a specified distance considering
 * both maximum velocity and acceleration constraints.
 *
 * @param distance Distance to travel in meters
 * @return Time required in seconds
 */
double estimateEETravelTime(double distance, double max_vel = 0.25, double max_acc = 0.40);

/**
 * @brief Enhanced trajectory prediction with workspace evaluation
 * 
 * Generates a trajectory prediction for an object with constant velocity,
 * evaluates when it enters a reachable workspace, and calculates how long
 * it remains reachable.
 *
 * @param p0 Current position vector
 * @param v0 Current velocity vector 
 * @param inner_radius Inner radius of exclusion zone
 * @param analyticIK Function to check if a position is reachable
 * @param computeDesiredOrientation Function to compute orientation
 * @return PredResult containing the trajectory data and analytics
 */
PredResult objectTrajectoryEstimator(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& v0,
    double inner_radius);

} // namespace dynamic_grasping_controller

#endif // DYNAMIC_GRASPING_CONTROLLER__TRAJECTORY_UTILS_HPP_