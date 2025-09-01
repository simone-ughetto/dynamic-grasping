#pragma once

#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace dynamic_grasping_controller {

/** @brief Structure to hold joint limits for a robot joint.
 *
 *  Defines the lower and upper bounds for each joint angle.
 */
struct JointLimits {
    double lower; /**< Lower bound (in radians) */
    double upper; /**< Upper bound (in radians) */
};

/** @brief Convert degrees to radians.
 *  @param deg Angle in degrees.
 *  @return Angle in radians.
 */
inline double radians(double deg) {
    return deg * M_PI / 180.0;
}

/*==================== Utility Functions ====================*/

/** @brief Computes the homogeneous transformation matrix using DH parameters.
 *  
 *  The function builds the standard Denavit-Hartenberg transformation matrix.
 *
 *  @param a     Link length.
 *  @param alpha Link twist.
 *  @param d     Link offset.
 *  @param theta Joint angle.
 *  @return 4x4 homogeneous transformation matrix.
 */
Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta);

/** @brief Checks if a given joint solution is within the defined joint limits.
 *  
 *  @param sol A vector of joint angles (radians).
 *  @return true if all joint angles are within their limits, false otherwise.
 */
bool withinJointLimits(const Eigen::VectorXd& sol);

/** @brief Calculates the wrist center position from an end-effector pose.
 *
 *  The wrist center is located at a distance d6 from the end-effector position 
 *  in the opposite direction of the end-effector's approach vector.
 *
 *  @param ee_pose A 6-element vector [x, y, z, roll, pitch, yaw] or 
 *                 a 7-element vector [x, y, z, qx, qy, qz, qw].
 *  @return A 3-element vector representing the wrist center position [x, y, z].
 */
Eigen::Vector3d calculateWristCenter(const Eigen::VectorXd& ee_pose);

/** @brief Overload of calculateWristCenter for std::vector input.
 *
 *  @param ee_pose A 6-element vector [x, y, z, roll, pitch, yaw] or 
 *                 a 7-element vector [x, y, z, qx, qy, qz, qw].
 *  @return A 3-element vector representing the wrist center position [x, y, z].
 */
Eigen::Vector3d calculateWristCenter(const std::vector<double>& ee_pose);

/** @brief Efficiently checks if a target position is within the robot's workspace
 *
 *  Unlike analyticIK, this function only performs calculations needed to determine
 *  reachability and doesn't output any warning messages. This makes it more suitable
 *  for high-frequency reachability checks like in trajectory prediction.
 *
 *  @param target A 3-, 6-, or 7-element vector describing the desired target
 *                (position, or position+orientation)
 *  @return true if the target is within the workspace, false otherwise
 */
bool isInWorkspace(const Eigen::VectorXd& target);

/** @brief Overload of isInWorkspace for std::vector input.
 *
 *  @param target A 3-, 6-, or 7-element vector describing the target
 *  @return true if the target is reachable, false otherwise
 */
bool isInWorkspace(const std::vector<double>& target);

/*==================== Forward Kinematics ====================*/

/** @brief Computes the forward kinematics for the WidowX 250 6DOF robot.
 *
 *  Given a 6-element joint angle vector (in radians), this function
 *  computes the end-effector pose expressed as a 7-element vector.
 *
 *  @param joint_angles A 6-element vector [θ₁, θ₂, θ₃, θ₄, θ₅, θ₆].
 *  @return A 7-element pose vector [x, y, z, qx, qy, qz, qw].
 */
Eigen::VectorXd forwardKinematics(const Eigen::VectorXd& joint_angles);

/** @brief Overload of forwardKinematics for std::vector input.
 *
 *  @param joint_angles 6-element vector (in radians).
 *  @return A 7-element pose vector [x, y, z, qx, qy, qz, qw].
 */
Eigen::VectorXd forwardKinematics(const std::vector<double>& joint_angles);

/*==================== Inverse Kinematics ====================*/

/** @brief Computes the analytic inverse kinematics for the WidowX 250S robot.
 *
 *  The function accepts either:
 *  - a 3-element target vector ([x, y, z]) for the wrist center position, solving only for joints 1-3
 *  - a 6-element target vector ([x, y, z, roll, pitch, yaw]) for the end-effector pose
 *  - a 7-element target vector ([x, y, z, qx, qy, qz, qw]) for the end-effector pose
 *
 *  @param target 3-, 6-, or 7-element vector describing the desired target.
 *  @return A vector of joint solutions. For 3-element input, each solution has 3 elements.
 *          For 6- or 7-element input, each solution has 6 elements.
 */
Eigen::VectorXd analyticIK(const Eigen::VectorXd& target);

/** @brief Overload of analyticIK for std::vector input.
 *
 *  @param target 6- or 7-element target pose vector.
 *  @return A vector of joint solutions.
 */
Eigen::VectorXd analyticIK(const std::vector<double>& target);

/*==================== Geometric Jacobian ====================*/

/** @brief Computes the geometric Jacobian of the WidowX 250S 6DOF robot arm.
 *
 *  The Jacobian matrix is 6x6 where the top three rows represent the linear velocity,
 *  and the bottom three rows represent the angular velocity.
 *
 *  @param joint_angles A 6-element joint angle vector (in radians).
 *  @return A 6x6 Jacobian matrix.
 */
Eigen::MatrixXd geometricJacobian(const Eigen::VectorXd& joint_angles);

/** @brief Overload of geometricJacobian for std::vector input.
 *
 *  @param joint_angles A 6-element joint angle vector (in radians).
 *  @return A 6x6 Jacobian matrix.
 */
Eigen::MatrixXd geometricJacobian(const std::vector<double>& joint_angles);

/*==================== Analytic Jacobian ====================*/

/** @brief Computes the analytic Jacobian mapping joint velocities to end-effector velocities.
 *
 *  Uses the geometric Jacobian and converts the angular component from quaternions to ZYX Euler rates.
 *
 *  @param joint_angles A 6-element joint angle vector (in radians).
 *  @return A 6x6 analytic Jacobian matrix.
 */
Eigen::MatrixXd analyticJacobian(const Eigen::VectorXd& joint_angles);

/** @brief Overload of analyticJacobian for std::vector input.
 *
 *  @param joint_angles A 6-element joint angle vector (in radians).
 *  @return A 6x6 analytic Jacobian matrix.
 */
Eigen::MatrixXd analyticJacobian(const std::vector<double>& joint_angles);

/*==================== Jacobian Inversion ====================*/

/** @brief Computes the inverse (or pseudoinverse) of a Jacobian matrix.
 *
 *  Applies direct inversion if the smallest singular value exceeds the threshold,
 *  or uses a damped least squares method if near singular.
 *
 *  @param J         The Jacobian matrix to invert.
 *  @param damping   Damping factor for the DLS inversion (default: 0.01).
 *  @param threshold Singular value threshold (default: 1e-6).
 *  @return The inverted (or pseudoinverse) Jacobian matrix.
 */
Eigen::MatrixXd invertJacobian(const Eigen::MatrixXd& J, double damping = 0.01, double threshold = 1e-6);

/*==================== Wrist Center Jacobian ====================*/

/** @brief Computes the Jacobian for the first 3 joints that affect the wrist center position.
 *
 *  This reduced Jacobian relates the velocities of the first 3 joints to the
 *  linear velocity of the wrist center (position right after joint 3).
 *
 *  @param joint_angles A 6-element joint angle vector (in radians).
 *  @return A pair containing the 3x3 Jacobian matrix and its largest singular value.
 */
std::pair<Eigen::MatrixXd, double> scaledReducedJacobian(const Eigen::VectorXd& joint_angles);

/** @brief Overload of wristCenterJacobian for std::vector input.
 *
 *  @param joint_angles A 6-element joint angle vector (in radians).
 *  @return A pair containing the 3x3 Jacobian matrix and its largest singular value.
 */
std::pair<Eigen::MatrixXd, double> scaledReducedJacobian(const std::vector<double>& joint_angles);

/*==================== Cartesian to Joint Velocity ====================*/

/** @brief Converts a Cartesian velocity twist to joint velocities.
 *  
 *  Uses an efficient damped least squares approach when near singularity.
 *
 *  @param J     The 6x6 Jacobian matrix.
 *  @param twist A 6-element vector representing Cartesian velocity [vx, vy, vz, wx, wy, wz].
 *  @return A 6-element joint velocity vector.
 */
Eigen::VectorXd cartesianToJointVel(const Eigen::MatrixXd& J, const Eigen::VectorXd& twist);

/** @brief Overload of cartesianToJointVel for std::vector input.
 *
 *  @param J     The 6x6 Jacobian matrix as a vector of vectors.
 *  @param twist A 6-element vector representing Cartesian velocity [vx, vy, vz, wx, wy, wz].
 *  @return A 6-element joint velocity vector.
 */
std::vector<double> cartesianToJointVel(const std::vector<std::vector<double>>& J, 
                                       const std::vector<double>& twist);

} // namespace dynamic_grasping_controller
