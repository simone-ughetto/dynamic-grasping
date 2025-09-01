#include "dynamic_grasping_controller/widowx_kinematics_library.hpp"

namespace dynamic_grasping_controller {

//======================================================================
//  Conversion Utility
//======================================================================
inline Eigen::VectorXd toEigen(const std::vector<double>& vec) {
    return Eigen::Map<const Eigen::VectorXd>(vec.data(), vec.size());
}

//======================================================================
//  DH Parameters and Global Constants
//======================================================================
namespace {
    // Tiny offset to avoid numerical issues at joint limits
    const double pi_offset = 0.00001;

    // Maximum joint velocity (rad/s)
    const double max_joint_velocity = 0.95 * M_PI;

    // ----- Link 1 (base to shoulder) -----
    const double a1 = 0.0;                  // No X offset
    const double alpha1 = -M_PI / 2;        // 90° rotation around X (Z1 points along -Y0)
    const double d1 = 0.11025;              // Height of the first joint (base height)

    // ----- Link 2 (shoulder to elbow) -----
    const double a2 = sqrt(0.25 * 0.25 + 0.05 * 0.05);  // Hypotenuse of the arm segment
    const double alpha2 = 0.0;              // No rotation around X
    const double d2 = 0.0;                  // No Z offset
    const double theta2_offset = atan2(0.25, 0.05);     // Physical offset from DH convention

    // ----- Link 3 (elbow to forearm) -----
    const double a3 = 0.0;                  // No X offset
    const double alpha3 = -M_PI / 2;        // 90° rotation around X
    const double d3 = 0.0;                  // No Z offset
    const double theta3_offset = atan2(0.05, 0.25);     // Physical offset from DH convention

    // ----- Link 4 (forearm roll) -----
    const double a4 = 0.0;                  // No X offset
    const double alpha4 = M_PI / 2;         // -90° rotation around X
    const double d4 = 0.25;                 // Length of the forearm

    // ----- Link 5 (wrist angle) -----
    const double a5 = 0.0;                  // No X offset
    const double alpha5 = -M_PI / 2;        // 90° rotation around X
    const double d5 = 0.0;                  // No Z offset

    // ----- Link 6 (wrist rotate to end-effector) -----
    const double a6 = 0.0;                  // No X offset
    const double alpha6 = 0.0;              // No rotation around X
    const double d6 = 0.158575;             // Distance to end effector (gripper)

    // ----- End-Effector Orientation Transformation -----
    // This rotates the wrist coordinate system to align with the end-effector frame.
    // The columns represent the mapping of the wrist frame axes to the tool frame.
    const Eigen::Matrix3d R_e = (Eigen::Matrix3d() <<
        0,  0,  1,    // X_wrist maps to Z_tool
        0, -1,  0,    // Y_wrist maps to -Y_tool
        1,  0,  0     // Z_wrist maps to X_tool
    ).finished();

    // ----- Joint Limits (based on wx250s.urdf.xacro) -----
    const std::vector<JointLimits> joint_limits = {
        { -M_PI + pi_offset,    M_PI - pi_offset },    // Waist (joint 1)
        { radians(-108),        radians(114) },        // Shoulder (joint 2)
        { radians(-123),        radians(92) },         // Elbow (joint 3)
        { -M_PI + pi_offset,    M_PI - pi_offset },    // Forearm Roll (joint 4)
        { radians(-100),        radians(123) },        // Wrist Angle (joint 5)
        { -M_PI + pi_offset,    M_PI - pi_offset }     // Wrist Rotate (joint 6)
    };
}

//======================================================================
//  Utility Function: DH Transformation
//======================================================================
Eigen::Matrix4d dhTransform(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T;
    double ct = cos(theta), st = sin(theta);
    double ca = cos(alpha), sa = sin(alpha);

    T << ct, -st * ca, st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
         0,   sa,       ca,      d,
         0,   0,        0,       1;
    return T;
}

//======================================================================
//  Joint Limits Check
//======================================================================
bool withinJointLimits(const Eigen::VectorXd& sol) {
    for (int i = 0; i < 6; ++i) {
        if (sol(i) < joint_limits[i].lower || sol(i) > joint_limits[i].upper)
            return false;
    }
    return true;
}

//======================================================================
//  Forward Kinematics
//======================================================================
/**
 * Computes the forward kinematics for the WidowX 250S 6DOF robot arm.
 * 
 * Input: 6-element vector of joint angles (radians).
 * Output: 7-element pose vector [x, y, z, qx, qy, qz, qw].
 */
Eigen::VectorXd forwardKinematics(const Eigen::VectorXd& joint_angles) {
    if (joint_angles.size() != 6) {
        std::cerr << "Error: Expected 6 joint angles." << std::endl;
        exit(EXIT_FAILURE);
    }
    
    // Convert physical joint angles to DH convention angles.
    double theta1 = joint_angles(0);                   // Waist (no offset)
    double theta2 = joint_angles(1) - theta2_offset;     // Shoulder with offset
    double theta3 = joint_angles(2) - theta3_offset;     // Elbow with offset
    double theta4 = joint_angles(3);                     // Forearm roll (no offset)
    double theta5 = joint_angles(4);                     // Wrist angle (no offset)
    double theta6 = joint_angles(5);                     // Wrist rotation (no offset)
    
    // Compute transformation matrices for each joint.
    Eigen::Matrix4d T1 = dhTransform(a1, alpha1, d1, theta1);
    Eigen::Matrix4d T2 = dhTransform(a2, alpha2, d2, theta2);
    Eigen::Matrix4d T3 = dhTransform(a3, alpha3, d3, theta3);
    Eigen::Matrix4d T4 = dhTransform(a4, alpha4, d4, theta4);
    Eigen::Matrix4d T5 = dhTransform(a5, alpha5, d5, theta5);
    Eigen::Matrix4d T6 = dhTransform(a6, alpha6, d6, theta6);
    
    // Additional end-effector transformation.
    Eigen::Matrix4d T_e = Eigen::Matrix4d::Identity();
    T_e.block<3,3>(0,0) = R_e;
    
    // Compose the complete transformation.
    Eigen::Matrix4d T_final = T1 * T2 * T3 * T4 * T5 * T6 * T_e;
    
    // Extract position and orientation.
    Eigen::Vector3d position = T_final.block<3,1>(0,3);
    Eigen::Matrix3d rotation = T_final.block<3,3>(0,0);
    Eigen::Quaterniond quaternion(rotation);
    
    // Assemble the pose vector.
    Eigen::VectorXd pose(7);
    pose.segment<3>(0) = position;
    pose(3) = quaternion.x();
    pose(4) = quaternion.y();
    pose(5) = quaternion.z();
    pose(6) = quaternion.w();
    
    return pose;
}

// Overload for std::vector input.
Eigen::VectorXd forwardKinematics(const std::vector<double>& joint_angles) {
    return forwardKinematics(toEigen(joint_angles));
}

//======================================================================
//  Calculate Wrist Center Position
//======================================================================
/**
 * Calculates the wrist center position from an end-effector pose.
 * Works with both 6-element poses (position + RPY) and 7-element poses (position + quaternion).
 */
Eigen::Vector3d calculateWristCenter(const Eigen::VectorXd& ee_pose) {
    Eigen::Vector3d position = ee_pose.segment<3>(0);
    Eigen::Matrix3d rotation;
    
    if (ee_pose.size() == 6) {
        // Convert RPY to rotation matrix
        double roll = ee_pose(3), pitch = ee_pose(4), yaw = ee_pose(5);
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        rotation = yawAngle * pitchAngle * rollAngle;
    } else if (ee_pose.size() == 7) {
        // Extract quaternion components
        Eigen::Quaterniond q(ee_pose(6), ee_pose(3), ee_pose(4), ee_pose(5));
        rotation = q.toRotationMatrix();
    } else {
        std::cerr << "Error: Invalid pose vector size for wrist center calculation." << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    // The approach vector is the x-axis of the end-effector frame
    Eigen::Vector3d approach_vector = rotation.col(0);
    
    // The wrist center is located at distance d6 from the end-effector position
    // in the opposite direction of the approach vector
    return position - d6 * approach_vector;
}

// Overload for std::vector input
Eigen::Vector3d calculateWristCenter(const std::vector<double>& ee_pose) {
    if (ee_pose.size() != 6 && ee_pose.size() != 7) {
        std::cerr << "Error: Expected 6 or 7 elements for end-effector pose." << std::endl;
        return Eigen::Vector3d::Zero();
    }
    
    Eigen::VectorXd pose_eigen(ee_pose.size());
    for (size_t i = 0; i < ee_pose.size(); ++i) {
        pose_eigen(i) = ee_pose[i];
    }
    
    return calculateWristCenter(pose_eigen);
}

//======================================================================
//  Workspace Check
//======================================================================
/**
 * Efficiently checks if a target is within the robot's reachable workspace.
 * Avoids full IK calculations and warning messages when only a boolean check is needed.
 */
bool isInWorkspace(const Eigen::VectorXd& target) {
    // --- Extract or compute wrist center position ---
    Eigen::Vector3d p_wc;
    
    if (target.size() == 3) {
        p_wc = target;  // Target is directly the wrist center
    } 
    else if (target.size() == 6 || target.size() == 7) {
        p_wc = calculateWristCenter(target);
    }
    else {
        return false;  // Invalid target size
    }
    
    // --- Calculate reach parameters ---
    // Calculate cylindrical coordinates of wrist center
    double r_wc = sqrt(p_wc(0)*p_wc(0) + p_wc(1)*p_wc(1));
    double z_wc = p_wc(2) - d1;
    
    // Calculate link parameters
    double L1 = a2, L2 = d4;
    double R = sqrt(r_wc * r_wc + z_wc * z_wc);
    
    // Calculate D parameter (cosine law)
    double D = (R * R - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    
    // Check if the target is within reach
    return (fabs(D) <= 1.0);
}

// Overload for std::vector input
bool isInWorkspace(const std::vector<double>& target) {
    if (target.size() != 3 && target.size() != 6 && target.size() != 7) {
        return false;  // Invalid target size
    }
    
    Eigen::VectorXd target_eigen(target.size());
    for (size_t i = 0; i < target.size(); ++i) {
        target_eigen(i) = target[i];
    }
    
    return isInWorkspace(target_eigen);
}

//======================================================================
//  Analytic Inverse Kinematics
//======================================================================
/**
 * Computes the inverse kinematics for the WidowX 250S using an analytic approach.
 * Supports both 6-parameter (position + RPY) and 7-parameter (position + quaternion) targets.
 */
Eigen::VectorXd analyticIK(const Eigen::VectorXd& target) {
    // Instead of creating a vector of solutions
    // std::vector<Eigen::VectorXd> solutions;
    
    // --- Step 1: Interpret the Input Target Pose ---
    Eigen::Matrix3d R_des;  // Desired rotation matrix.
    Eigen::Matrix3d R06;    // Initialize R06 at the top level
    Eigen::Vector3d p;      // Desired position.
    Eigen::Vector3d p_wc;   // Initialize wrist center position at the top level
    
    if (target.size() == 6) {
        p = target.segment<3>(0);
        double roll  = target(3);
        double pitch = target(4);
        double yaw   = target(5);
        Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
        R_des = yawAngle * pitchAngle * rollAngle;
        
        // --- Step 2: Adjust End-Effector Orientation ---
        R06 = R_des * R_e.transpose();
        
        // --- Step 3: Compute the Wrist Center Position ---
        p_wc = calculateWristCenter(target);
    } else if (target.size() == 7) {
        p = target.segment<3>(0);
        // Eigen's Quaterniond: (w, x, y, z)
        Eigen::Quaterniond q(target(6), target(3), target(4), target(5));
        R_des = q.toRotationMatrix();
        
        // --- Step 2: Adjust End-Effector Orientation ---
        R06 = R_des * R_e.transpose();
        
        // --- Step 3: Compute the Wrist Center Position ---
        p_wc = calculateWristCenter(target);
    } else {
        std::cerr << "Target vector must be of size 3 (wrist center), 6 (RPY) or 7 (quaternion)." << std::endl;
        return Eigen::VectorXd(0);  // Return empty vector to indicate error
    }
    
    // Calculate cylindrical coordinates of wrist center
    double r_wc = sqrt(p_wc(0)*p_wc(0) + p_wc(1)*p_wc(1));
    
    // Additional computations to handle singularity.
    Eigen::Vector3d x_prime;
    bool negative_projection = false;
    x_prime = Eigen::Vector3d(p(0), p(1), 0.0);
    Eigen::Vector3d wc_horizontal(p_wc(0), p_wc(1), 0.0);
    double x_prime_norm = x_prime.norm();
    if (x_prime_norm > 1e-6) {
        x_prime /= x_prime_norm;
        double projection = wc_horizontal.dot(x_prime);
        if (projection < 0) {
            r_wc = -r_wc;
            negative_projection = true;
        }
    }

    // Solve for first joint angle (theta1)
    const double epsilon = 1e-5;
    const double damping_factor = 0.01;
    double theta1, sin_theta1;

    if (fabs(r_wc) < epsilon) {
        std::cerr << "Near shoulder singularity (r_wc = " << r_wc 
                  << "). Applying damped solution." << std::endl;
        double x_component = p_wc(0);
        double y_component = p_wc(1);
        if (fabs(x_component) < 1e-6 && fabs(y_component) < 1e-6) {
            x_component = 1.0;
            y_component = 0.0;
        }
        double magnitude = sqrt(x_component*x_component + y_component*y_component);
        x_component /= magnitude;
        y_component /= magnitude;
        double damped_x = p_wc(0) + x_component * damping_factor;
        double damped_y = p_wc(1) + y_component * damping_factor;
        theta1 = atan2(damped_y, damped_x);
    } else {
        sin_theta1 = sin(atan2(p_wc(1), p_wc(0)));
        if (fabs(sin_theta1) > 0.1) {
            theta1 = (negative_projection ? atan2(-p_wc(1), -p_wc(0)) : atan2(p_wc(1), p_wc(0)));
        } else {
            theta1 = sin_theta1;
        }
    }

    // Solve for joints 2 and 3
    double z_wc = p_wc(2) - d1;
    double L1 = a2, L2 = d4;
    double R = sqrt(r_wc * r_wc + z_wc * z_wc);
    double D = (R * R - L1 * L1 - L2 * L2) / (2 * L1 * L2);
    if (fabs(D) > 1.0) {
        std::cerr << "Target position is out of reach (D = " << D << ")." << std::endl;
        return Eigen::VectorXd(0);  // Return empty vector to indicate error
    }
    
    double theta3 = acos(D) - theta2_offset;
    double beta = atan2(z_wc, r_wc);
    double gamma = atan2(L2 * sin(acos(D)), L1 + L2 * D);
    double theta2 = theta2_offset - (beta + gamma);
    
    // --- Step 4: Compute the First Three Transformations ---
    Eigen::Matrix4d T1 = dhTransform(a1, alpha1, d1, theta1);
    Eigen::Matrix4d T2 = dhTransform(a2, alpha2, d2, theta2 - theta2_offset);
    Eigen::Matrix4d T3 = dhTransform(a3, alpha3, d3, theta3 - theta3_offset);
    Eigen::Matrix4d T03 = T1 * T2 * T3;
    
    // --- Step 5: Solve for Wrist Joints (4-6) ---
    Eigen::Matrix3d R03 = T03.block<3,3>(0,0);
    Eigen::Matrix3d R36 = R03.transpose() * R06;
    
    auto solveWrist = [&](const Eigen::Matrix3d& R36) -> Eigen::Vector3d {
        Eigen::Vector3d wrist;
        const double epsilon = 1e-3;
        double s_theta5 = sqrt(R36(0,2) * R36(0,2) + R36(1,2) * R36(1,2));
        
        if (fabs(s_theta5) < epsilon) {
            std::cerr << "Singular wrist configuration detected." << std::endl;
            double theta4 = 0.0;
            double theta5 = atan2(s_theta5, R36(2,2));
            double theta6 = atan2(R36(1,0), R36(0,0));
            wrist << theta4, theta5, theta6;
            return wrist;
        }
        
        double theta5_A = atan2(s_theta5, R36(2,2));
        double theta4_A = atan2(-R36(1,2), -R36(0,2));
        double theta6_A = atan2(-R36(2,1), R36(2,0));
        
        double theta5_B = atan2(-s_theta5, R36(2,2));
        double theta4_B = atan2(R36(1,2), R36(0,2));
        double theta6_B = atan2(R36(2,1), -R36(2,0));
        
        double normA2 = theta4_A * theta4_A + theta5_A * theta5_A + theta6_A * theta6_A;
        double normB2 = theta4_B * theta4_B + theta5_B * theta5_B + theta6_B * theta6_B;
        
        double theta4, theta5, theta6;
        if (normA2 <= normB2) {
            theta4 = theta4_A;
            theta5 = theta5_A;
            theta6 = theta6_A;
        } else {
            theta4 = theta4_B;
            theta5 = theta5_B;
            theta6 = theta6_B;
        }
        
        wrist << theta4, theta5, theta6;
        return wrist;
    };

    Eigen::Vector3d wrist = solveWrist(R36);
    
    // --- Step 6: Assemble the Full Joint Solution ---
    Eigen::VectorXd sol(6);
    sol << theta1, theta2, theta3, wrist(0), wrist(1), wrist(2);
    
    return sol;  // Return the single solution directly
}

// Convenience overload for std::vector input.
Eigen::VectorXd analyticIK(const std::vector<double>& target) {
    if (target.size() != 3 && target.size() != 6 && target.size() != 7) {
        std::cerr << "Target vector must be of size 3 (wrist center), 6 (RPY) or 7 (quaternion)." << std::endl;
        return Eigen::VectorXd(0);  // Return empty vector to indicate error
    }
    
    Eigen::VectorXd target_eigen(target.size());
    for (size_t i = 0; i < target.size(); ++i)
        target_eigen(i) = target[i];
    
    return analyticIK(target_eigen);
}

//======================================================================
//  Geometric Jacobian
//======================================================================
/**
 * Computes the 6x6 geometric Jacobian for the given joint angles.
 */
Eigen::MatrixXd geometricJacobian(const Eigen::VectorXd& q) {
    if (q.size() != 6) {
        throw std::runtime_error("Expected 6 joint angles for Jacobian calculation, got " +
                                 std::to_string(q.size()));
    }
    
    double theta1 = q(0);
    double theta2 = q(1) - theta2_offset;
    double theta3 = q(2) - theta3_offset;
    double theta4 = q(3);
    double theta5 = q(4);
    double theta6 = q(5);
    
    Eigen::Matrix4d T1 = dhTransform(a1, alpha1, d1, theta1);
    Eigen::Matrix4d T2 = dhTransform(a2, alpha2, d2, theta2);
    Eigen::Matrix4d T3 = dhTransform(a3, alpha3, d3, theta3);
    Eigen::Matrix4d T4 = dhTransform(a4, alpha4, d4, theta4);
    Eigen::Matrix4d T5 = dhTransform(a5, alpha5, d5, theta5);
    Eigen::Matrix4d T6 = dhTransform(a6, alpha6, d6, theta6);
    
    Eigen::Matrix4d T_e = Eigen::Matrix4d::Identity();
    T_e.block<3,3>(0,0) = R_e;
    
    Eigen::Matrix4d T0_1 = T1;
    Eigen::Matrix4d T0_2 = T0_1 * T2;
    Eigen::Matrix4d T0_3 = T0_2 * T3;
    Eigen::Matrix4d T0_4 = T0_3 * T4;
    Eigen::Matrix4d T0_5 = T0_4 * T5;
    Eigen::Matrix4d T0_6 = T0_5 * T6;
    Eigen::Matrix4d T_final = T0_6 * T_e;
    
    Eigen::Vector3d p_e = T_final.block<3,1>(0,3);
    
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d z0 = Eigen::Vector3d::UnitZ();
    
    Eigen::Vector3d p1 = T0_1.block<3,1>(0,3);
    Eigen::Vector3d z1 = T0_1.block<3,1>(0,2);
    
    Eigen::Vector3d p2 = T0_2.block<3,1>(0,3);
    Eigen::Vector3d z2 = T0_2.block<3,1>(0,2);
    
    Eigen::Vector3d p3 = T0_3.block<3,1>(0,3);
    Eigen::Vector3d z3 = T0_3.block<3,1>(0,2);
    
    Eigen::Vector3d p4 = T0_4.block<3,1>(0,3);
    Eigen::Vector3d z4 = T0_4.block<3,1>(0,2);
    
    Eigen::Vector3d p5 = T0_5.block<3,1>(0,3);
    Eigen::Vector3d z5 = T0_5.block<3,1>(0,2);
    
    Eigen::MatrixXd J(6,6);
    J.block<3,1>(0,0) = z0.cross(p_e - p0);
    J.block<3,1>(3,0) = z0;
    J.block<3,1>(0,1) = z1.cross(p_e - p1);
    J.block<3,1>(3,1) = z1;
    J.block<3,1>(0,2) = z2.cross(p_e - p2);
    J.block<3,1>(3,2) = z2;
    J.block<3,1>(0,3) = z3.cross(p_e - p3);
    J.block<3,1>(3,3) = z3;
    J.block<3,1>(0,4) = z4.cross(p_e - p4);
    J.block<3,1>(3,4) = z4;
    J.block<3,1>(0,5) = z5.cross(p_e - p5);
    J.block<3,1>(3,5) = z5;
    
    return J;
}

// Overload for std::vector input.
Eigen::MatrixXd geometricJacobian(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != 6) {
        std::cerr << "Error: Expected 6 joint angles for Jacobian calculation." << std::endl;
        exit(EXIT_FAILURE);
    }
    Eigen::VectorXd j(6);
    for (size_t i = 0; i < 6; ++i)
         j(i) = joint_angles[i];
    return geometricJacobian(j);
}

//======================================================================
//  Pseudoinverse Jacobian (with Damped Least Squares)
//======================================================================
Eigen::MatrixXd invertJacobian(const Eigen::MatrixXd& J, double damping, double threshold) {
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double minSingular = svd.singularValues().minCoeff();

    if (minSingular > threshold) {
        return svd.matrixV() *
               svd.singularValues().asDiagonal().inverse() *
               svd.matrixU().transpose();
    } else {
        Eigen::VectorXd singularValues = svd.singularValues();
        Eigen::MatrixXd sigmaInv(singularValues.size(), singularValues.size());
        sigmaInv.setZero();
        for (int i = 0; i < singularValues.size(); ++i) {
            double sigma = singularValues(i);
            sigmaInv(i, i) = sigma / (sigma * sigma + damping * damping);
        }
        return svd.matrixV() * sigmaInv * svd.matrixU().transpose();
    }
}

//======================================================================
//  Analytic Jacobian (Euler Angle Conversion)
//======================================================================
/**
 * Computes the analytic Jacobian that relates joint velocities to the time
 * derivative of the end-effector pose [x, y, z, roll, pitch, yaw].
 */
Eigen::MatrixXd analyticJacobian(const Eigen::VectorXd& q) {
    Eigen::MatrixXd J_geo = geometricJacobian(q);
    Eigen::VectorXd pose = forwardKinematics(q);
    Eigen::Quaterniond q_e(pose(6), pose(3), pose(4), pose(5));
    
    double sinr_cosp = 2.0 * (q_e.w() * q_e.x() + q_e.y() * q_e.z());
    double cosr_cosp = 1.0 - 2.0 * (q_e.x() * q_e.x() + q_e.y() * q_e.y());
    double roll = atan2(sinr_cosp, cosr_cosp);
    
    double sinp = 2.0 * (q_e.w() * q_e.y() - q_e.z() * q_e.x());
    double pitch = (fabs(sinp) >= 1 ? copysign(M_PI / 2, sinp) : asin(sinp));
    
    // Create transformation matrix from angular velocities (in base frame) to Euler angle rates
    Eigen::Matrix3d T_inv;
    double cos_pitch = cos(pitch);
    if (fabs(cos_pitch) < 1e-4) {
        cos_pitch = (cos_pitch >= 0 ? 1e-4 : -1e-4);
        std::cerr << "Warning: Pitch angle near singularity for analytic Jacobian computation." << std::endl;
    }
    T_inv << 1, sin(roll) * tan(pitch), cos(roll) * tan(pitch),
             0, cos(roll),             -sin(roll),
             0, sin(roll) / cos_pitch,  cos(roll) / cos_pitch;
    
    // The geometric Jacobian's angular part gives angular velocities in base frame
    // but we need to transform them from end-effector frame to base frame first
    Eigen::Matrix3d R_base_to_ee = q_e.toRotationMatrix();
    Eigen::Matrix3d R_ee_to_base = R_base_to_ee.transpose();
    
    Eigen::MatrixXd J_analytic(6, 6);
    J_analytic.topRows(3) = J_geo.topRows(3);
    // Transform angular velocities from end-effector frame to base frame, then apply T_inv
    J_analytic.bottomRows(3) = T_inv * R_ee_to_base * J_geo.bottomRows(3);
    
    return J_analytic;
}

// Overload for std::vector input.
Eigen::MatrixXd analyticJacobian(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != 6) {
        std::cerr << "Error: Expected 6 joint angles for analytic Jacobian calculation." << std::endl;
        exit(EXIT_FAILURE);
    }
    Eigen::VectorXd q(6);
    for (size_t i = 0; i < 6; ++i)
        q(i) = joint_angles[i];
    return analyticJacobian(q);
}

//======================================================================
//  Wrist Center Jacobian
//======================================================================
/**
 * Computes a reduced 3x3 Jacobian for the first 3 joints that affect the wrist center position.
 * The Jacobian is scaled by the maximum joint velocities to produce a normalized Jacobian.
 * This is useful for calculating the velocity of the wrist center.
 * Also returns the largest singular value of the scaled Jacobian for manipulability analysis.
 */
std::pair<Eigen::MatrixXd, double> scaledReducedJacobian(const Eigen::VectorXd& q) {
    if (q.size() != 6) {
        throw std::runtime_error("Expected 6 joint angles for wristCenterJacobian calculation, got " +
                                 std::to_string(q.size()));
    }
    
    double theta1 = q(0);
    double theta2 = q(1) - theta2_offset;
    double theta3 = q(2) - theta3_offset;
    
    // Compute transformations for the first 3 joints
    Eigen::Matrix4d T1 = dhTransform(a1, alpha1, d1, theta1);
    Eigen::Matrix4d T2 = dhTransform(a2, alpha2, d2, theta2);
    Eigen::Matrix4d T3 = dhTransform(a3, alpha3, d3, theta3);
    
    // Compose the transformations
    Eigen::Matrix4d T0_1 = T1;
    Eigen::Matrix4d T0_2 = T0_1 * T2;
    Eigen::Matrix4d T0_3 = T0_2 * T3;
    
    // Calculate the position of the wrist center (after joint 3)
    Eigen::Vector3d p_wc = T0_3.block<3,1>(0,3);
    
    // Calculate the rotation axes for the first 3 joints in the global frame
    Eigen::Vector3d p0 = Eigen::Vector3d::Zero();
    Eigen::Vector3d z0 = Eigen::Vector3d::UnitZ();
    
    Eigen::Vector3d p1 = T0_1.block<3,1>(0,3);
    Eigen::Vector3d z1 = T0_1.block<3,1>(0,2);
    
    Eigen::Vector3d p2 = T0_2.block<3,1>(0,3);
    Eigen::Vector3d z2 = T0_2.block<3,1>(0,2);
    
    // Construct the Jacobian matrix (3x3)
    Eigen::MatrixXd J(3,3);
    
    // Each column represents the contribution of the joint to the linear velocity of the wrist center
    J.col(0) = z0.cross(p_wc - p0);  // Linear velocity contribution from joint 1
    J.col(1) = z1.cross(p_wc - p1);  // Linear velocity contribution from joint 2
    J.col(2) = z2.cross(p_wc - p2);  // Linear velocity contribution from joint 3
    
    // Scale the Jacobian by the maximum joint velocities: J̃ = J diag(q̇_max)
    Eigen::Vector3d q_dot_max(max_joint_velocity, max_joint_velocity, max_joint_velocity);
    Eigen::MatrixXd J_scaled = J * q_dot_max.asDiagonal();
    
    // Calculate the largest singular value of the scaled Jacobian
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_scaled, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double maxSingularValue = svd.singularValues()(0);
    
    return std::make_pair(J_scaled, maxSingularValue);
}

// Overload for std::vector input
std::pair<Eigen::MatrixXd, double> scaledReducedJacobian(const std::vector<double>& joint_angles) {
    if (joint_angles.size() != 6) {
        std::cerr << "Error: Expected 6 joint angles for wrist center Jacobian calculation." << std::endl;
        exit(EXIT_FAILURE);
    }
    Eigen::VectorXd q(6);
    for (size_t i = 0; i < 6; ++i)
        q(i) = joint_angles[i];
    return scaledReducedJacobian(q);
}

//======================================================================
//  Cartesian to Joint Velocity Conversion
//======================================================================
/**
 * Converts a Cartesian velocity twist to joint velocities using an efficient
 * damped least squares approach when near singularity.
 */
Eigen::VectorXd cartesianToJointVel(const Eigen::MatrixXd& J, const Eigen::VectorXd& twist) {
    if (J.rows() != 6 || J.cols() != 6 || twist.size() != 6) {
        std::cerr << "Error: cartesianToJointVel requires 6x6 Jacobian and 6D twist vector." << std::endl;
        return Eigen::VectorXd::Zero(6);
    }
    
    static constexpr double lambda = 1e-2;  // DLS gain
    Eigen::PartialPivLU<Eigen::Matrix<double, 6, 6>> lu(J);
    
    // Quick singularity test
    if (std::abs(lu.determinant()) > 1e-4) {
        // Not near singularity, use direct solve
        return lu.solve(twist);  // ~0.6 µs
    } else {
        // Near singularity, use damped least squares
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += lambda * lambda;  // damping
        Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(JJt);
        return J.transpose() * llt.solve(twist);    // ~1 µs
    }
}

// Overload for std::vector input
std::vector<double> cartesianToJointVel(const std::vector<std::vector<double>>& J, 
                                        const std::vector<double>& twist) {
    if (J.size() != 6 || twist.size() != 6) {
        std::cerr << "Error: cartesianToJointVel requires 6x6 Jacobian and 6D twist vector." << std::endl;
        return std::vector<double>(6, 0.0);
    }
    
    // Convert std::vector to Eigen
    Eigen::MatrixXd J_eigen(6, 6);
    for (int i = 0; i < 6; ++i) {
        if (J[i].size() != 6) {
            std::cerr << "Error: Invalid Jacobian matrix dimensions." << std::endl;
            return std::vector<double>(6, 0.0);
        }
        for (int j = 0; j < 6; ++j) {
            J_eigen(i, j) = J[i][j];
        }
    }
    
    Eigen::VectorXd twist_eigen(6);
    for (int i = 0; i < 6; ++i) {
        twist_eigen(i) = twist[i];
    }
    
    // Call the Eigen version
    Eigen::VectorXd result_eigen = cartesianToJointVel(J_eigen, twist_eigen);
    
    // Convert back to std::vector
    std::vector<double> result(6);
    for (int i = 0; i < 6; ++i) {
        result[i] = result_eigen(i);
    }
    
    return result;
}

} // namespace dynamic_grasping_controller
