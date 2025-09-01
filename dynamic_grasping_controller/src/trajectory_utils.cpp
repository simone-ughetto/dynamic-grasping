/**
 * @file trajectory_utils.cpp
 * @brief Implementation of utility functions for trajectory planning and manipulation
 */

#pragma region Includes
#include "dynamic_grasping_controller/trajectory_utils.hpp"
#include "dynamic_grasping_controller/widowx_kinematics_library.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#pragma endregion Includes

namespace dynamic_grasping_controller
{

#pragma region Coordinate System Transformations
/*==============================================================================
 * Coordinate System Transformations
 *============================================================================*/

Eigen::Vector3d quaternionToEuler(const Eigen::Quaterniond& quat)
{
  Eigen::Vector3d euler;
  
  // Roll (x-axis rotation)
  double sinr_cosp = 2.0 * (quat.w() * quat.x() + quat.y() * quat.z());
  double cosr_cosp = 1.0 - 2.0 * (quat.x() * quat.x() + quat.y() * quat.y());
  euler(0) = std::atan2(sinr_cosp, cosr_cosp);
  
  // Pitch (y-axis rotation)
  double sinp = 2.0 * (quat.w() * quat.y() - quat.z() * quat.x());
  if (std::abs(sinp) >= 1)
    euler(1) = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
  else
    euler(1) = std::asin(sinp);
  
  // Yaw (z-axis rotation)
  double siny_cosp = 2.0 * (quat.w() * quat.z() + quat.x() * quat.y());
  double cosy_cosp = 1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z());
  euler(2) = std::atan2(siny_cosp, cosy_cosp);
  
  return euler;
}

Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw)
{
  Eigen::Quaterniond quat;
  
  // Calculate trig identities once
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);
  
  // Quaternion from Euler angles: q = q_z * q_y * q_x
  quat.w() = cr * cp * cy + sr * sp * sy;
  quat.x() = sr * cp * cy - cr * sp * sy;
  quat.y() = cr * sp * cy + sr * cp * sy;
  quat.z() = cr * cp * sy - sr * sp * cy;
  
  return quat;
}
#pragma endregion

#pragma region Object Motion Planning
/*==============================================================================
 * Object Motion Planning
 *============================================================================*/

Eigen::Quaterniond computeDesiredOrientation(
  const Eigen::Vector3d& object_pos,
  const Eigen::Vector3d& object_vel)
{
  Eigen::Vector3d desired_x, desired_y, desired_z;

  // If object is moving, align with velocity direction
  if (object_vel.norm() > 1e-3) {
    // Create a perpendicular vector in the horizontal plane to velocity
    Eigen::Vector3d vel_proj_perpendicular(-object_vel.y(), object_vel.x(), 0.0);

    // Form orthonormal coordinate frame
    desired_z = object_vel.normalized();
    desired_y = vel_proj_perpendicular.normalized();
    desired_x = desired_y.cross(desired_z).normalized();

    // Apply small angle correction to avoid vertical singularity BEFORE mirroring
    Eigen::Vector3d verticality = desired_x - Eigen::Vector3d(0.0, 0.0, -1.0);
    if (verticality.norm() < 0.1) {
      // Add a small tilt
      double angle_rad = -7 * M_PI / 180.0;

      Eigen::AngleAxisd rotation(angle_rad, desired_y);
      desired_x = rotation * desired_x;
      desired_x.normalize();
      desired_z = desired_x.cross(desired_y).normalized();
    }

    // (Optional) Avoid large wrist rotations ---------------------------------
    Eigen::Vector3d neg_desired_z(-desired_z.x(), -desired_z.y(), 0);
    
    // Calculate dot products to determine which has smaller angle
    double dot_positive = desired_z.dot(object_pos);
    double dot_negative = neg_desired_z.dot(object_pos);
    
    // If negative desired_z has better alignment, mirror z-axis about (0,0,1) and flip y-axis
    if (dot_negative > dot_positive) {
      // Mirror desired_z about (0,0,1): mirrored = 2 * (z · n) * n - z, where n = (0,0,1)
      Eigen::Vector3d mirror_normal(0.0, 0.0, 1.0);
      double dot_product = desired_z.dot(mirror_normal);
      desired_z = 2.0 * dot_product * mirror_normal - desired_z;
      desired_y = -desired_y;
      desired_x = desired_y.cross(desired_z).normalized();
    }
    // -----------------------------------------------------------------------
  } 
  // If object is stationary, align with position vector
  else {
    // Create a perpendicular vector in the horizontal plane to position
    Eigen::Vector3d pos_proj_perpendicular(-object_pos.y(), object_pos.x(), 0.0);
    
    // Form orthonormal coordinate frame
    desired_z = object_pos.normalized();
    desired_y = pos_proj_perpendicular.normalized();
    desired_x = desired_y.cross(desired_z).normalized();

    // Apply small angle correction to avoid vertical singularity for stationary case
    Eigen::Vector3d verticality = desired_x - Eigen::Vector3d(0.0, 0.0, -1.0);
    if (verticality.norm() < 0.1) {
      // Add a small tilt
      double angle_rad = -7 * M_PI / 180.0;  
        
      Eigen::AngleAxisd rotation(angle_rad, desired_y);
      desired_x = rotation * desired_x;
      desired_x.normalize();
      desired_z = desired_x.cross(desired_y).normalized();
    }
  }

  // Create rotation matrix
  Eigen::Matrix3d R;
  R.col(0) = desired_x;
  R.col(1) = desired_y;
  R.col(2) = desired_z;

  // Create quaternion and normalize
  Eigen::Quaterniond q(R);
  q.normalize();
  
  return q;
}

Eigen::Vector3d predictObjectPosition(
  const Eigen::Vector3d& object_pos,
  const Eigen::Vector3d& object_vel,
  double time_horizon)
{
  return object_pos + object_vel * time_horizon;
}

double computeApproachDuration(
  const Eigen::Vector3d& p_obj,
  const Eigen::Vector3d& v_obj,
  const Eigen::Vector3d& p_ee,
  double offset,
  double max_vel,
  double dt)
{
  const double fallback_t1 = 4.0;  // Default duration if calculation fails
  const double v_e = 0.4 * max_vel; // 0.4 of max velocity for end-effector
  const double epsilon = 1e-4;      // Small value to check for near-zero conditions

  // Create a local copy with the Z offset
  Eigen::Vector3d p_obj_with_offset = p_obj + Eigen::Vector3d(0.0, 0.0, offset);
  Eigen::Vector3d delta = p_obj_with_offset - p_ee;
  
  // Compute coefficients for quadratic equation: a*t^2 + b*t + c = 0
  double a = v_obj.squaredNorm() - v_e * v_e;
  double b = 2.0 * v_obj.dot(delta);
  double c = delta.squaredNorm();

  double t1 = std::numeric_limits<double>::infinity();

  // Check if 'a' is close to zero (linear equation case)
  if (std::abs(a) < epsilon) {
    // If 'b' is also close to zero, the equation is degenerate
    if (std::abs(b) < epsilon) {
      t1 = fallback_t1;
    } else {
      // Solve linear equation: b*t + c = 0 => t = -c / b
      double linear_t = -c / b;
      if (linear_t > epsilon) {
        t1 = linear_t;
      } else {
        t1 = fallback_t1;
      }
    }
  } else {
    // Solve quadratic equation: a*t^2 + b*t + c = 0
    double disc = b * b - 4.0 * a * c;
    if (disc < 0) {
      // No real solutions
      t1 = fallback_t1;
    } else {
      double sqrt_disc = std::sqrt(disc);
      double t1a = (-b + sqrt_disc) / (2.0 * a);
      double t1b = (-b - sqrt_disc) / (2.0 * a);

      // Find the smallest positive root
      double positive_t1 = std::numeric_limits<double>::infinity();
      if (t1a > epsilon) {
        positive_t1 = t1a;
      }
      if (t1b > epsilon && t1b < positive_t1) {
        positive_t1 = t1b;
      }

      if (!std::isfinite(positive_t1)) {
        t1 = fallback_t1;
      } else {
        t1 = positive_t1;
      }
    }
  }

  // If t1 is still infinity
  if (!std::isfinite(t1)) {
    t1 = fallback_t1;
  }

  // Adjust t1 to be the smallest multiple of dt >= t1
  double adjusted_t1 = std::ceil(t1 / dt) * dt;

  // Ensure the adjusted time is not excessively small
  if (adjusted_t1 < dt) {
    adjusted_t1 = dt;
  }

  return adjusted_t1;
}
#pragma endregion

#pragma region Trajectory Sampling and Processing
/*==============================================================================
 * Trajectory Sampling and Processing
 *============================================================================*/

double sampleTrajectorySegment(
  const ruckig::Trajectory<6>& traj,
  int phase,
  double time_offset,
  trajectory_msgs::msg::JointTrajectory& jt,
  std::vector<Eigen::Vector3d>& p,
  std::vector<Eigen::Vector3d>& rpy,
  std::vector<Eigen::Vector3d>& v_lin,
  std::vector<Eigen::Vector3d>& v_ang,
  double dt,
  const std::vector<std::string>& joint_names,
  const rclcpp::Logger& log,
  bool skip_first,
  bool include_velocity)
{
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  // Early exit for empty trajectory
  const double duration = traj.get_duration();
  if (duration <= 0.0) {
    RCLCPP_WARN(log, "[Phase %d] Empty trajectory segment", phase);
    return time_offset;
  }

  // One-shot initialization
  if (jt.joint_names.empty())
    jt.joint_names = joint_names;

  const int n_steps = static_cast<int>(std::floor(duration / dt + 1e-12));
  const size_t expected_pts = (skip_first ? n_steps : (n_steps + 1)) + 1;  // +1 for residual
  jt.points.reserve(jt.points.size() + expected_pts);
  p.reserve(p.size() + expected_pts);
  rpy.reserve(rpy.size() + expected_pts);
  v_lin.reserve(v_lin.size() + expected_pts);
  v_ang.reserve(v_ang.size() + expected_pts);

  const auto make_time = [](double t_sec) {
    builtin_interfaces::msg::Duration d;
    d.sec = static_cast<int32_t>(t_sec);
    d.nanosec = static_cast<uint32_t>(std::lrint((t_sec - d.sec) * 1e9));
    return d;
  };

  // Reusable state buffers
  std::array<double, 6> pos{}, vel{}, acc{};
  Vector6d tgt, joint_pos, joint_vel, cart_vel;
  Matrix6d J;

  // Helper to append a point in-place
  auto emplace_point = [&](double global_t, const Vector6d& q, const Vector6d* dq) {
    auto& pt = jt.points.emplace_back();
    pt.positions.assign(q.data(), q.data() + 6);
    if (include_velocity && dq)
      pt.velocities.assign(dq->data(), dq->data() + 6);
    pt.time_from_start = make_time(global_t);
  };

  // Main uniform sampling loop
  double t_local = skip_first ? dt : 0.0;
  double t_global = time_offset + t_local;
  int attempted = 0;

  for (int k = skip_first ? 1 : 0; k <= n_steps; ++k, t_local += dt, t_global += dt) {
    ++attempted;
    traj.at_time(t_local, pos, vel, acc);

    // Cartesian logging & storage
    p.emplace_back(pos[0], pos[1], pos[2]);
    rpy.emplace_back(pos[3], pos[4], pos[5]);
    v_lin.emplace_back(vel[0], vel[1], vel[2]);
    v_ang.emplace_back(vel[3], vel[4], vel[5]);

    // Inverse kinematics
    tgt << pos[0], pos[1], pos[2], pos[3], pos[4], pos[5];
    Eigen::VectorXd joint_pos = analyticIK(tgt);
    if (joint_pos.size() == 0) {  // Check for error
      RCLCPP_WARN_THROTTLE(
        log, *rclcpp::Clock::make_shared(), 1.0,
        "[Phase %d] No IK at t=%.3f -> skipping", phase, t_global
      );
      continue;
    }

    // Optional joint velocities
    if (include_velocity) {
      try {
        J = analyticJacobian(joint_pos);
        cart_vel << vel[0], vel[1], vel[2], vel[3], vel[4], vel[5];
        joint_vel = cartesianToJointVel(J, cart_vel);
      } catch (const std::exception&) {
        joint_vel.setZero();
      }
      emplace_point(t_global, joint_pos, &joint_vel);
    } else {
      emplace_point(t_global, joint_pos, nullptr);
    }
  }

  // Residual sample if duration not multiple of dt
  const double residual = duration - n_steps * dt;
  if (residual > 1e-12) {
    const double t_res_local = duration;
    const double t_res_global = time_offset + t_res_local;
    ++attempted;

    traj.at_time(t_res_local, pos, vel, acc);

    p.emplace_back(pos[0], pos[1], pos[2]);
    rpy.emplace_back(pos[3], pos[4], pos[5]);
    v_lin.emplace_back(vel[0], vel[1], vel[2]);
    v_ang.emplace_back(vel[3], vel[4], vel[5]);

    tgt << pos[0], pos[1], pos[2], pos[3], pos[4], pos[5];
    const auto joint_pos = analyticIK(tgt);
    if (joint_pos.size() != 0) {
      if (include_velocity) {
        try {
          J = analyticJacobian(joint_pos);
          cart_vel << vel[0], vel[1], vel[2], vel[3], vel[4], vel[5];
          joint_vel = cartesianToJointVel(J, cart_vel);
        } catch (const std::exception&) {
          joint_vel.setZero();
        }
        emplace_point(t_res_global, joint_pos, &joint_vel);
      } else {
        emplace_point(t_res_global, joint_pos, nullptr);
      }
      RCLCPP_DEBUG(
        log,
        "[Phase %d] Added final residual sample of %.4f s",
        phase, residual
      );
    } else {
      RCLCPP_WARN(log, "[Phase %d] No IK for final point at t=%.3f", phase, t_res_global);
    }
  }

  // Summary
  const size_t added = expected_pts;
  RCLCPP_WARN(
    log,
    "[Phase %d] Attempted %d samples, added %zu points (%s)",
    phase, attempted, added,
    include_velocity ? "with velocities" : "position only"
  );

  return time_offset + duration;
}
#pragma endregion

#pragma region Robot Arm Max Velocity Estimation
/*==============================================================================
 * Trajectory Velocity Calculation
 *============================================================================*/

double computeAdaptiveMaxVelocity(
    const Eigen::VectorXd& start_pose,
    const Eigen::VectorXd& target_pose,
    double max_default_velocity,
    const rclcpp::Logger& logger)
{
  // Step 1: Calculate path length in Cartesian space
  Eigen::Vector3d start_pos = start_pose.segment<3>(0);
  Eigen::Vector3d target_pos = target_pose.segment<3>(0);
  double path_length = (target_pos - start_pos).norm();
  
  // Step 2: Determine number of samples (~1 cm spacing)
  int num_samples = std::ceil(path_length / 0.01) + 1;
  
  // Step 3: Interpolate poses and compute min max velocity
  double adaptive_max_velocity = std::numeric_limits<double>::infinity();
  int valid_samples = 0;
  
  for (int i = 0; i < num_samples; ++i) {
    double s = static_cast<double>(i) / (num_samples - 1);  // Parameter from 0 to 1
    
    // Linear interpolation of position
    Eigen::Vector3d interp_pos = start_pos + s * (target_pos - start_pos);
    
    // Create interpolated pose (keep orientation from target)
    Eigen::VectorXd interp_pose = target_pose;
    interp_pose.segment<3>(0) = interp_pos;
    
    // Compute IK solution for this interpolated pose
    Eigen::VectorXd solutions = analyticIK(interp_pose);
    
    if (solutions.size() != 0) {
      valid_samples++;
      
      // Compute the scaled reduced Jacobian and get max singular value
      auto [jacobian, max_singular_value] = scaledReducedJacobian(solutions);
      
      // Update the minimum of max singular values
      adaptive_max_velocity = std::min(adaptive_max_velocity, max_singular_value);
    }
  }
  
  // Step 4: Check if we have valid samples
  if (valid_samples == 0) {
    RCLCPP_WARN(logger, "Could not find valid IK solutions along path, using default max velocity");
    return max_default_velocity;  // Fallback to default
  }
  
  RCLCPP_INFO(logger, "Adaptive max velocity: %.4f m/s (from %d/%d valid samples, min σ_max: %.4f)", 
              adaptive_max_velocity, valid_samples, num_samples, adaptive_max_velocity);
              
  // Cap the adaptive velocity to respect the user-defined maximum
  return adaptive_max_velocity;
}
#pragma endregion

#pragma region Data Logging and Visualization
/*==============================================================================
 * Data Logging and Visualization
 *============================================================================*/

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
    const rclcpp::Logger& logger)
{
  // Open file for writing
  std::ofstream file(trajectory_log_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(logger, "Failed to open file for trajectory logging: %s", trajectory_log_path.c_str());
    return;
  }
  
  // Write CSV headers
  file << "time,";
  
  // Joint position headers
  for (const auto& name : joint_names) {
    file << name << "_pos,";
  }
  
  // Joint velocity headers
  for (const auto& name : joint_names) {
    file << name << "_vel,";
  }
  
  // Cartesian motion headers
  file << "x,y,z,roll,pitch,yaw,vx,vy,vz,wx,wy,wz,";
  
  // Object state headers
  file << "obj_x,obj_y,obj_z,obj_vx,obj_vy,obj_vz\n";
  
  // Write data rows
  for (size_t i = 0; i < joint_trajectory.points.size(); ++i) {
    const auto& point = joint_trajectory.points[i];
    
    // Convert ROS time to seconds
    double time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9;
    file << std::fixed << std::setprecision(6) << time << ",";
    
    // Write joint positions
    for (double pos : point.positions) {
      file << pos << ",";
    }
    
    // Write joint velocities
    for (double vel : point.velocities) {
      file << vel << ",";
    }
    
    // Write Cartesian positions
    if (i < cartesian_positions.size()) {
      file << cartesian_positions[i].x() << "," 
           << cartesian_positions[i].y() << "," 
           << cartesian_positions[i].z() << ",";
    } else {
      file << "0,0,0,";
    }
    
    // Write Euler angles (orientation)
    if (i < cartesian_orientations.size()) {
      file << cartesian_orientations[i].x() << "," 
           << cartesian_orientations[i].y() << "," 
           << cartesian_orientations[i].z() << ",";
    } else {
      file << "0,0,0,";
    }
    
    // Write Cartesian linear velocities
    if (i < cartesian_velocities.size()) {
      file << cartesian_velocities[i].x() << "," 
           << cartesian_velocities[i].y() << "," 
           << cartesian_velocities[i].z() << ",";
    } else {
      file << "0,0,0,";
    }
    
    // Write angular velocities
    if (i < angular_velocities.size()) {
      file << angular_velocities[i].x() << "," 
           << angular_velocities[i].y() << "," 
           << angular_velocities[i].z() << ",";
    } else {
      file << "0,0,0,";
    }
    
    // Write object state (constant for all points)
    file << object_pos.x() << "," 
         << object_pos.y() << "," 
         << object_pos.z() << "," 
         << object_vel.x() << "," 
         << object_vel.y() << "," 
         << object_vel.z() << "\n";
  }
  
  file.close();
  RCLCPP_INFO(logger, "Saved trajectory data to %s", trajectory_log_path.c_str());
}

// Helper function to load a URDF or XACRO file into a string
static std::string loadURDF(const std::string& path)
{
  // Process xacro transparently (requires xacro in $PATH)
  if (path.rfind(".xacro") != std::string::npos)
  {
    std::string cmd = "xacro " + path;
    FILE* pipe = popen(cmd.c_str(), "r");
    if(!pipe) { return {}; }
    char buffer[4096];
    std::ostringstream ss;
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) { ss << buffer; }
    pclose(pipe);
    return ss.str();
  }

  // Plain URDF text file
  std::ifstream ifs(path);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

bool predictGrasp(const rclcpp::Node::SharedPtr &node,
                const rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr &pub,
                const geometry_msgs::msg::Pose &object_pose,
                const geometry_msgs::msg::Twist &object_twist,
                double t1,
                const std::string &hardware_type,
                const std::string &urdf_path,
                bool &has_spawned_marker)
{ 
  const double t_grasp = t1 + 0.4 + 0.7;  // User-specified rule

  // Simple constant-velocity extrapolation
  geometry_msgs::msg::PoseStamped pred;
  pred.header.stamp = node->now() + rclcpp::Duration::from_seconds(t_grasp);
  pred.header.frame_id = "world";

  pred.pose.position.x = object_pose.position.x + object_twist.linear.x * t_grasp;
  pred.pose.position.y = object_pose.position.y + object_twist.linear.y * t_grasp;
  pred.pose.position.z = object_pose.position.z + object_twist.linear.z * t_grasp;
  pred.pose.orientation = object_pose.orientation;  // Keep current orientation

  // Publish prediction
  pub->publish(pred);
  RCLCPP_INFO(node->get_logger(),
              "Published grasp prediction at t=%.3f s  (%.3f, %.3f, %.3f)",
              t_grasp,
              pred.pose.position.x, pred.pose.position.y, pred.pose.position.z);

  // Optionally spawn marker in Gazebo Classic
  if (hardware_type == "gz_classic") {
    static rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client;
    static rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client;
  
    // Create the clients once (thread-safe in C++11)
    if (!client) {
      client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    }
    
    if (!delete_client) {
      delete_client = node->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
    }

    // If we've already spawned a marker, delete it first
    if (has_spawned_marker) {
      if (delete_client->wait_for_service(std::chrono::milliseconds(100))) {
        auto delete_req = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        delete_req->name = "visual_cylinder";
        
        // Send request without waiting (fire and forget)
        delete_client->async_send_request(delete_req,
            [node](rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedFuture future) {
              if (future.valid()) {
                RCLCPP_INFO(node->get_logger(), "Visual cylinder delete request completed");
              }
            });
        
        // Short sleep to allow service communication to process
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        has_spawned_marker = false;
        RCLCPP_INFO(node->get_logger(), "Previous visual_cylinder delete requested");
      } else {
        RCLCPP_WARN(node->get_logger(), "/delete_entity service not available");
      }
    }
  
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(node->get_logger(), "/spawn_entity service not available");
      return false;
    }
  
    auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    req->name = "visual_cylinder";
    req->xml = loadURDF(urdf_path);
    req->robot_namespace = "";
    req->initial_pose = pred.pose;
  
    // Fire and forget - don't wait for response to avoid executor conflicts
    client->async_send_request(req,
        [node, &has_spawned_marker](rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedFuture future) {
          if (future.valid()) {
            auto result = future.get();
            RCLCPP_INFO(node->get_logger(),
                    "SpawnEntity reply: success=%s  msg=%s",
                    result->success ? "true" : "false",
                    result->status_message.c_str());
            if (result->success) {
              has_spawned_marker = true;
            }
          }
        });
  }

  return true;
}
#pragma endregion

#pragma region Trajectory Time Estimation
/*==============================================================================
 * Trajectory Time Estimation
 *============================================================================*/

double estimateEETravelTime(double distance, double max_vel, double max_acc) {
  const double t_acc = max_vel / max_acc;
  const double s_acc = 0.5 * max_acc * t_acc * t_acc;  // Distance to full speed

  if (distance <= 2.0 * s_acc) {  // Triangular profile
    const double t_peak = std::sqrt(distance / max_acc);
    return 2.0 * t_peak;
  }
  // Trapezoidal profile
  const double s_cruise = distance - 2.0 * s_acc;
  const double t_cruise = s_cruise / max_vel;
  return 2.0 * t_acc + t_cruise;
}

PredResult objectTrajectoryEstimator(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& v0,
    double inner_radius)
{
  PredResult out;
  out.idx_enter = -1;
  out.inside_window = 0.0;

  const double speed = v0.norm();
  
  // Handle stationary objects (zero velocity) by treating them as constant position
  double step_dt;
  if (speed < 1e-4) {
    // For stationary objects, use a fixed time step and only evaluate current position
    step_dt = 0.1;  // 0.1 second intervals for stationary objects
  } else {
    step_dt = kStepLen / speed;  // Time step for ~1 cm spacing for moving objects
  }
  const int N = 150;

  out.pos.reserve(N);
  out.t.reserve(N);

  bool reachable_started = false;

  for (int i = 0; i < N; ++i) {
    const double ti = i * step_dt;
    
    // For stationary objects, position remains constant; for moving objects, extrapolate
    const Eigen::Vector3d pi = (speed < 1e-4) ? p0 : (p0 + v0 * ti);
    
    out.pos.push_back(pi);
    out.t.push_back(ti);

    // Decide reachability
    bool reach_ok = false;
    if (pi.norm() > inner_radius) {  // Maintain inner exclusion
      Eigen::Quaterniond q = computeDesiredOrientation(pi, v0);
      Eigen::VectorXd tgt(7);
      tgt << pi.x(), pi.y(), pi.z(), q.x(), q.y(), q.z(), q.w();
      reach_ok = isInWorkspace(tgt);
    }

    if (reach_ok) {
      if (!reachable_started) {
        reachable_started = true;
        out.idx_enter = i;  // First valid point
      }
      out.inside_window += step_dt;
    } else if (reachable_started) {
      break;  // Contiguous window ended
    }
    
    // For stationary objects, we only need to check once since position doesn't change
    if (speed < 1e-4 && i == 0) {
      if (reach_ok) {
        // Stationary object is immediately reachable and stays reachable
        out.inside_window = 150.0 * step_dt; // Large enough duration
      }
      break; // No need to continue the loop for stationary objects
    }
  }
  return out;
}
#pragma endregion

} // namespace dynamic_grasping_controller