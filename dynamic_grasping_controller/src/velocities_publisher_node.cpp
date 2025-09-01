#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "dynamic_grasping_controller/widowx_kinematics_library.hpp" // geom./analytic J, FK

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <numeric>
#include <optional>
#include <string>
#include <vector>

class VelocityPublisher : public rclcpp::Node
{
public:
  VelocityPublisher()
  : Node("velocity_publisher"),
    window_size_(50)
  {
    hardware_type_ = this->declare_parameter("hardware_type", std::string("gz_classic"));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/wx250s/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&VelocityPublisher::jointStateCb, this, std::placeholders::_1));

    velocity_pub_        = create_publisher<sensor_msgs::msg::JointState>("/wx250s/joint_velocities", 10);
    ee_velocity_pub_     = create_publisher<geometry_msgs::msg::TwistStamped>("/wx250s/end_effector_velocities", 10);
    ee_velocity_eul_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/wx250s/end_effector_velocities_euler", 10);

    RCLCPP_INFO(get_logger(), "Velocity publisher running in %s mode; %zu‑sample moving average.",
                hardware_type_.c_str(), window_size_);

    latest_joint_pos_ = Eigen::VectorXd::Zero(6);
    vel_buffers_.resize(6);       // per‑joint buffers
    ee_linear_buffers_.resize(3); // v_x, v_y, v_z
    ee_angular_buffers_.resize(3);// ω_x, ω_y, ω_z
  }

private:
  // === Callback === //
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Build name‑to‑index map once.
    if (name_to_index_.empty()) {
      for (size_t i = 0; i < msg->name.size(); ++i)
        name_to_index_[msg->name[i]] = i;
      RCLCPP_INFO(get_logger(), "Mapped %zu joint names from joint_states.", name_to_index_.size());
    }

    static const std::vector<std::string> kJoints {
      "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"};

    // --- Extract joint positions --- //
    Eigen::VectorXd q(6);
    for (size_t i = 0; i < kJoints.size(); ++i) {
      auto it = name_to_index_.find(kJoints[i]);
      if (it == name_to_index_.end() || it->second >= msg->position.size()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5.0,
                             "Joint '%s' missing — skipping frame.", kJoints[i].c_str());
        return;
      }
      q(i) = msg->position[it->second];
    }

    // True if we are on real hardware *and* velocity array is populated.
    const bool have_hw_vel = (hardware_type_ == "actual" &&
                              !msg->velocity.empty() &&
                              msg->velocity.size() >= msg->name.size());

    const double t = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

    Eigen::VectorXd raw_joint_vel(6);
    // ------------------------------------ joint velocities ----------------------------------- //
    if (have_hw_vel) {
      for (size_t i = 0; i < kJoints.size(); ++i)
        raw_joint_vel(i) = msg->velocity[name_to_index_[kJoints[i]]];
    } else {
      if (!prev_time_) {
        prev_time_ = t; prev_pos_ = q; prev_quat_.reset();
        publishZeroes(kJoints); return; // need history first
      }
      const double dt = t - *prev_time_;
      if (dt <= std::numeric_limits<double>::epsilon()) return;
      raw_joint_vel = (q - *prev_pos_) / dt;
    }

    // ------------------------------- moving‑average filter (joints) ----------------------------- //
    std::vector<double> joint_vel_avg(6);
    for (size_t i = 0; i < 6; ++i) {
      auto &buf = vel_buffers_[i];
      buf.push_back(raw_joint_vel(i));
      if (buf.size() > window_size_) buf.pop_front();
      joint_vel_avg[i] = std::accumulate(buf.begin(), buf.end(), 0.0) / buf.size();
    }

    // Publish smoothed joint velocities
    sensor_msgs::msg::JointState jv_msg;
    jv_msg.header.stamp = now();
    jv_msg.name   = kJoints;
    jv_msg.velocity = joint_vel_avg;
    velocity_pub_->publish(jv_msg);

    // ================================= EE linear & angular ==================================== //
    latest_joint_pos_ = q; // cache
    Eigen::MatrixXd Jg = dynamic_grasping_controller::geometricJacobian(latest_joint_pos_);
    Eigen::VectorXd ee_twist = Jg * Eigen::Map<Eigen::VectorXd>(joint_vel_avg.data(), 6);

    // Linear component from Jacobian regardless of source
    Eigen::Vector3d lin_raw = ee_twist.head<3>();

    // Decide source for angular
    Eigen::Vector3d ang_raw = Eigen::Vector3d::Zero();
    if (have_hw_vel) {
      ang_raw = ee_twist.tail<3>(); // measured q̇, so Jacobian is trustworthy
    } else {
      // quaternion differentiation fallback
      Eigen::VectorXd fk = dynamic_grasping_controller::forwardKinematics(q);
      Eigen::Quaterniond quat_curr(fk(6), fk(3), fk(4), fk(5)); // w, x, y, z order for constructor
      quat_curr.normalize();
      if (prev_quat_ && prev_time_) {
        const double dt = t - *prev_time_;
        if (dt > std::numeric_limits<double>::epsilon()) {
          Eigen::Quaterniond q_rel = prev_quat_->conjugate() * quat_curr;
          if (q_rel.w() < 0) q_rel.coeffs() *= -1.0; // shortest‑path
          ang_raw = 2.0 * q_rel.vec() / dt;
        }
      }
      prev_quat_ = quat_curr; // store for next diff
    }

    // ------------------------------- moving‑average filter (EE) -------------------------------- //
    Eigen::Vector3d lin_avg, ang_avg;
    for (int i = 0; i < 3; ++i) {
      auto &lbuf = ee_linear_buffers_[i];
      lbuf.push_back(lin_raw(i));
      if (lbuf.size() > window_size_) lbuf.pop_front();
      lin_avg(i) = std::accumulate(lbuf.begin(), lbuf.end(), 0.0) / lbuf.size();

      auto &abuf = ee_angular_buffers_[i];
      abuf.push_back(ang_raw(i));
      if (abuf.size() > window_size_) abuf.pop_front();
      ang_avg(i) = std::accumulate(abuf.begin(), abuf.end(), 0.0) / abuf.size();
    }

    // Publish twist (xyz + ω)
    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = now();
    twist_msg.twist.linear.x  = lin_avg.x();
    twist_msg.twist.linear.y  = lin_avg.y();
    twist_msg.twist.linear.z  = lin_avg.z();
    twist_msg.twist.angular.x = ang_avg.x();
    twist_msg.twist.angular.y = ang_avg.y();
    twist_msg.twist.angular.z = ang_avg.z();
    ee_velocity_pub_->publish(twist_msg);

    // Euler‑rate version (analysis Jacobian) – unchanged
    Eigen::MatrixXd Ja = dynamic_grasping_controller::analyticJacobian(q);
    Eigen::VectorXd ee_eul = Ja * Eigen::Map<Eigen::VectorXd>(joint_vel_avg.data(), 6);
    geometry_msgs::msg::TwistStamped eul_msg;
    eul_msg.header.stamp = now();
    if (ee_eul.size() >= 6) {
      eul_msg.twist.linear.x  = ee_eul(0);
      eul_msg.twist.linear.y  = ee_eul(1);
      eul_msg.twist.linear.z  = ee_eul(2);
      eul_msg.twist.angular.x = ee_eul(3);
      eul_msg.twist.angular.y = ee_eul(4);
      eul_msg.twist.angular.z = ee_eul(5);
    }
    ee_velocity_eul_pub_->publish(eul_msg);

    // Update finite‑difference state
    if (!have_hw_vel) { prev_time_ = t; prev_pos_ = q; }
    else { prev_time_.reset(); prev_pos_.reset(); } // reset diff state when hardware gives vel
  }

  void publishZeroes(const std::vector<std::string>& names)
  {
    sensor_msgs::msg::JointState zero;
    zero.header.stamp = now();
    zero.name = names;
    zero.velocity.assign(names.size(), 0.0);
    velocity_pub_->publish(zero);
  }

  // === ROS interfaces === //
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr     velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_velocity_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ee_velocity_eul_pub_;

  // === Buffers & state === //
  std::string hardware_type_;
  const size_t window_size_;

  std::vector<std::deque<double>> vel_buffers_;       // per‑joint
  std::vector<std::deque<double>> ee_linear_buffers_; // per‑axis linear
  std::vector<std::deque<double>> ee_angular_buffers_;// per‑axis angular

  std::optional<double>          prev_time_;
  std::optional<Eigen::VectorXd> prev_pos_;
  std::optional<Eigen::Quaterniond> prev_quat_;

  Eigen::VectorXd latest_joint_pos_;
  std::map<std::string, size_t> name_to_index_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VelocityPublisher>());
  rclcpp::shutdown();
  return 0;
}
