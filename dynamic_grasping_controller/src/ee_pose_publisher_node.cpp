#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "dynamic_grasping_controller/widowx_kinematics_library.hpp"
#include "dynamic_grasping_controller/trajectory_utils.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <algorithm>

// This node subscribes to joint state messages from "/wx250s/joint_states"
// (sensor_msgs/msg/JointState) and publishes the computed end-effector pose
// on the "/wx250s/end_effector_pose" topic and with Euler angles on
// "/wx250s/end_effector_pose_euler" topic.

class EndEffectorPublisher : public rclcpp::Node
{
public:
  EndEffectorPublisher()
  : Node("end_effector_publisher"),
    joint_vector_(Eigen::VectorXd::Zero(6))  // Initialize a 6-DOF joint vector with zeros
  {
    // Create publishers for end-effector pose messages
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/wx250s/end_effector_pose", 10);
    publisher_euler_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/wx250s/end_effector_pose_euler", 10);

    // Subscribe to the joint state topic; using SensorDataQoS because joint states are high-frequency.
    // The topic now is "/wx250s/joint_states" with message type sensor_msgs/msg/JointState.
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/wx250s/joint_states", 
      rclcpp::SensorDataQoS(),
      std::bind(&EndEffectorPublisher::jointStatesCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "End Effector Publisher Node started.");
  }

private:
  // Callback function called every time a new JointState message is received.
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Define the fixed joint order expected by the forward kinematics algorithm.
    // This order must match the expected order in the kinematics library.
    const std::vector<std::string> joint_order = {
      "waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"
    };

    // Iterate over each joint in the expected order.
    for (size_t i = 0; i < joint_order.size(); ++i) {
      // Search for the joint name in the incoming message's name array.
      auto it = std::find(msg->name.begin(), msg->name.end(), joint_order[i]);
      if (it != msg->name.end()) {
        // Get the index of the found joint name.
        size_t idx = std::distance(msg->name.begin(), it);
        // Check if the position array has a valid element for this index.
        if (idx < msg->position.size()) {
          // Update the corresponding entry in the joint vector.
          joint_vector_(i) = msg->position[idx];
        } else {
          RCLCPP_WARN(this->get_logger(), "No position data for joint: %s", joint_order[i].c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Joint not found in message: %s", joint_order[i].c_str());
      }
    }

    // Compute both pose messages (quaternion and Euler)
    auto [pose_quat, pose_euler] = computeForwardKinematics();

    // Publish both computed pose messages
    publisher_->publish(pose_quat);
    publisher_euler_->publish(pose_euler);
  }

  // Computes the forward kinematics using the current joint vector.
  // Returns both quaternion and Euler angle representations.
  std::pair<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped> computeForwardKinematics()
  {
    geometry_msgs::msg::PoseStamped pose_quat;
    geometry_msgs::msg::PoseStamped pose_euler;
    
    // Set the header timestamp and frame id for both messages
    auto timestamp = this->now();
    pose_quat.header.stamp = timestamp;
    pose_quat.header.frame_id = "wx250s/base_link";
    pose_euler.header.stamp = timestamp;
    pose_euler.header.frame_id = "wx250s/base_link";

    // Compute forward kinematics. Expected output order:
    // [x, y, z, qx, qy, qz, qw]
    Eigen::VectorXd fk = dynamic_grasping_controller::forwardKinematics(joint_vector_);

    // Map the computed position for both messages
    pose_quat.pose.position.x = fk(0);
    pose_quat.pose.position.y = fk(1);
    pose_quat.pose.position.z = fk(2);
    pose_euler.pose.position.x = fk(0);
    pose_euler.pose.position.y = fk(1);
    pose_euler.pose.position.z = fk(2);

    // For quaternion message: use quaternion directly
    pose_quat.pose.orientation.x = fk(3);
    pose_quat.pose.orientation.y = fk(4);
    pose_quat.pose.orientation.z = fk(5);
    pose_quat.pose.orientation.w = fk(6);

    // For Euler message: convert quaternion to Euler angles
    Eigen::Quaterniond quat(fk(6), fk(3), fk(4), fk(5)); // w, x, y, z
    Eigen::Vector3d euler = dynamic_grasping_controller::quaternionToEuler(quat);
    
    // Store Euler angles in the orientation fields (repurposing them)
    pose_euler.pose.orientation.x = euler(0); // roll
    pose_euler.pose.orientation.y = euler(1); // pitch
    pose_euler.pose.orientation.z = euler(2); // yaw
    pose_euler.pose.orientation.w = 0.0;      // unused for Euler representation

    return {pose_quat, pose_euler};
  }

  // Member variables
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_euler_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  Eigen::VectorXd joint_vector_; // 6-DOF joint vector for forward kinematics
};

int main(int argc, char * argv[])
{
  // Initialize ROS 2 communication.
  rclcpp::init(argc, argv);
  // Create an instance of the EndEffectorPublisher node.
  auto node = std::make_shared<EndEffectorPublisher>();
  // Spin the node so it can process incoming messages and callbacks.
  rclcpp::spin(node);
  // Shutdown ROS 2 when done.
  rclcpp::shutdown();
  return 0;
}
