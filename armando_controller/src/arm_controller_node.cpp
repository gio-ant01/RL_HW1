#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArmControllerNode : public rclcpp::Node
{
public:
  ArmControllerNode() : Node("arm_controller_node"), command_index_(0)
  {
    // Declare and get the controller_type parameter
    this->declare_parameter<std::string>("controller_type", "position");
    this->get_parameter("controller_type", controller_type_);

    RCLCPP_INFO(this->get_logger(), "Using controller type: %s", controller_type_.c_str());

    // Validate controller type
    if (controller_type_ != "position" && controller_type_ != "trajectory") {
      RCLCPP_ERROR(this->get_logger(), 
        "Invalid controller_type '%s'. Must be 'position' or 'trajectory'", 
        controller_type_.c_str());
      rclcpp::shutdown();
      return;
    }

    // Subscriber to joint_states topic
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      10,
      std::bind(&ArmControllerNode::jointStateCallback, this, _1));

    // Create appropriate publisher based on controller type
    if (controller_type_ == "position") {
      position_command_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands",
        10);
      RCLCPP_INFO(this->get_logger(), "Position controller publisher created");
    } else {
      trajectory_command_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory",
        10);
      RCLCPP_INFO(this->get_logger(), "Trajectory controller publisher created");
    }

    // Define at least 4 different position commands (in radians)
    position_commands_ = {
      {0.0, 0.25, 0.25, 0.25},      // Position 1
      {0.0, 0.5, 0.5, 0.5},         // Position 2
      {0.0, 1.0, 1.0, 0.7},         // Position 3
      {0.0, 0.8, 0.8, 0.8},         // Position 4
      {0.0, 0.25, 0.25, 0.25}       // Back to Position 1
    };

    // Timer to publish commands sequentially every 5 seconds
    timer_ = this->create_wall_timer(
      8s,
      std::bind(&ArmControllerNode::publishNextCommand, this));

    RCLCPP_INFO(this->get_logger(), "Arm Controller Node initialized");
    RCLCPP_INFO(this->get_logger(), "Waiting for joint_states...");
  }

private:
  // Callback function that prints current joint positions
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() >= 4) {
      RCLCPP_INFO(this->get_logger(), 
        "Current Joint Positions - j0: %.3f, j1: %.3f, j2: %.3f, j3: %.3f",
        msg->position[0], msg->position[1], msg->position[2], msg->position[3]);
    } else {
      RCLCPP_WARN(this->get_logger(), "Received joint state with less than 4 joints");
    }
  }

  // Function to publish position commands sequentially
  void publishNextCommand()
  {
    if (command_index_ < position_commands_.size()) {
      if (controller_type_ == "position") {
        publishPositionCommand(position_commands_[command_index_]);
      } else {
        publishTrajectoryCommand(position_commands_[command_index_]);
      }
      command_index_++;
    } else {
      RCLCPP_INFO(this->get_logger(), "All commands sent. Repeating from the start...");
      command_index_ = 0;
    }
  }

  // Publish command for position controller
  void publishPositionCommand(const std::vector<double>& positions)
  {
    auto command_msg = std_msgs::msg::Float64MultiArray();
    command_msg.data = positions;

    RCLCPP_INFO(this->get_logger(), 
      "[POSITION] Publishing command %zu: [%.3f, %.3f, %.3f, %.3f]",
      command_index_ + 1,
      positions[0], positions[1], positions[2], positions[3]);

    position_command_publisher_->publish(command_msg);
  }

  // Publish command for trajectory controller
  void publishTrajectoryCommand(const std::vector<double>& positions)
  {
    auto trajectory_msg = trajectory_msgs::msg::JointTrajectory();
    
    // Set header
    trajectory_msg.header.stamp = rclcpp::Time(0);  // Cambiato da this->now() a Time(0)
    trajectory_msg.header.frame_id = "";  // Aggiungi frame_id vuoto
    
    // Set joint names
    trajectory_msg.joint_names = {"j0", "j1", "j2", "j3"};
    
    // Create trajectory point
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = positions;
    point.velocities = {0.0, 0.0, 0.0, 0.0};
    point.time_from_start = rclcpp::Duration(2, 0);  // 2 seconds to reach target
    
    trajectory_msg.points.push_back(point);

    RCLCPP_INFO(this->get_logger(), 
      "[TRAJECTORY] Publishing command %zu: [%.3f, %.3f, %.3f, %.3f]",
      command_index_ + 1,
      positions[0], positions[1], positions[2], positions[3]);

    trajectory_command_publisher_->publish(trajectory_msg);
  }

  // Class members
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr position_command_publisher_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  std::string controller_type_;
  std::vector<std::vector<double>> position_commands_;
  size_t command_index_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}