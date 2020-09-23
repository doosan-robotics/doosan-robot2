#ifndef FAKE_NODE_HPP_
#define FAKE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#define LEFT 0
#define RIGHT 1

class FakeNode : public rclcpp::Node
{
 public:
  FakeNode();
  ~FakeNode();
  
 private:
  // ROS time
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time prev_update_time_;

  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // ROS topic publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  sensor_msgs::msg::JointState joint_states_;

  double dsr_joint[6];

  double wheel_seperation_;
  double wheel_radius_;

  // Function prototypes
  void init_parameters();
  void init_variables();
  void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
  void update_callback();
  void update_joint_state();
};
#endif // TURTLEBOT3_FAKE_NODE_HPP_