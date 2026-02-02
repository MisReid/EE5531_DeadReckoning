#include "dead_reckoning/dead_reckoner.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

DeadReckoner::DeadReckoner() : Node("dead_reckoner") {
  this->theta = 0;
  this->x = 0;
  this->y = 0;
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
		"/cmd_vel", 10, std::bind(&DeadReckoner::Update, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
}

// v = linear.x
// w = angular.z
void DeadReckoner::Update(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  int sec = msg->header.stamp.sec;
  int nsec = msg->header.stamp.nanosec;
  float v = msg->twist.linear.x;
  float w = msg->twist.angular.z;
  RCLCPP_INFO(this->get_logger(), "Received cmd_vel: v=%f, w=%f", v, w);

  int dt = (sec + nsec * 1e-9) - t;
  this->t += dt;
  this->theta += w * dt;
  this->x += v * std::cos(theta) * dt;
  this->y += v * std::sin(theta) * dt;
  RCLCPP_INFO(this->get_logger(), "Updated Pose: theta=%f, x=%f, y=%f", this->theta, this->x, this->y);

  auto odom_msg = nav_msgs::msg::Odometry();	
  auto path_msg = nav_msgs::msg::Path();
  static auto poses = std::vector<geometry_msgs::msg::PoseStamped>();

  // Pose message (used in both)
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "odom";
  pose_msg.pose.position.x = this->x;
  pose_msg.pose.position.y = this->y;
  pose_msg.pose.orientation.z = std::sin(this->theta / 2);
  pose_msg.pose.orientation.w = std::cos(this->theta / 2);
  poses.push_back(pose_msg);

  // Odom message
  odom_msg.header.stamp = this->now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose = pose_msg.pose;
  odom_pub_->publish(odom_msg);

  // Path message
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "odom";
  path_msg.poses = poses;
  path_pub_->publish(path_msg);

}

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DeadReckoner>());
	rclcpp::shutdown();
	return 0;
}
