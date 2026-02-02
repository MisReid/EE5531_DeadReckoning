#include <cmath>
#include "dead_reckoning/imu_estimator.h"

IMUEstimator::IMUEstimator() : Node("imu_estimator") {
	this->theta = 0;
	this->vx = 0;
	this->vy = 0;
	this->x = 0;
	this->y = 0;
	this->t = this->now().seconds();
	imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
		"/imu", 10, std::bind(&IMUEstimator::Update, this, std::placeholders::_1));
	odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/imu_integration/odom", 10);
	path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/imu_integration/path", 10);
}

void IMUEstimator::Update(const sensor_msgs::msg::Imu::SharedPtr msg) {
  int sec = msg->header.stamp.sec;
  int nsec = msg->header.stamp.nanosec;
  float ax = msg->linear_acceleration.x;
  float ay = msg->linear_acceleration.y;
  float wz = msg->angular_velocity.z;
  RCLCPP_INFO(this->get_logger(), "Received IMU data: ax=%f, ay=%f, wz=%f", ax, ay, wz);

  float ax_world = ax * std::cos(theta) - ay * std::sin(theta);
  float ay_world = ax * std::sin(theta) + ay * std::cos(theta);
  RCLCPP_INFO(this->get_logger(), "Conversion done: ax=%f, ay=%f", ax_world, ay_world);

  float dt = (sec + nsec * 1e-9) - t;
  this->t += dt;
  this->theta += wz * dt;
  this->vx += ax_world * dt;
  this->vy += ay_world * dt;
  this->x += this->vx * dt;
  this->y += this->vy * dt;
  RCLCPP_INFO(this->get_logger(), "Updated Pose: theta=%f, x=%f, y=%f (vx=%f, vy=%f, dt=%f)", this->theta, this->x, this->y, this->vx, this->vy, dt);

  auto odom_msg = nav_msgs::msg::Odometry();	
  auto path_msg = nav_msgs::msg::Path();
  static auto poses = std::vector<geometry_msgs::msg::PoseStamped>();

  // Pose message (used in both)
  auto pose_msg = geometry_msgs::msg::PoseStamped();
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "odom";
  pose_msg.pose.position.x = this->x;
  pose_msg.pose.position.y = this->y;
  pose_msg.pose.orientation = msg->orientation;
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
	rclcpp::spin(std::make_shared<IMUEstimator>());
	rclcpp::shutdown();
	return 0;
}
