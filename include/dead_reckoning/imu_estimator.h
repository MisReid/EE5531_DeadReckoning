#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class IMUEstimator : public rclcpp::Node {
  public:
    IMUEstimator();
  private:
    void Update(const sensor_msgs::msg::Imu::SharedPtr msg);
    float t;
    float theta;
    float x;
    float y;
    float vx;
    float vy;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};


