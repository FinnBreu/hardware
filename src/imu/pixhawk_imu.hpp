#pragma once

#include <string>

#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace hardware {
namespace imu {

class PixhawkImu : public rclcpp::Node {
 public:
  explicit PixhawkImu(rclcpp::NodeOptions const& options);

 private:
  void OnSensorCombined(const px4_msgs::msg::SensorCombined::SharedPtr msg);

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  std::string frame_id_;
  std::string input_topic_;
  std::string output_topic_;
  double accel_stddev_{0.05};
  double gyro_stddev_{0.005};
};

}  // namespace imu
}  // namespace hardware
