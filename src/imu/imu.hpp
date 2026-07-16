#pragma once

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "hardware/imu/ism330.hpp"

namespace hardware {
namespace imu {

class Imu : public rclcpp::Node {
 public:
  explicit Imu(rclcpp::NodeOptions const& options);

 private:
  struct Params {
    std::string i2c_bus{"/dev/i2c-1"};
    int i2c_address{0x6a};
    std::string frame_id{"imu_link"};
    std::string topic{"imu/data"};
    double publish_rate_hz{104.0};
    double accel_range_g{4.0};
    double gyro_range_dps{500.0};
    double accel_stddev{0.05};
    double gyro_stddev{0.005};
    bool check_who_am_i{true};
    std::vector<int64_t> expected_who_am_i{0x6a, 0x6b, 0x6c};
  };

  void InitParams();
  void InitPublishers();
  void InitTimers();
  void ReadAndPublish();
  sensor_msgs::msg::Imu BuildImuMsg(const Ism330::Sample& sample) const;
  Ism330::Config SensorConfig() const;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Ism330 sensor_;
  Params params_;
};

}  // namespace imu
}  // namespace hardware
