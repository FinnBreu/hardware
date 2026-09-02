#include "imu.hpp"

#include <algorithm>
#include <chrono>
#include <rclcpp_components/register_node_macro.hpp>
#include <stdexcept>

namespace {

double Variance(double stddev) { return stddev * stddev; }

}  // namespace

namespace hardware {
namespace imu {

Imu::Imu(rclcpp::NodeOptions const& options) : Node("imu", options) {
  InitParams();
  InitPublishers();
  InitTimers();
}

void Imu::InitParams() {
  params_.i2c_bus = declare_parameter<std::string>("i2c_bus", params_.i2c_bus);
  params_.i2c_address =
      declare_parameter<int>("i2c_address", params_.i2c_address);
  params_.frame_id =
      declare_parameter<std::string>("frame_id", params_.frame_id);
  params_.topic = declare_parameter<std::string>("topic", params_.topic);
  params_.publish_rate_hz =
      declare_parameter<double>("publish_rate_hz", params_.publish_rate_hz);
  params_.accel_range_g =
      declare_parameter<double>("accel_range_g", params_.accel_range_g);
  params_.gyro_range_dps =
      declare_parameter<double>("gyro_range_dps", params_.gyro_range_dps);
  params_.accel_stddev =
      declare_parameter<double>("accel_stddev", params_.accel_stddev);
  params_.gyro_stddev =
      declare_parameter<double>("gyro_stddev", params_.gyro_stddev);
  params_.check_who_am_i =
      declare_parameter<bool>("check_who_am_i", params_.check_who_am_i);
  params_.expected_who_am_i = declare_parameter<std::vector<int64_t>>(
      "expected_who_am_i", params_.expected_who_am_i);
}

void Imu::InitPublishers() {
  imu_pub_ =
      create_publisher<sensor_msgs::msg::Imu>(params_.topic, rclcpp::QoS(10));
}

void Imu::InitTimers() {
  const auto period = std::chrono::duration<double>(
      1.0 / std::max(params_.publish_rate_hz, 1.0));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { ReadAndPublish(); });
}

bool Imu::EnsureSensorInitialized() {
  if (sensor_initialized_) {
    return true;
  }

  try {
    sensor_.Open(SensorConfig());
  } catch (const std::exception& exception) {
    if (!sensor_init_failure_logged_) {
      RCLCPP_ERROR(
          get_logger(), "Could not initialize ISM330 on %s at 0x%02x: %s",
          params_.i2c_bus.c_str(), params_.i2c_address, exception.what());
      sensor_init_failure_logged_ = true;
    }
    return false;
  }

  RCLCPP_INFO(
      get_logger(),
      "Initialized ISM330 on %s at 0x%02x, WHO_AM_I=0x%02x, ODR=%.1f Hz",
      params_.i2c_bus.c_str(), params_.i2c_address, sensor_.who_am_i(),
      sensor_.configured_output_data_rate_hz());
  sensor_initialized_ = true;
  sensor_init_failure_logged_ = false;
  return true;
}

void Imu::ReadAndPublish() {
  if (!EnsureSensorInitialized()) {
    return;
  }

  try {
    imu_pub_->publish(BuildImuMsg(sensor_.Read()));
  } catch (const std::exception& exception) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed to read ISM330 sample: %s", exception.what());
    sensor_initialized_ = false;
    sensor_init_failure_logged_ = false;
  }
}

sensor_msgs::msg::Imu Imu::BuildImuMsg(const Ism330::Sample& sample) const {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = now();
  msg.header.frame_id = params_.frame_id;
  msg.orientation_covariance[0] = -1.0;

  msg.angular_velocity.x = sample.angular_velocity_radps[0];
  msg.angular_velocity.y = sample.angular_velocity_radps[1];
  msg.angular_velocity.z = sample.angular_velocity_radps[2];
  msg.linear_acceleration.x = sample.linear_acceleration_mps2[0];
  msg.linear_acceleration.y = sample.linear_acceleration_mps2[1];
  msg.linear_acceleration.z = sample.linear_acceleration_mps2[2];

  const auto gyro_variance = Variance(params_.gyro_stddev);
  const auto accel_variance = Variance(params_.accel_stddev);
  msg.angular_velocity_covariance[0] = gyro_variance;
  msg.angular_velocity_covariance[4] = gyro_variance;
  msg.angular_velocity_covariance[8] = gyro_variance;
  msg.linear_acceleration_covariance[0] = accel_variance;
  msg.linear_acceleration_covariance[4] = accel_variance;
  msg.linear_acceleration_covariance[8] = accel_variance;
  return msg;
}

Ism330::Config Imu::SensorConfig() const {
  Ism330::Config config;
  config.i2c_bus = params_.i2c_bus;
  config.i2c_address = params_.i2c_address;
  config.output_data_rate_hz = params_.publish_rate_hz;
  config.accel_range_g = params_.accel_range_g;
  config.gyro_range_dps = params_.gyro_range_dps;
  config.check_who_am_i = params_.check_who_am_i;
  config.expected_who_am_i = params_.expected_who_am_i;
  return config;
}

}  // namespace imu
}  // namespace hardware

RCLCPP_COMPONENTS_REGISTER_NODE(hardware::imu::Imu)
