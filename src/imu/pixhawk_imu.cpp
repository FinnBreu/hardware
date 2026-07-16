#include "pixhawk_imu.hpp"

#include <string>

#include <rclcpp_components/register_node_macro.hpp>

namespace {

double Variance(double stddev) { return stddev * stddev; }

}  // namespace

namespace hardware {
namespace imu {

PixhawkImu::PixhawkImu(rclcpp::NodeOptions const& options)
    : rclcpp::Node("pixhawk_imu", options) {
  input_topic_ = declare_parameter<std::string>("input_topic",
                                                "fmu/out/sensor_combined");
  output_topic_ = declare_parameter<std::string>("output_topic", "imu/data");
  frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
  accel_stddev_ = declare_parameter<double>("accel_stddev", 0.05);
  gyro_stddev_ = declare_parameter<double>("gyro_stddev", 0.005);

  pub_ = create_publisher<sensor_msgs::msg::Imu>(output_topic_, rclcpp::QoS(10));
  sub_ = create_subscription<px4_msgs::msg::SensorCombined>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](px4_msgs::msg::SensorCombined::SharedPtr msg) {
        OnSensorCombined(msg);
      });

  RCLCPP_INFO(get_logger(), "Converting Pixhawk IMU from <%s> to <%s>",
              input_topic_.c_str(), output_topic_.c_str());
}

void PixhawkImu::OnSensorCombined(
    const px4_msgs::msg::SensorCombined::SharedPtr msg) {
  sensor_msgs::msg::Imu out;
  out.header.stamp = now();
  out.header.frame_id = frame_id_;

  out.orientation_covariance[0] = -1.0;

  out.angular_velocity.x = msg->gyro_rad[0];
  out.angular_velocity.y = -msg->gyro_rad[1];
  out.angular_velocity.z = -msg->gyro_rad[2];
  out.linear_acceleration.x = msg->accelerometer_m_s2[0];
  out.linear_acceleration.y = -msg->accelerometer_m_s2[1];
  out.linear_acceleration.z = -msg->accelerometer_m_s2[2];

  const auto gyro_variance = Variance(gyro_stddev_);
  const auto accel_variance = Variance(accel_stddev_);
  out.angular_velocity_covariance[0] = gyro_variance;
  out.angular_velocity_covariance[4] = gyro_variance;
  out.angular_velocity_covariance[8] = gyro_variance;
  out.linear_acceleration_covariance[0] = accel_variance;
  out.linear_acceleration_covariance[4] = accel_variance;
  out.linear_acceleration_covariance[8] = accel_variance;

  pub_->publish(out);
}

}  // namespace imu
}  // namespace hardware

RCLCPP_COMPONENTS_REGISTER_NODE(hardware::imu::PixhawkImu)
