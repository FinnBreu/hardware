#include "pixhawk_imu.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hardware::imu::PixhawkImu>(
      rclcpp::NodeOptions().use_intra_process_comms(true));
  rclcpp::executors::StaticSingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  exec.remove_node(node);
}
