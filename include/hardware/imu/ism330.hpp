#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hardware {
namespace imu {

class Ism330 {
 public:
  struct Config {
    std::string i2c_bus{"/dev/i2c-1"};
    int i2c_address{0x6a};
    double output_data_rate_hz{104.0};
    double accel_range_g{4.0};
    double gyro_range_dps{500.0};
    bool check_who_am_i{true};
    std::vector<int64_t> expected_who_am_i{0x6a, 0x6b, 0x6c};
  };

  struct Sample {
    double angular_velocity_radps[3]{};
    double linear_acceleration_mps2[3]{};
  };

  struct OdrConfig {
    double hz;
    uint8_t bits;
  };

  Ism330() = default;
  ~Ism330();

  void Open(const Config& config);
  void Close();
  Sample Read();
  double configured_output_data_rate_hz() const {
    return configured_output_data_rate_hz_;
  }
  uint8_t who_am_i() const { return who_am_i_; }

 private:
  void ConfigureSensor();
  uint8_t ReadRegister(uint8_t reg) const;
  void ReadRegisters(uint8_t start_reg, uint8_t* data,
                     std::size_t length) const;
  void WriteRegister(uint8_t reg, uint8_t value) const;
  OdrConfig SelectOdr(double requested_hz) const;
  uint8_t AccelRangeBits(double range_g) const;
  uint8_t GyroRangeBits(double range_dps) const;
  double AccelScale(double range_g) const;
  double GyroScale(double range_dps) const;
  bool IsExpectedDevice(uint8_t who_am_i) const;

  Config config_;
  int file_descriptor_{-1};
  double accel_scale_{0.0};
  double gyro_scale_{0.0};
  double configured_output_data_rate_hz_{0.0};
  uint8_t who_am_i_{0};
};

}  // namespace imu
}  // namespace hardware
