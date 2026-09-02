#include "hardware/imu/ism330.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace {

constexpr uint8_t kRegWhoAmI = 0x0f;
constexpr uint8_t kRegCtrl1Xl = 0x10;
constexpr uint8_t kRegCtrl2G = 0x11;
constexpr uint8_t kRegCtrl3C = 0x12;
constexpr uint8_t kRegOutxLG = 0x22;
constexpr uint8_t kCtrl3BduAndAutoIncrement = 0x44;
constexpr double kGravity = 9.81;

int16_t MakeInt16(uint8_t low, uint8_t high) {
  return static_cast<int16_t>(static_cast<uint16_t>(low) |
                              (static_cast<uint16_t>(high) << 8));
}

std::string ErrnoMessage(const std::string& action) {
  return action + ": " + std::strerror(errno);
}

}  // namespace

namespace hardware {
namespace imu {

Ism330::~Ism330() { Close(); }

void Ism330::Close() {
  if (file_descriptor_ >= 0) {
    close(file_descriptor_);
    file_descriptor_ = -1;
  }
}

void Ism330::Open(const Config& config) {
  Close();
  config_ = config;
  file_descriptor_ = open(config_.i2c_bus.c_str(), O_RDWR);
  if (file_descriptor_ < 0) {
    throw std::runtime_error(ErrnoMessage("Failed to open " + config_.i2c_bus));
  }

  if (ioctl(file_descriptor_, I2C_SLAVE, config_.i2c_address) < 0) {
    throw std::runtime_error(ErrnoMessage("Failed to select I2C address"));
  }

  accel_scale_ = AccelScale(config_.accel_range_g);
  gyro_scale_ = GyroScale(config_.gyro_range_dps);
  ConfigureSensor();
}

void Ism330::ConfigureSensor() {
  who_am_i_ = ReadRegister(kRegWhoAmI);
  if (config_.check_who_am_i && !IsExpectedDevice(who_am_i_)) {
    throw std::runtime_error("Unexpected ISM330 WHO_AM_I value 0x" +
                             std::to_string(who_am_i_));
  }

  const auto odr = SelectOdr(config_.output_data_rate_hz);
  configured_output_data_rate_hz_ = odr.hz;
  WriteRegister(kRegCtrl3C, kCtrl3BduAndAutoIncrement);
  WriteRegister(kRegCtrl1Xl, odr.bits | AccelRangeBits(config_.accel_range_g));
  WriteRegister(kRegCtrl2G, odr.bits | GyroRangeBits(config_.gyro_range_dps));
}

Ism330::Sample Ism330::Read() {
  std::array<uint8_t, 12> data{};
  ReadRegisters(kRegOutxLG, data.data(), data.size());

  const auto gyro_x = MakeInt16(data[0], data[1]);
  const auto gyro_y = MakeInt16(data[2], data[3]);
  const auto gyro_z = MakeInt16(data[4], data[5]);
  const auto accel_x = MakeInt16(data[6], data[7]);
  const auto accel_y = MakeInt16(data[8], data[9]);
  const auto accel_z = MakeInt16(data[10], data[11]);

  Sample sample;
  sample.angular_velocity_radps[0] = static_cast<double>(gyro_x) * gyro_scale_;
  sample.angular_velocity_radps[1] = static_cast<double>(gyro_y) * gyro_scale_;
  sample.angular_velocity_radps[2] = static_cast<double>(gyro_z) * gyro_scale_;
  sample.linear_acceleration_mps2[0] =
      static_cast<double>(accel_x) * accel_scale_;
  sample.linear_acceleration_mps2[1] =
      static_cast<double>(accel_y) * accel_scale_;
  sample.linear_acceleration_mps2[2] =
      static_cast<double>(accel_z) * accel_scale_;
  return sample;
}

uint8_t Ism330::ReadRegister(uint8_t reg) const {
  uint8_t value = 0;
  ReadRegisters(reg, &value, 1);
  return value;
}

void Ism330::ReadRegisters(uint8_t start_reg, uint8_t* data,
                           std::size_t length) const {
  if (write(file_descriptor_, &start_reg, 1) != 1) {
    throw std::runtime_error(ErrnoMessage("Failed to write I2C register"));
  }

  const auto bytes_read = read(file_descriptor_, data, length);
  if (bytes_read != static_cast<ssize_t>(length)) {
    throw std::runtime_error(ErrnoMessage("Failed to read I2C registers"));
  }
}

void Ism330::WriteRegister(uint8_t reg, uint8_t value) const {
  const std::array<uint8_t, 2> data{reg, value};
  if (write(file_descriptor_, data.data(), data.size()) !=
      static_cast<ssize_t>(data.size())) {
    throw std::runtime_error(ErrnoMessage("Failed to write I2C register"));
  }
}

Ism330::OdrConfig Ism330::SelectOdr(double requested_hz) const {
  const std::array<OdrConfig, 8> odrs{{
      {12.5, 0x10},
      {26.0, 0x20},
      {52.0, 0x30},
      {104.0, 0x40},
      {208.0, 0x50},
      {416.0, 0x60},
      {833.0, 0x70},
      {1660.0, 0x80},
  }};

  return *std::min_element(
      odrs.begin(), odrs.end(),
      [requested_hz](const OdrConfig& lhs, const OdrConfig& rhs) {
        return std::abs(lhs.hz - requested_hz) <
               std::abs(rhs.hz - requested_hz);
      });
}

uint8_t Ism330::AccelRangeBits(double range_g) const {
  if (range_g <= 2.0) {
    return 0x00;
  }
  if (range_g <= 4.0) {
    return 0x08;
  }
  if (range_g <= 8.0) {
    return 0x0c;
  }
  return 0x04;
}

uint8_t Ism330::GyroRangeBits(double range_dps) const {
  if (range_dps <= 250.0) {
    return 0x00;
  }
  if (range_dps <= 500.0) {
    return 0x04;
  }
  if (range_dps <= 1000.0) {
    return 0x08;
  }
  return 0x0c;
}

double Ism330::AccelScale(double range_g) const {
  if (range_g <= 2.0) {
    return 0.061e-3 * kGravity;
  }
  if (range_g <= 4.0) {
    return 0.122e-3 * kGravity;
  }
  if (range_g <= 8.0) {
    return 0.244e-3 * kGravity;
  }
  return 0.488e-3 * kGravity;
}

double Ism330::GyroScale(double range_dps) const {
  constexpr double deg_to_rad = M_PI / 180.0;
  if (range_dps <= 250.0) {
    return 8.75e-3 * deg_to_rad;
  }
  if (range_dps <= 500.0) {
    return 17.50e-3 * deg_to_rad;
  }
  if (range_dps <= 1000.0) {
    return 35.0e-3 * deg_to_rad;
  }
  return 70.0e-3 * deg_to_rad;
}

bool Ism330::IsExpectedDevice(uint8_t who_am_i) const {
  return std::find(
             config_.expected_who_am_i.begin(), config_.expected_who_am_i.end(),
             static_cast<int64_t>(who_am_i)) != config_.expected_who_am_i.end();
}

}  // namespace imu
}  // namespace hardware
