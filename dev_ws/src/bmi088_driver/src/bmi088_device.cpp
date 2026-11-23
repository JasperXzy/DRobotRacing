#include "bmi088_driver/bmi088_device.hpp"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace bmi088_driver
{
namespace
{

constexpr uint8_t kSpiReadMask = 0x80U;
constexpr uint8_t kSpiWriteMask = 0x7FU;
constexpr uint32_t kMaximumSpiSpeedHz = 10000000U;

constexpr uint8_t kAccelChipIdRegister = 0x00U;
constexpr uint8_t kAccelExpectedChipId = 0x1EU;
constexpr uint8_t kAccelDataRegister = 0x12U;
constexpr uint8_t kAccelConfigRegister = 0x40U;
constexpr uint8_t kAccelRangeRegister = 0x41U;
constexpr uint8_t kAccelTemperatureRegister = 0x22U;
constexpr uint8_t kAccelPowerConfigRegister = 0x7CU;
constexpr uint8_t kAccelPowerControlRegister = 0x7DU;
constexpr uint8_t kAccelSoftResetRegister = 0x7EU;

constexpr uint8_t kGyroChipIdRegister = 0x00U;
constexpr uint8_t kGyroExpectedChipId = 0x0FU;
constexpr uint8_t kGyroDataRegister = 0x02U;
constexpr uint8_t kGyroRangeRegister = 0x0FU;
constexpr uint8_t kGyroBandwidthRegister = 0x10U;
constexpr uint8_t kGyroPowerRegister = 0x11U;
constexpr uint8_t kGyroSoftResetRegister = 0x14U;

constexpr uint8_t kSoftResetCommand = 0xB6U;
constexpr double kStandardGravity = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;

std::string error_message(const std::string & operation, const std::string & path)
{
  std::ostringstream stream;
  stream << operation << " " << path << ": " << std::strerror(errno);
  return stream.str();
}

int16_t decode_int16(uint8_t lsb, uint8_t msb)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(lsb) | (static_cast<uint16_t>(msb) << 8U));
}

std::string hex_byte(uint8_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') <<
    static_cast<int>(value);
  return stream.str();
}

void sleep_ms(int milliseconds)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

}  // namespace

SpiDevice::SpiDevice(const std::string & path, uint32_t speed_hz)
: path_(path), speed_hz_(speed_hz)
{
  fd_ = ::open(path_.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw std::runtime_error(error_message("Unable to open SPI device", path_));
  }

  uint8_t mode = SPI_MODE_0;
  uint8_t bits_per_word = 8U;
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0)
  {
    const std::string message = error_message("Unable to configure SPI device", path_);
    close_fd();
    throw std::runtime_error(message);
  }
}

SpiDevice::~SpiDevice()
{
  close_fd();
}

SpiDevice::SpiDevice(SpiDevice && other) noexcept
: path_(std::move(other.path_)), speed_hz_(other.speed_hz_), fd_(other.fd_)
{
  other.fd_ = -1;
}

SpiDevice & SpiDevice::operator=(SpiDevice && other) noexcept
{
  if (this != &other) {
    close_fd();
    path_ = std::move(other.path_);
    speed_hz_ = other.speed_hz_;
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void SpiDevice::close_fd() noexcept
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::vector<uint8_t> SpiDevice::transfer(const std::vector<uint8_t> & tx) const
{
  if (fd_ < 0) {
    throw std::runtime_error("SPI device is closed: " + path_);
  }
  if (tx.empty()) {
    return {};
  }

  std::vector<uint8_t> rx(tx.size(), 0U);
  spi_ioc_transfer transfer{};
  transfer.tx_buf = reinterpret_cast<unsigned long>(tx.data());
  transfer.rx_buf = reinterpret_cast<unsigned long>(rx.data());
  transfer.len = static_cast<uint32_t>(tx.size());
  transfer.speed_hz = speed_hz_;
  transfer.bits_per_word = 8U;
  transfer.cs_change = 0U;

  const int result = ::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer);
  if (result < 0 || static_cast<std::size_t>(result) != tx.size()) {
    throw std::runtime_error(error_message("SPI transfer failed on", path_));
  }
  return rx;
}

Bmi088Device::Bmi088Device(Bmi088Config config)
: config_(std::move(config))
{
  validate_config();
}

void Bmi088Device::validate_config() const
{
  if (config_.accel_device.empty() || config_.gyro_device.empty()) {
    throw std::invalid_argument("SPI device paths must not be empty");
  }
  if (config_.accel_device == config_.gyro_device) {
    throw std::invalid_argument("BMI088 accelerometer and gyroscope require separate chip selects");
  }
  if (config_.spi_speed_hz == 0U || config_.spi_speed_hz > kMaximumSpiSpeedHz) {
    throw std::invalid_argument("spi_speed_hz must be between 1 and 10000000");
  }
  (void)accel_range_code();
  (void)accel_odr_code();
  (void)accel_bandwidth_code();
  (void)gyro_range_code();
  (void)gyro_bandwidth_code();
}

uint8_t Bmi088Device::accel_range_code() const
{
  switch (config_.accel_range_g) {
    case 3: return 0x00U;
    case 6: return 0x01U;
    case 12: return 0x02U;
    case 24: return 0x03U;
    default: throw std::invalid_argument("accel_range_g must be one of 3, 6, 12, 24");
  }
}

uint8_t Bmi088Device::accel_odr_code() const
{
  switch (config_.accel_odr_hz) {
    case 25: return 0x06U;
    case 50: return 0x07U;
    case 100: return 0x08U;
    case 200: return 0x09U;
    case 400: return 0x0AU;
    case 800: return 0x0BU;
    case 1600: return 0x0CU;
    default:
      throw std::invalid_argument(
              "accel_odr_hz must be one of 25, 50, 100, 200, 400, 800, 1600");
  }
}

uint8_t Bmi088Device::accel_bandwidth_code() const
{
  if (config_.accel_bandwidth == "osr4") {
    return 0x08U;
  }
  if (config_.accel_bandwidth == "osr2") {
    return 0x09U;
  }
  if (config_.accel_bandwidth == "normal") {
    return 0x0AU;
  }
  throw std::invalid_argument("accel_bandwidth must be osr4, osr2, or normal");
}

uint8_t Bmi088Device::gyro_range_code() const
{
  switch (config_.gyro_range_dps) {
    case 2000: return 0x00U;
    case 1000: return 0x01U;
    case 500: return 0x02U;
    case 250: return 0x03U;
    case 125: return 0x04U;
    default: throw std::invalid_argument("gyro_range_dps must be one of 125, 250, 500, 1000, 2000");
  }
}

uint8_t Bmi088Device::gyro_bandwidth_code() const
{
  if (config_.gyro_odr_hz == 2000 && config_.gyro_bandwidth_hz == 532) {
    return 0x00U;
  }
  if (config_.gyro_odr_hz == 2000 && config_.gyro_bandwidth_hz == 230) {
    return 0x01U;
  }
  if (config_.gyro_odr_hz == 1000 && config_.gyro_bandwidth_hz == 116) {
    return 0x02U;
  }
  if (config_.gyro_odr_hz == 400 && config_.gyro_bandwidth_hz == 47) {
    return 0x03U;
  }
  if (config_.gyro_odr_hz == 200 && config_.gyro_bandwidth_hz == 23) {
    return 0x04U;
  }
  if (config_.gyro_odr_hz == 100 && config_.gyro_bandwidth_hz == 12) {
    return 0x05U;
  }
  if (config_.gyro_odr_hz == 200 && config_.gyro_bandwidth_hz == 64) {
    return 0x06U;
  }
  if (config_.gyro_odr_hz == 100 && config_.gyro_bandwidth_hz == 32) {
    return 0x07U;
  }
  throw std::invalid_argument("Unsupported gyro ODR/bandwidth pair");
}

std::vector<uint8_t> Bmi088Device::read_accel(uint8_t reg, std::size_t length) const
{
  if (!accel_spi_) {
    throw std::runtime_error("Accelerometer SPI device is not initialized");
  }
  // BMI088 accelerometer SPI reads return one extra unpredictable dummy byte.
  std::vector<uint8_t> tx(length + 2U, 0U);
  tx[0] = static_cast<uint8_t>(reg | kSpiReadMask);
  const auto rx = accel_spi_->transfer(tx);
  return std::vector<uint8_t>(rx.begin() + 2, rx.end());
}

std::vector<uint8_t> Bmi088Device::read_gyro(uint8_t reg, std::size_t length) const
{
  if (!gyro_spi_) {
    throw std::runtime_error("Gyroscope SPI device is not initialized");
  }
  std::vector<uint8_t> tx(length + 1U, 0U);
  tx[0] = static_cast<uint8_t>(reg | kSpiReadMask);
  const auto rx = gyro_spi_->transfer(tx);
  return std::vector<uint8_t>(rx.begin() + 1, rx.end());
}

void Bmi088Device::write_accel(uint8_t reg, uint8_t value) const
{
  if (!accel_spi_) {
    throw std::runtime_error("Accelerometer SPI device is not initialized");
  }
  (void)accel_spi_->transfer({static_cast<uint8_t>(reg & kSpiWriteMask), value});
  std::this_thread::sleep_for(std::chrono::microseconds(2));
}

void Bmi088Device::write_gyro(uint8_t reg, uint8_t value) const
{
  if (!gyro_spi_) {
    throw std::runtime_error("Gyroscope SPI device is not initialized");
  }
  (void)gyro_spi_->transfer({static_cast<uint8_t>(reg & kSpiWriteMask), value});
  std::this_thread::sleep_for(std::chrono::microseconds(2));
}

void Bmi088Device::initialize()
{
  accel_spi_ = std::make_unique<SpiDevice>(config_.accel_device, config_.spi_speed_hz);
  gyro_spi_ = std::make_unique<SpiDevice>(config_.gyro_device, config_.spi_speed_hz);

  // A CS1 rising edge is required after power-up to switch the accelerometer to SPI mode.
  (void)read_accel(kAccelChipIdRegister, 1U);
  sleep_ms(1);

  write_accel(kAccelSoftResetRegister, kSoftResetCommand);
  sleep_ms(1);
  // Soft reset returns the accelerometer interface to I2C mode; switch it back to SPI.
  (void)read_accel(kAccelChipIdRegister, 1U);
  sleep_ms(1);

  write_gyro(kGyroSoftResetRegister, kSoftResetCommand);
  sleep_ms(30);

  accel_chip_id_ = read_accel(kAccelChipIdRegister, 1U).at(0);
  gyro_chip_id_ = read_gyro(kGyroChipIdRegister, 1U).at(0);
  if (accel_chip_id_ != kAccelExpectedChipId || gyro_chip_id_ != kGyroExpectedChipId) {
    std::ostringstream stream;
    stream << "BMI088 chip ID mismatch: accel=" << hex_byte(accel_chip_id_) <<
      " (expected 0x1E), gyro=" << hex_byte(gyro_chip_id_) << " (expected 0x0F). " <<
      "Check CS1/CS2 wiring and spidev paths.";
    throw std::runtime_error(stream.str());
  }

  // Bosch's reference sequence writes PWR_CONF, waits 5 ms, then enables PWR_CTRL.
  write_accel(kAccelPowerConfigRegister, 0x00U);
  sleep_ms(5);
  write_accel(kAccelPowerControlRegister, 0x04U);
  sleep_ms(5);

  const uint8_t accel_config = static_cast<uint8_t>(
    (accel_bandwidth_code() << 4U) | accel_odr_code());
  write_accel(kAccelConfigRegister, accel_config);
  sleep_ms(40);
  write_accel(kAccelRangeRegister, accel_range_code());
  sleep_ms(5);

  write_gyro(kGyroPowerRegister, 0x00U);
  sleep_ms(30);
  write_gyro(kGyroBandwidthRegister, gyro_bandwidth_code());
  sleep_ms(10);
  write_gyro(kGyroRangeRegister, gyro_range_code());
  sleep_ms(10);

  verify_configuration();
}

void Bmi088Device::verify_configuration() const
{
  const uint8_t expected_accel_config = static_cast<uint8_t>(
    (accel_bandwidth_code() << 4U) | accel_odr_code());
  const uint8_t actual_accel_config = read_accel(kAccelConfigRegister, 1U).at(0);
  const uint8_t actual_accel_range = read_accel(kAccelRangeRegister, 1U).at(0);
  const uint8_t actual_gyro_bandwidth = read_gyro(kGyroBandwidthRegister, 1U).at(0);
  const uint8_t actual_gyro_range = read_gyro(kGyroRangeRegister, 1U).at(0);

  if ((actual_accel_config & 0xFFU) != expected_accel_config ||
    (actual_accel_range & 0x03U) != accel_range_code() ||
    (actual_gyro_bandwidth & 0x0FU) != gyro_bandwidth_code() ||
    (actual_gyro_range & 0x07U) != gyro_range_code())
  {
    std::ostringstream stream;
    stream << "BMI088 configuration readback failed: accel_conf=" <<
      hex_byte(actual_accel_config) << ", accel_range=" << hex_byte(actual_accel_range) <<
      ", gyro_bw=" << hex_byte(actual_gyro_bandwidth) <<
      ", gyro_range=" << hex_byte(actual_gyro_range);
    throw std::runtime_error(stream.str());
  }
}

RawSample Bmi088Device::read_sample()
{
  // Read acceleration and the adjacent 24-bit sensor time in one coherent burst.
  const auto accel_data = read_accel(kAccelDataRegister, 9U);
  const auto gyro_data = read_gyro(kGyroDataRegister, 6U);

  RawSample sample;
  sample.accel = {
    decode_int16(accel_data[0], accel_data[1]),
    decode_int16(accel_data[2], accel_data[3]),
    decode_int16(accel_data[4], accel_data[5])};
  sample.gyro = {
    decode_int16(gyro_data[0], gyro_data[1]),
    decode_int16(gyro_data[2], gyro_data[3]),
    decode_int16(gyro_data[4], gyro_data[5])};
  sample.accel_sensor_time = static_cast<uint32_t>(accel_data[6]) |
    (static_cast<uint32_t>(accel_data[7]) << 8U) |
    (static_cast<uint32_t>(accel_data[8]) << 16U);
  return sample;
}

double Bmi088Device::read_temperature_c()
{
  const auto data = read_accel(kAccelTemperatureRegister, 2U);
  return decode_temperature_c(data[0], data[1]);
}

double Bmi088Device::accel_sensitivity_lsb_per_g(int range_g)
{
  switch (range_g) {
    case 3: return 10920.0;
    case 6: return 5460.0;
    case 12: return 2730.0;
    case 24: return 1365.0;
    default: throw std::invalid_argument("Unsupported accelerometer range");
  }
}

double Bmi088Device::gyro_sensitivity_lsb_per_dps(int range_dps)
{
  switch (range_dps) {
    case 2000: return 16.384;
    case 1000: return 32.768;
    case 500: return 65.536;
    case 250: return 131.072;
    case 125: return 262.144;
    default: throw std::invalid_argument("Unsupported gyroscope range");
  }
}

ScaledSample Bmi088Device::scale_sample(const RawSample & raw) const
{
  ScaledSample scaled;
  const double accel_scale = kStandardGravity /
    accel_sensitivity_lsb_per_g(config_.accel_range_g);
  const double gyro_scale = kDegreesToRadians /
    gyro_sensitivity_lsb_per_dps(config_.gyro_range_dps);
  for (std::size_t axis = 0; axis < 3U; ++axis) {
    scaled.acceleration_mps2[axis] = static_cast<double>(raw.accel[axis]) * accel_scale;
    scaled.angular_velocity_rps[axis] = static_cast<double>(raw.gyro[axis]) * gyro_scale;
  }
  return scaled;
}

double Bmi088Device::decode_temperature_c(uint8_t msb, uint8_t lsb)
{
  int16_t raw = static_cast<int16_t>(
    (static_cast<uint16_t>(msb) << 3U) | (static_cast<uint16_t>(lsb) >> 5U));
  if (raw > 1023) {
    raw = static_cast<int16_t>(raw - 2048);
  }
  return 23.0 + static_cast<double>(raw) * 0.125;
}

}  // namespace bmi088_driver
