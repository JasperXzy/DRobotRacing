#ifndef BMI088_DRIVER__BMI088_DEVICE_HPP_
#define BMI088_DRIVER__BMI088_DEVICE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace bmi088_driver
{

struct Bmi088Config
{
  std::string accel_device{"/dev/spidev0.0"};
  std::string gyro_device{"/dev/spidev0.1"};
  uint32_t spi_speed_hz{5000000U};
  int accel_range_g{3};
  int accel_odr_hz{200};
  std::string accel_bandwidth{"osr4"};
  int gyro_range_dps{500};
  int gyro_odr_hz{200};
  int gyro_bandwidth_hz{64};
};

struct RawSample
{
  std::array<int16_t, 3> accel{};
  std::array<int16_t, 3> gyro{};
  uint32_t accel_sensor_time{0U};
};

struct ScaledSample
{
  std::array<double, 3> acceleration_mps2{};
  std::array<double, 3> angular_velocity_rps{};
};

class SpiDevice
{
public:
  SpiDevice(const std::string & path, uint32_t speed_hz);
  ~SpiDevice();

  SpiDevice(const SpiDevice &) = delete;
  SpiDevice & operator=(const SpiDevice &) = delete;
  SpiDevice(SpiDevice && other) noexcept;
  SpiDevice & operator=(SpiDevice && other) noexcept;

  std::vector<uint8_t> transfer(const std::vector<uint8_t> & tx) const;
  const std::string & path() const noexcept {return path_;}

private:
  void close_fd() noexcept;

  std::string path_;
  uint32_t speed_hz_;
  int fd_{-1};
};

class Bmi088Device
{
public:
  explicit Bmi088Device(Bmi088Config config);

  void initialize();
  RawSample read_sample();
  double read_temperature_c();

  uint8_t accel_chip_id() const noexcept {return accel_chip_id_;}
  uint8_t gyro_chip_id() const noexcept {return gyro_chip_id_;}
  const Bmi088Config & config() const noexcept {return config_;}

  ScaledSample scale_sample(const RawSample & raw) const;

  static double accel_sensitivity_lsb_per_g(int range_g);
  static double gyro_sensitivity_lsb_per_dps(int range_dps);
  static double decode_temperature_c(uint8_t msb, uint8_t lsb);

private:
  std::vector<uint8_t> read_accel(uint8_t reg, std::size_t length) const;
  std::vector<uint8_t> read_gyro(uint8_t reg, std::size_t length) const;
  void write_accel(uint8_t reg, uint8_t value) const;
  void write_gyro(uint8_t reg, uint8_t value) const;
  void validate_config() const;
  uint8_t accel_range_code() const;
  uint8_t accel_odr_code() const;
  uint8_t accel_bandwidth_code() const;
  uint8_t gyro_range_code() const;
  uint8_t gyro_bandwidth_code() const;
  void verify_configuration() const;

  Bmi088Config config_;
  std::unique_ptr<SpiDevice> accel_spi_;
  std::unique_ptr<SpiDevice> gyro_spi_;
  uint8_t accel_chip_id_{0U};
  uint8_t gyro_chip_id_{0U};
};

}  // namespace bmi088_driver

#endif  // BMI088_DRIVER__BMI088_DEVICE_HPP_
