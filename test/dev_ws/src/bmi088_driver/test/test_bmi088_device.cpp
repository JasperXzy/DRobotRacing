#include "bmi088_driver/bmi088_device.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace bmi088_driver
{

TEST(Bmi088DeviceTest, ConvertsConfiguredRangesToSiUnits)
{
  Bmi088Config config;
  config.accel_range_g = 3;
  config.gyro_range_dps = 500;
  Bmi088Device device(config);

  RawSample raw;
  raw.accel = {10920, -10920, 0};
  raw.gyro = {0, 6554, -6554};
  const ScaledSample scaled = device.scale_sample(raw);

  EXPECT_NEAR(scaled.acceleration_mps2[0], 9.80665, 1.0e-6);
  EXPECT_NEAR(scaled.acceleration_mps2[1], -9.80665, 1.0e-6);
  EXPECT_NEAR(scaled.angular_velocity_rps[1], 100.0061 * M_PI / 180.0, 1.0e-4);
  EXPECT_NEAR(scaled.angular_velocity_rps[2], -100.0061 * M_PI / 180.0, 1.0e-4);
}

TEST(Bmi088DeviceTest, DecodesPositiveAndNegativeTemperature)
{
  EXPECT_DOUBLE_EQ(Bmi088Device::decode_temperature_c(0x00, 0x00), 23.0);
  EXPECT_DOUBLE_EQ(Bmi088Device::decode_temperature_c(0x01, 0x00), 24.0);
  EXPECT_DOUBLE_EQ(Bmi088Device::decode_temperature_c(0xFF, 0xE0), 22.875);
}

TEST(Bmi088DeviceTest, RejectsSharedChipSelect)
{
  Bmi088Config config;
  config.gyro_device = config.accel_device;
  EXPECT_THROW((void)Bmi088Device(config), std::invalid_argument);
}

TEST(Bmi088DeviceTest, RejectsClockAboveDatasheetLimit)
{
  Bmi088Config config;
  config.spi_speed_hz = 10000001U;
  EXPECT_THROW((void)Bmi088Device(config), std::invalid_argument);
}

}  // namespace bmi088_driver
