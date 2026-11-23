#include "bmi088_driver/bmi088_device.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace bmi088_driver
{
namespace
{

constexpr double kStandardGravity = 9.80665;
constexpr double kDegreesToRadians = 0.017453292519943295;

std::chrono::nanoseconds period_from_hz(double rate_hz)
{
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    throw std::invalid_argument("Rate parameters must be finite and greater than zero");
  }
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / rate_hz));
}

std::string format_vector(const std::array<double, 3> & value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(8) << value[0] << ", " << value[1] << ", " << value[2];
  return stream.str();
}

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

template<typename T>
diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const T & value)
{
  std::ostringstream stream;
  stream << value;
  return key_value(key, stream.str());
}

double vector_norm(const std::array<double, 3> & value)
{
  return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

}  // namespace

class Bmi088Node : public rclcpp::Node
{
public:
  Bmi088Node()
  : Node("bmi088_driver")
  {
    declare_and_read_parameters();
    validate_parameters();

    raw_publisher_ = create_publisher<sensor_msgs::msg::Imu>(raw_topic_, rclcpp::SensorDataQoS());
    calibrated_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      calibrated_topic_, rclcpp::SensorDataQoS());
    temperature_publisher_ = create_publisher<sensor_msgs::msg::Temperature>(
      temperature_topic_, rclcpp::SensorDataQoS());
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, rclcpp::QoS(10));

    calibration_service_ = create_service<std_srvs::srv::Trigger>(
      "calibrate_gyro",
      std::bind(
        &Bmi088Node::handle_calibration_request, this, std::placeholders::_1,
        std::placeholders::_2));

    if (calibrate_on_startup_) {
      start_calibration();
    }

    if (mock_mode_) {
      sensor_ready_ = true;
      last_error_.clear();
      RCLCPP_WARN(get_logger(), "BMI088 driver is running in mock_mode; no SPI hardware is used");
    } else {
      try_initialize_sensor();
    }

    sample_timer_ = create_wall_timer(
      period_from_hz(publish_rate_hz_), std::bind(&Bmi088Node::sample_callback, this));
    diagnostic_timer_ = create_wall_timer(
      period_from_hz(diagnostic_rate_hz_), std::bind(&Bmi088Node::diagnostic_callback, this));
    last_diagnostic_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      get_logger(),
      "Independent BMI088 node started: raw=%s calibrated=%s frame=%s rate=%.1f Hz",
      raw_publisher_->get_topic_name(), calibrated_publisher_->get_topic_name(), frame_id_.c_str(),
      publish_rate_hz_);
  }

private:
  void declare_and_read_parameters()
  {
    device_config_.accel_device = declare_parameter<std::string>(
      "accel_device", device_config_.accel_device);
    device_config_.gyro_device = declare_parameter<std::string>(
      "gyro_device", device_config_.gyro_device);
    device_config_.spi_speed_hz = static_cast<uint32_t>(declare_parameter<int>(
        "spi_speed_hz", static_cast<int>(device_config_.spi_speed_hz)));
    device_config_.accel_range_g = declare_parameter<int>(
      "accel_range_g", device_config_.accel_range_g);
    device_config_.accel_odr_hz = declare_parameter<int>(
      "accel_odr_hz", device_config_.accel_odr_hz);
    device_config_.accel_bandwidth = declare_parameter<std::string>(
      "accel_bandwidth", device_config_.accel_bandwidth);
    device_config_.gyro_range_dps = declare_parameter<int>(
      "gyro_range_dps", device_config_.gyro_range_dps);
    device_config_.gyro_odr_hz = declare_parameter<int>(
      "gyro_odr_hz", device_config_.gyro_odr_hz);
    device_config_.gyro_bandwidth_hz = declare_parameter<int>(
      "gyro_bandwidth_hz", device_config_.gyro_bandwidth_hz);

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);
    temperature_rate_hz_ = declare_parameter<double>("temperature_rate_hz", 1.0);
    diagnostic_rate_hz_ = declare_parameter<double>("diagnostic_rate_hz", 1.0);
    reconnect_interval_sec_ = declare_parameter<double>("reconnect_interval_sec", 2.0);
    max_consecutive_errors_ = declare_parameter<int>("max_consecutive_errors", 5);
    mock_mode_ = declare_parameter<bool>("mock_mode", false);

    frame_id_ = declare_parameter<std::string>("frame_id", "bmi088_link");
    raw_topic_ = declare_parameter<std::string>("raw_topic", "imu/data_raw");
    calibrated_topic_ = declare_parameter<std::string>("calibrated_topic", "imu/data");
    temperature_topic_ = declare_parameter<std::string>("temperature_topic", "temperature");
    diagnostics_topic_ = declare_parameter<std::string>("diagnostics_topic", "diagnostics");

    const auto axis_map_parameter = declare_parameter<std::vector<int64_t>>(
      "axis_map", std::vector<int64_t>{0, 1, 2});
    const auto axis_sign_parameter = declare_parameter<std::vector<int64_t>>(
      "axis_sign", std::vector<int64_t>{1, 1, 1});
    if (axis_map_parameter.size() != 3U || axis_sign_parameter.size() != 3U) {
      throw std::invalid_argument("axis_map and axis_sign must each contain exactly three values");
    }
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      axis_map_[axis] = static_cast<int>(axis_map_parameter[axis]);
      axis_sign_[axis] = static_cast<int>(axis_sign_parameter[axis]);
    }

    calibrate_on_startup_ = declare_parameter<bool>("calibrate_on_startup", true);
    calibration_samples_required_ = declare_parameter<int>("calibration_samples", 300);
    calibration_gyro_limit_rps_ = declare_parameter<double>(
      "calibration_gyro_limit_rps", 0.05);
    calibration_accel_tolerance_mps2_ = declare_parameter<double>(
      "calibration_accel_tolerance_mps2", 0.8);
    const auto initial_bias = declare_parameter<std::vector<double>>(
      "initial_gyro_bias_rps", std::vector<double>{0.0, 0.0, 0.0});
    if (initial_bias.size() != 3U) {
      throw std::invalid_argument("initial_gyro_bias_rps must contain exactly three values");
    }
    std::copy(initial_bias.begin(), initial_bias.end(), gyro_bias_rps_.begin());

    angular_velocity_stddev_ = declare_parameter<double>("angular_velocity_stddev", 0.003);
    linear_acceleration_stddev_ = declare_parameter<double>("linear_acceleration_stddev", 0.12);
    temperature_stddev_ = declare_parameter<double>("temperature_stddev", 1.0);
  }

  void validate_parameters()
  {
    // Constructing validates all BMI088 range/ODR/bandwidth combinations without touching hardware.
    (void)Bmi088Device(device_config_);
    (void)period_from_hz(publish_rate_hz_);
    (void)period_from_hz(temperature_rate_hz_);
    (void)period_from_hz(diagnostic_rate_hz_);

    if (publish_rate_hz_ > static_cast<double>(device_config_.accel_odr_hz) ||
      publish_rate_hz_ > static_cast<double>(device_config_.gyro_odr_hz))
    {
      throw std::invalid_argument("publish_rate_hz must not exceed either sensor ODR");
    }
    if (temperature_rate_hz_ > publish_rate_hz_) {
      throw std::invalid_argument("temperature_rate_hz must not exceed publish_rate_hz");
    }
    if (!std::isfinite(reconnect_interval_sec_) || reconnect_interval_sec_ <= 0.0) {
      throw std::invalid_argument("reconnect_interval_sec must be greater than zero");
    }
    if (max_consecutive_errors_ < 1 || calibration_samples_required_ < 1) {
      throw std::invalid_argument("Error and calibration sample counts must be at least one");
    }
    if (frame_id_.empty() || raw_topic_.empty() || calibrated_topic_.empty()) {
      throw std::invalid_argument("Frame and IMU topic names must not be empty");
    }
    std::array<bool, 3> mapped{{false, false, false}};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      if (axis_map_[axis] < 0 || axis_map_[axis] > 2 || mapped[axis_map_[axis]]) {
        throw std::invalid_argument("axis_map must be a permutation of [0, 1, 2]");
      }
      if (axis_sign_[axis] != -1 && axis_sign_[axis] != 1) {
        throw std::invalid_argument("axis_sign values must be -1 or 1");
      }
      mapped[axis_map_[axis]] = true;
    }
    if (calibration_gyro_limit_rps_ <= 0.0 || calibration_accel_tolerance_mps2_ <= 0.0) {
      throw std::invalid_argument("Calibration stationary thresholds must be greater than zero");
    }
    if (!std::isfinite(calibration_gyro_limit_rps_) ||
      !std::isfinite(calibration_accel_tolerance_mps2_))
    {
      throw std::invalid_argument("Calibration stationary thresholds must be finite");
    }
    if (!std::all_of(
        gyro_bias_rps_.begin(), gyro_bias_rps_.end(),
        [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument("initial_gyro_bias_rps values must be finite");
    }
    if (!std::isfinite(angular_velocity_stddev_) ||
      !std::isfinite(linear_acceleration_stddev_) ||
      !std::isfinite(temperature_stddev_) || angular_velocity_stddev_ < 0.0 ||
      linear_acceleration_stddev_ < 0.0 || temperature_stddev_ < 0.0)
    {
      throw std::invalid_argument("Standard deviation parameters must be finite and non-negative");
    }

    temperature_publish_divider_ = std::max<int64_t>(
      1, static_cast<int64_t>(std::llround(publish_rate_hz_ / temperature_rate_hz_)));
  }

  void try_initialize_sensor()
  {
    last_reconnect_attempt_ = std::chrono::steady_clock::now();
    try {
      auto candidate = std::make_unique<Bmi088Device>(device_config_);
      candidate->initialize();
      device_ = std::move(candidate);
      sensor_ready_ = true;
      consecutive_errors_ = 0U;
      last_error_.clear();
      RCLCPP_INFO(
        get_logger(),
        "BMI088 detected: accel ID=0x%02X on %s, gyro ID=0x%02X on %s",
        device_->accel_chip_id(), device_config_.accel_device.c_str(), device_->gyro_chip_id(),
        device_config_.gyro_device.c_str());
    } catch (const std::exception & error) {
      device_.reset();
      sensor_ready_ = false;
      last_error_ = error.what();
      RCLCPP_ERROR(get_logger(), "BMI088 initialization failed: %s", last_error_.c_str());
    }
  }

  bool reconnect_due() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_reconnect_attempt_).count() >=
           reconnect_interval_sec_;
  }

  ScaledSample mock_sample()
  {
    ScaledSample sample;
    sample.acceleration_mps2 = {0.0, 0.0, kStandardGravity};
    sample.angular_velocity_rps = {0.0, 0.0, 0.0};
    const double tick_seconds = 0.0000390625;
    const uint32_t increment = static_cast<uint32_t>(
      std::llround((1.0 / publish_rate_hz_) / tick_seconds));
    last_sensor_time_ = (last_sensor_time_ + increment) & 0x00FFFFFFU;
    return sample;
  }

  template<typename T>
  std::array<T, 3> remap(const std::array<T, 3> & input) const
  {
    std::array<T, 3> output{};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      output[axis] = static_cast<T>(axis_sign_[axis]) * input[axis_map_[axis]];
    }
    return output;
  }

  void sample_callback()
  {
    if (!sensor_ready_) {
      if (!mock_mode_ && reconnect_due()) {
        try_initialize_sensor();
      }
      return;
    }

    try {
      ScaledSample scaled;
      if (mock_mode_) {
        scaled = mock_sample();
      } else {
        const RawSample raw = device_->read_sample();
        last_sensor_time_ = raw.accel_sensor_time;
        scaled = device_->scale_sample(raw);
      }

      const auto acceleration = remap(scaled.acceleration_mps2);
      const auto angular_velocity = remap(scaled.angular_velocity_rps);
      const rclcpp::Time stamp = now();

      publish_imu(raw_publisher_, stamp, acceleration, angular_velocity);
      update_calibration(acceleration, angular_velocity);

      std::array<double, 3> calibrated_velocity = angular_velocity;
      for (std::size_t axis = 0; axis < 3U; ++axis) {
        calibrated_velocity[axis] -= gyro_bias_rps_[axis];
      }
      // Do not feed consumers a nominally "calibrated" sample while the
      // startup/manual calibration is still collecting its bias estimate.
      // update_calibration() clears calibrating_ on the final accepted sample,
      // so publishing begins immediately with the newly computed bias.
      if (!calibrating_) {
        publish_imu(calibrated_publisher_, stamp, acceleration, calibrated_velocity);
      }

      ++sample_count_;
      consecutive_errors_ = 0U;
      if (sample_count_ % static_cast<uint64_t>(temperature_publish_divider_) == 0U) {
        const double temperature = mock_mode_ ? 25.0 : device_->read_temperature_c();
        publish_temperature(stamp, temperature);
      }
      last_error_.clear();
    } catch (const std::exception & error) {
      ++read_error_count_;
      ++consecutive_errors_;
      last_error_ = error.what();
      if (consecutive_errors_ == 1U) {
        RCLCPP_ERROR(get_logger(), "BMI088 read failed: %s", last_error_.c_str());
      }
      if (consecutive_errors_ >= static_cast<uint64_t>(max_consecutive_errors_)) {
        RCLCPP_ERROR(
          get_logger(), "BMI088 marked offline after %llu consecutive errors; reconnecting",
          static_cast<unsigned long long>(consecutive_errors_));
        sensor_ready_ = false;
        device_.reset();
        last_reconnect_attempt_ = std::chrono::steady_clock::now();
      }
    }
  }

  void publish_imu(
    const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr & publisher,
    const rclcpp::Time & stamp,
    const std::array<double, 3> & acceleration,
    const std::array<double, 3> & angular_velocity)
  {
    sensor_msgs::msg::Imu message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.orientation.w = 1.0;
    message.orientation_covariance[0] = -1.0;

    message.angular_velocity.x = angular_velocity[0];
    message.angular_velocity.y = angular_velocity[1];
    message.angular_velocity.z = angular_velocity[2];
    const double gyro_variance = angular_velocity_stddev_ * angular_velocity_stddev_;
    message.angular_velocity_covariance[0] = gyro_variance;
    message.angular_velocity_covariance[4] = gyro_variance;
    message.angular_velocity_covariance[8] = gyro_variance;

    message.linear_acceleration.x = acceleration[0];
    message.linear_acceleration.y = acceleration[1];
    message.linear_acceleration.z = acceleration[2];
    const double accel_variance = linear_acceleration_stddev_ * linear_acceleration_stddev_;
    message.linear_acceleration_covariance[0] = accel_variance;
    message.linear_acceleration_covariance[4] = accel_variance;
    message.linear_acceleration_covariance[8] = accel_variance;
    publisher->publish(message);
  }

  void publish_temperature(const rclcpp::Time & stamp, double temperature_c)
  {
    sensor_msgs::msg::Temperature message;
    message.header.stamp = stamp;
    message.header.frame_id = frame_id_;
    message.temperature = temperature_c;
    message.variance = temperature_stddev_ * temperature_stddev_;
    last_temperature_c_ = temperature_c;
    temperature_publisher_->publish(message);
  }

  void start_calibration()
  {
    calibrating_ = true;
    calibration_collected_ = 0U;
    calibration_rejected_ = 0U;
    calibration_sum_ = {0.0, 0.0, 0.0};
    calibration_sum_squares_ = {0.0, 0.0, 0.0};
    RCLCPP_INFO(
      get_logger(), "Gyroscope calibration started; keep the BMI088 completely stationary (%d samples)",
      calibration_samples_required_);
  }

  void update_calibration(
    const std::array<double, 3> & acceleration,
    const std::array<double, 3> & angular_velocity)
  {
    if (!calibrating_) {
      return;
    }
    const bool gyro_stationary = vector_norm(angular_velocity) <= calibration_gyro_limit_rps_;
    const bool accel_stationary =
      std::abs(vector_norm(acceleration) - kStandardGravity) <= calibration_accel_tolerance_mps2_;
    if (!gyro_stationary || !accel_stationary) {
      ++calibration_rejected_;
      return;
    }

    for (std::size_t axis = 0; axis < 3U; ++axis) {
      calibration_sum_[axis] += angular_velocity[axis];
      calibration_sum_squares_[axis] += angular_velocity[axis] * angular_velocity[axis];
    }
    ++calibration_collected_;
    if (calibration_collected_ < static_cast<uint64_t>(calibration_samples_required_)) {
      return;
    }

    std::array<double, 3> standard_deviation{};
    for (std::size_t axis = 0; axis < 3U; ++axis) {
      gyro_bias_rps_[axis] = calibration_sum_[axis] /
        static_cast<double>(calibration_collected_);
      const double mean_square = calibration_sum_squares_[axis] /
        static_cast<double>(calibration_collected_);
      standard_deviation[axis] = std::sqrt(
        std::max(
          0.0, mean_square - gyro_bias_rps_[axis] * gyro_bias_rps_[axis]));
    }
    calibrating_ = false;
    RCLCPP_INFO(
      get_logger(), "Gyroscope calibration complete: bias=[%s] rad/s, stddev=[%s] rad/s",
      format_vector(gyro_bias_rps_).c_str(), format_vector(standard_deviation).c_str());
  }

  void handle_calibration_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;
    if (!sensor_ready_) {
      response->success = false;
      response->message = "BMI088 is offline; calibration was not started";
      return;
    }
    start_calibration();
    response->success = true;
    response->message = "Calibration started; monitor /bmi088/diagnostics for completion";
  }

  void diagnostic_callback()
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": BMI088 SPI IMU");
    status.hardware_id = mock_mode_ ? "BMI088-MOCK" : "BMI088";

    if (!sensor_ready_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = last_error_.empty() ? "sensor offline" : last_error_;
    } else if (calibrating_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "sensor online; gyroscope calibration in progress";
    } else if (consecutive_errors_ > 0U) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "sensor online with recent SPI errors";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = mock_mode_ ? "mock data active" : "sensor online";
    }

    const auto current_time = std::chrono::steady_clock::now();
    const double elapsed =
      std::chrono::duration<double>(current_time - last_diagnostic_time_).count();
    const double measured_rate = elapsed > 0.0 ?
      static_cast<double>(sample_count_ - samples_at_last_diagnostic_) / elapsed : 0.0;
    samples_at_last_diagnostic_ = sample_count_;
    last_diagnostic_time_ = current_time;

    status.values.push_back(key_value("mock_mode", mock_mode_ ? "true" : "false"));
    status.values.push_back(key_value("accel_device", device_config_.accel_device));
    status.values.push_back(key_value("gyro_device", device_config_.gyro_device));
    status.values.push_back(key_value("spi_speed_hz", device_config_.spi_speed_hz));
    status.values.push_back(key_value("configured_publish_rate_hz", publish_rate_hz_));
    status.values.push_back(key_value("measured_publish_rate_hz", measured_rate));
    status.values.push_back(key_value("published_samples", sample_count_));
    status.values.push_back(key_value("read_errors", read_error_count_));
    status.values.push_back(key_value("consecutive_errors", consecutive_errors_));
    status.values.push_back(key_value("accel_sensor_time_24bit", last_sensor_time_));
    status.values.push_back(key_value("temperature_c", last_temperature_c_));
    status.values.push_back(key_value("gyro_bias_rps", format_vector(gyro_bias_rps_)));
    status.values.push_back(key_value("calibrating", calibrating_ ? "true" : "false"));
    status.values.push_back(key_value("calibration_valid_samples", calibration_collected_));
    status.values.push_back(key_value("calibration_rejected_samples", calibration_rejected_));
    status.values.push_back(
      key_value(
        "calibration_required_samples",
        calibration_samples_required_));
    if (!last_error_.empty()) {
      status.values.push_back(key_value("last_error", last_error_));
    }
    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  Bmi088Config device_config_;
  std::unique_ptr<Bmi088Device> device_;
  bool mock_mode_{false};
  bool sensor_ready_{false};
  std::string last_error_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_{};

  double publish_rate_hz_{100.0};
  double temperature_rate_hz_{1.0};
  double diagnostic_rate_hz_{1.0};
  double reconnect_interval_sec_{2.0};
  int max_consecutive_errors_{5};
  int64_t temperature_publish_divider_{100};

  std::string frame_id_;
  std::string raw_topic_;
  std::string calibrated_topic_;
  std::string temperature_topic_;
  std::string diagnostics_topic_;
  std::array<int, 3> axis_map_{{0, 1, 2}};
  std::array<int, 3> axis_sign_{{1, 1, 1}};

  bool calibrate_on_startup_{true};
  bool calibrating_{false};
  int calibration_samples_required_{300};
  double calibration_gyro_limit_rps_{0.05};
  double calibration_accel_tolerance_mps2_{0.8};
  uint64_t calibration_collected_{0U};
  uint64_t calibration_rejected_{0U};
  std::array<double, 3> calibration_sum_{{0.0, 0.0, 0.0}};
  std::array<double, 3> calibration_sum_squares_{{0.0, 0.0, 0.0}};
  std::array<double, 3> gyro_bias_rps_{{0.0, 0.0, 0.0}};

  double angular_velocity_stddev_{0.003};
  double linear_acceleration_stddev_{0.12};
  double temperature_stddev_{1.0};
  double last_temperature_c_{std::numeric_limits<double>::quiet_NaN()};

  uint64_t sample_count_{0U};
  uint64_t read_error_count_{0U};
  uint64_t consecutive_errors_{0U};
  uint64_t samples_at_last_diagnostic_{0U};
  uint32_t last_sensor_time_{0U};
  std::chrono::steady_clock::time_point last_diagnostic_time_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr calibrated_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

}  // namespace bmi088_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<bmi088_driver::Bmi088Node>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("bmi088_driver"), "Fatal error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
