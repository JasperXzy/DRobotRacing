// Copyright (c) 2026.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0

#include <alsa/asoundlib.h>
#include <dlfcn.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "dual_rate_tts/tts_api.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace dual_rate_tts
{

namespace
{

constexpr std::size_t kMaxMessageQueueSize = 10;
constexpr std::size_t kMaxPlaybackQueueSize = 5;
constexpr int kPlaybackChannels = 2;

std::string default_model_directory()
{
  const char * distro = std::getenv("TROS_DISTRO");
  const std::string tros_distro =
    (distro != nullptr && distro[0] != '\0') ? distro : "humble";
  return "/opt/tros/" + tros_distro + "/lib/hobot_tts/tts_model";
}

bool starts_with(const std::string & input, std::size_t index, const char * value)
{
  const std::string token(value);
  return index + token.size() <= input.size() &&
         input.compare(index, token.size(), token) == 0;
}

std::size_t delimiter_length(const std::string & input, std::size_t index)
{
  static const std::vector<const char *> kUtf8Delimiters = {
    "，", "。", "！", "？", "：", "；", "、"};
  for (const char * delimiter : kUtf8Delimiters) {
    if (starts_with(input, index, delimiter)) {
      return std::string(delimiter).size();
    }
  }

  const unsigned char byte = static_cast<unsigned char>(input[index]);
  if (byte < 0x80 &&
    (std::isspace(byte) != 0 || std::ispunct(byte) != 0))
  {
    return 1;
  }
  return 0;
}

std::vector<std::string> split_text(std::string input)
{
  for (char & value : input) {
    const unsigned char byte = static_cast<unsigned char>(value);
    if (byte < 0x80 && std::isupper(byte) != 0) {
      value = static_cast<char>(std::tolower(byte));
    }
  }

  std::vector<std::string> segments;
  std::size_t segment_start = 0;
  std::size_t index = 0;
  while (index < input.size()) {
    const std::size_t delimiter_size = delimiter_length(input, index);
    if (delimiter_size == 0) {
      ++index;
      continue;
    }
    if (index > segment_start) {
      segments.emplace_back(input.substr(segment_start, index - segment_start));
    }
    index += delimiter_size;
    segment_start = index;
  }
  if (segment_start < input.size()) {
    segments.emplace_back(input.substr(segment_start));
  }
  return segments;
}

}  // namespace

class SonicApi
{
public:
  explicit SonicApi(const rclcpp::Logger & logger)
  : logger_(logger)
  {
    library_ = dlopen("libsonic.so.0", RTLD_NOW | RTLD_LOCAL);
    if (library_ == nullptr) {
      RCLCPP_WARN(
        logger_, "libsonic unavailable; accelerated TTS will use 1.00x: %s",
        dlerror());
      return;
    }

    create_stream_ = load_symbol<CreateStream>("sonicCreateStream");
    destroy_stream_ = load_symbol<DestroyStream>("sonicDestroyStream");
    set_speed_ = load_symbol<SetSpeed>("sonicSetSpeed");
    write_short_ = load_symbol<WriteShort>("sonicWriteShortToStream");
    flush_stream_ = load_symbol<FlushStream>("sonicFlushStream");
    samples_available_ =
      load_symbol<SamplesAvailable>("sonicSamplesAvailable");
    read_short_ = load_symbol<ReadShort>("sonicReadShortFromStream");

    if (create_stream_ == nullptr || destroy_stream_ == nullptr ||
      set_speed_ == nullptr || write_short_ == nullptr ||
      flush_stream_ == nullptr || samples_available_ == nullptr ||
      read_short_ == nullptr)
    {
      RCLCPP_WARN(
        logger_, "libsonic is missing required symbols; accelerated TTS "
        "will use 1.00x");
      dlclose(library_);
      library_ = nullptr;
    }
  }

  ~SonicApi()
  {
    if (library_ != nullptr) {
      dlclose(library_);
    }
  }

  SonicApi(const SonicApi &) = delete;
  SonicApi & operator=(const SonicApi &) = delete;

  bool available() const
  {
    return library_ != nullptr;
  }

  bool change_speed(
    const std::vector<int16_t> & input, int sample_rate, double speed,
    std::vector<int16_t> & output) const
  {
    static_assert(sizeof(int16_t) == sizeof(short));
    output.clear();
    if (!available() || input.empty() || speed <= 1.0001 ||
      input.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    {
      return false;
    }

    void * stream = create_stream_(sample_rate, 1);
    if (stream == nullptr) {
      return false;
    }

    bool succeeded = false;
    set_speed_(stream, static_cast<float>(speed));
    auto * samples = const_cast<short *>(
      reinterpret_cast<const short *>(input.data()));
    if (write_short_(stream, samples, static_cast<int>(input.size())) != 0 &&
      flush_stream_(stream) != 0)
    {
      while (true) {
        const int available_samples = samples_available_(stream);
        if (available_samples <= 0) {
          break;
        }
        std::vector<int16_t> chunk(
          static_cast<std::size_t>(available_samples));
        const int read_samples = read_short_(
          stream, reinterpret_cast<short *>(chunk.data()), available_samples);
        if (read_samples <= 0) {
          break;
        }
        output.insert(
          output.end(), chunk.begin(), chunk.begin() + read_samples);
      }
      succeeded = !output.empty();
    }

    destroy_stream_(stream);
    return succeeded;
  }

private:
  using CreateStream = void * (*)(int, int);
  using DestroyStream = void (*)(void *);
  using SetSpeed = void (*)(void *, float);
  using WriteShort = int (*)(void *, short *, int);
  using FlushStream = int (*)(void *);
  using SamplesAvailable = int (*)(void *);
  using ReadShort = int (*)(void *, short *, int);

  template<typename SymbolType>
  SymbolType load_symbol(const char * name)
  {
    return reinterpret_cast<SymbolType>(dlsym(library_, name));
  }

  rclcpp::Logger logger_;
  void * library_{nullptr};
  CreateStream create_stream_{nullptr};
  DestroyStream destroy_stream_{nullptr};
  SetSpeed set_speed_{nullptr};
  WriteShort write_short_{nullptr};
  FlushStream flush_stream_{nullptr};
  SamplesAvailable samples_available_{nullptr};
  ReadShort read_short_{nullptr};
};

struct SpeechRequest
{
  std::string text;
  double speed{1.0};
  std::string source;
};

struct AudioClip
{
  std::vector<int16_t> stereo_pcm;
  double speed{1.0};
  std::string source;
  std::string text;
};

class DualRateTtsNode : public rclcpp::Node
{
public:
  DualRateTtsNode()
  : Node("dual_rate_tts_node")
  {
    playback_device_name_ = declare_parameter<std::string>(
      "playback_device", "plughw:0,0");
    normal_topic_ = declare_parameter<std::string>(
      "normal_topic", "/tts_text");
    fast_topic_ = declare_parameter<std::string>(
      "fast_topic", "/vlm_tts_text");
    fast_speed_ = declare_parameter<double>("fast_speed", 1.25);
    speaker_id_ = declare_parameter<int>("speaker_id", 1);
    model_directory_ = declare_parameter<std::string>(
      "model_directory", default_model_directory());
    model_flags_ = declare_parameter<std::string>(
      "model_flags", "tts.flags");
    alsa_latency_us_ = declare_parameter<int>("alsa_latency_us", 150000);

    if (normal_topic_ == fast_topic_) {
      throw std::runtime_error("normal_topic and fast_topic must be different");
    }
    const double requested_speed = fast_speed_;
    fast_speed_ = std::clamp(fast_speed_, 1.0, 2.0);
    if (fast_speed_ != requested_speed) {
      RCLCPP_WARN(
        get_logger(), "fast_speed %.3f is outside [1.0, 2.0]; using %.3f",
        requested_speed, fast_speed_);
    }
    if (fast_speed_ > 1.25) {
      RCLCPP_WARN(
        get_logger(), "fast_speed above 1.25 may reduce speech clarity");
    }
    alsa_latency_us_ = std::max(20000, alsa_latency_us_);

    try {
      initialize_model();
      sonic_ = std::make_unique<SonicApi>(get_logger());
      open_playback_device();

      normal_subscription_ = create_subscription<std_msgs::msg::String>(
        normal_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          enqueue_message(msg->data, 1.0, "normal");
        });
      fast_subscription_ = create_subscription<std_msgs::msg::String>(
        fast_topic_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
          enqueue_message(msg->data, fast_speed_, "vlm");
        });

      processing_thread_ = std::thread(
        &DualRateTtsNode::process_messages, this);
      playback_thread_ = std::thread(
        &DualRateTtsNode::playback_messages, this);
    } catch (...) {
      stop();
      cleanup_resources();
      throw;
    }

    RCLCPP_INFO(
      get_logger(),
      "TTS ready: %s=1.00x, %s=%.2fx, device=%s, sample_rate=%d",
      normal_topic_.c_str(), fast_topic_.c_str(), fast_speed_,
      playback_device_name_.c_str(), audio_info_.sample_rate);
  }

  ~DualRateTtsNode() override
  {
    stop();
    cleanup_resources();
  }

private:
  void initialize_model()
  {
    int error_code = 0;
    tts_ = wetts_init(
      model_directory_.c_str(), model_flags_.c_str(), &error_code);
    if (tts_ == nullptr) {
      throw std::runtime_error(
              "wetts_init failed with code " + std::to_string(error_code));
    }

    audio_info_ = wetts_audio_info(tts_);
    if (audio_info_.sample_rate <= 0 || audio_info_.max_len <= 0) {
      throw std::runtime_error("wetts_audio_info returned invalid values");
    }
    const std::size_t float_count =
      (static_cast<std::size_t>(audio_info_.max_len) + sizeof(float) - 1) /
      sizeof(float);
    pcm_buffer_.resize(float_count);

    RCLCPP_INFO(
      get_logger(),
      "Loaded TTS model: rate=%d, bit_depth=%d, channels=%d, max_ms=%d",
      audio_info_.sample_rate, audio_info_.bit_depth,
      audio_info_.num_channels, audio_info_.max_dur_ms);
  }

  void open_playback_device()
  {
    int error = snd_pcm_open(
      &playback_handle_, playback_device_name_.c_str(),
      SND_PCM_STREAM_PLAYBACK, 0);
    if (error < 0) {
      throw std::runtime_error(
              "snd_pcm_open failed: " + std::string(snd_strerror(error)));
    }

    error = snd_pcm_set_params(
      playback_handle_, SND_PCM_FORMAT_S16_LE,
      SND_PCM_ACCESS_RW_INTERLEAVED, kPlaybackChannels,
      static_cast<unsigned int>(audio_info_.sample_rate), 1,
      static_cast<unsigned int>(alsa_latency_us_));
    if (error < 0) {
      throw std::runtime_error(
              "snd_pcm_set_params failed: " +
              std::string(snd_strerror(error)));
    }
  }

  void enqueue_message(
    const std::string & text, double speed, const std::string & source)
  {
    if (text.empty()) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(message_mutex_);
      if (message_queue_.size() >= kMaxMessageQueueSize) {
        message_queue_.pop_front();
        RCLCPP_WARN(
          get_logger(), "TTS queue full; discarded oldest pending message");
      }
      message_queue_.push_back(SpeechRequest{text, speed, source});
    }
    message_cv_.notify_one();
  }

  void process_messages()
  {
    while (!stop_requested_.load()) {
      SpeechRequest request;
      {
        std::unique_lock<std::mutex> lock(message_mutex_);
        message_cv_.wait(
          lock,
          [this]() {
            return stop_requested_.load() || !message_queue_.empty();
          });
        if (stop_requested_.load()) {
          return;
        }
        request = std::move(message_queue_.front());
        message_queue_.pop_front();
      }

      for (const std::string & segment : split_text(request.text)) {
        if (stop_requested_.load()) {
          return;
        }
        std::vector<int16_t> mono_pcm;
        if (!synthesize(segment, mono_pcm)) {
          continue;
        }

        double effective_speed = 1.0;
        if (request.speed > 1.0001 && sonic_ != nullptr) {
          std::vector<int16_t> accelerated_pcm;
          if (sonic_->change_speed(
              mono_pcm, audio_info_.sample_rate, request.speed,
              accelerated_pcm))
          {
            mono_pcm = std::move(accelerated_pcm);
            effective_speed = request.speed;
          } else {
            RCLCPP_WARN(
              get_logger(),
              "TTS acceleration failed; playing this segment at 1.00x");
          }
        }

        AudioClip clip;
        clip.stereo_pcm.reserve(mono_pcm.size() * kPlaybackChannels);
        for (const int16_t sample : mono_pcm) {
          clip.stereo_pcm.push_back(sample);
          clip.stereo_pcm.push_back(sample);
        }
        clip.speed = effective_speed;
        clip.source = request.source;
        clip.text = segment;

        {
          std::lock_guard<std::mutex> lock(playback_mutex_);
          if (playback_queue_.size() >= kMaxPlaybackQueueSize) {
            playback_queue_.pop_front();
            RCLCPP_WARN(
              get_logger(),
              "TTS playback queue full; discarded oldest audio segment");
          }
          playback_queue_.push_back(std::move(clip));
        }
        playback_cv_.notify_one();
      }
    }
  }

  bool synthesize(const std::string & text, std::vector<int16_t> & output)
  {
    int pcm_size = 0;
    const int error_code = wetts_synthesis(
      tts_, text.c_str(), speaker_id_,
      reinterpret_cast<char *>(pcm_buffer_.data()), &pcm_size);
    if (error_code != ERRCODE_TTS_SUCC) {
      RCLCPP_ERROR(
        get_logger(), "TTS synthesis failed with code 0x%x for: %s",
        error_code, text.c_str());
      return false;
    }
    if (pcm_size <= 0 ||
      static_cast<std::size_t>(pcm_size) > pcm_buffer_.size())
    {
      RCLCPP_ERROR(
        get_logger(), "TTS synthesis returned invalid PCM size: %d", pcm_size);
      return false;
    }

    output.clear();
    output.reserve(static_cast<std::size_t>(pcm_size));
    for (int index = 0; index < pcm_size; ++index) {
      const float sample = pcm_buffer_[static_cast<std::size_t>(index)];
      if (!std::isfinite(sample)) {
        output.push_back(0);
        continue;
      }
      const float bounded = std::clamp(sample, -32768.0F, 32767.0F);
      output.push_back(static_cast<int16_t>(bounded));
    }
    return !output.empty();
  }

  void playback_messages()
  {
    while (true) {
      AudioClip clip;
      {
        std::unique_lock<std::mutex> lock(playback_mutex_);
        playback_cv_.wait(
          lock,
          [this]() {
            return stop_requested_.load() || !playback_queue_.empty();
          });
        if (stop_requested_.load() && playback_queue_.empty()) {
          return;
        }
        clip = std::move(playback_queue_.front());
        playback_queue_.pop_front();
      }

      RCLCPP_INFO(
        get_logger(), "Playing %s TTS at %.2fx: %s",
        clip.source.c_str(), clip.speed, clip.text.c_str());
      if (!write_audio(clip.stereo_pcm)) {
        RCLCPP_ERROR(get_logger(), "Failed to write TTS audio to ALSA");
      }
    }
  }

  bool write_audio(const std::vector<int16_t> & stereo_pcm)
  {
    if (playback_handle_ == nullptr || stereo_pcm.empty()) {
      return false;
    }
    int error = snd_pcm_prepare(playback_handle_);
    if (error < 0) {
      RCLCPP_ERROR(
        get_logger(), "snd_pcm_prepare failed: %s", snd_strerror(error));
      return false;
    }

    const int16_t * cursor = stereo_pcm.data();
    snd_pcm_uframes_t frames_left =
      static_cast<snd_pcm_uframes_t>(
      stereo_pcm.size() / kPlaybackChannels);
    while (frames_left > 0) {
      const snd_pcm_sframes_t written = snd_pcm_writei(
        playback_handle_, cursor, frames_left);
      if (written < 0) {
        error = snd_pcm_recover(
          playback_handle_, static_cast<int>(written), 1);
        if (error < 0) {
          RCLCPP_ERROR(
            get_logger(), "snd_pcm_writei failed: %s", snd_strerror(error));
          return false;
        }
        continue;
      }
      if (written == 0) {
        continue;
      }
      frames_left -= static_cast<snd_pcm_uframes_t>(written);
      cursor += written * kPlaybackChannels;
    }

    error = snd_pcm_drain(playback_handle_);
    if (error < 0) {
      RCLCPP_ERROR(
        get_logger(), "snd_pcm_drain failed: %s", snd_strerror(error));
      return false;
    }
    return true;
  }

  void stop()
  {
    if (stop_requested_.exchange(true)) {
      return;
    }
    message_cv_.notify_all();
    playback_cv_.notify_all();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
    if (playback_thread_.joinable()) {
      playback_thread_.join();
    }
  }

  void cleanup_resources()
  {
    if (playback_handle_ != nullptr) {
      snd_pcm_drop(playback_handle_);
      snd_pcm_close(playback_handle_);
      playback_handle_ = nullptr;
    }
    sonic_.reset();
    if (tts_ != nullptr) {
      wetts_free(tts_);
      tts_ = nullptr;
    }
  }

  std::string playback_device_name_;
  std::string normal_topic_;
  std::string fast_topic_;
  double fast_speed_{1.25};
  int speaker_id_{1};
  std::string model_directory_;
  std::string model_flags_;
  int alsa_latency_us_{150000};

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr normal_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fast_subscription_;

  std::deque<SpeechRequest> message_queue_;
  std::mutex message_mutex_;
  std::condition_variable message_cv_;

  std::deque<AudioClip> playback_queue_;
  std::mutex playback_mutex_;
  std::condition_variable playback_cv_;

  std::atomic<bool> stop_requested_{false};
  std::thread processing_thread_;
  std::thread playback_thread_;

  void * tts_{nullptr};
  struct audio_info audio_info_ {};
  std::vector<float> pcm_buffer_;
  std::unique_ptr<SonicApi> sonic_;
  snd_pcm_t * playback_handle_{nullptr};
};

}  // namespace dual_rate_tts

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<dual_rate_tts::DualRateTtsNode>();
    rclcpp::spin(node);
    node.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("dual_rate_tts_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
