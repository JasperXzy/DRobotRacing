// Copyright 2024 D-Robotics Inc. All Rights Reserved.
// Author: zhendong.peng@d-robotics.cc (Zhendong Peng)

#ifndef DUAL_RATE_TTS__TTS_API_H_
#define DUAL_RATE_TTS__TTS_API_H_

#ifdef __cplusplus
extern "C" {
#endif

enum en_rec_err_code {
  ERRCODE_TTS_SUCC = 0x10000,
  ERRCODE_TTS_INVALID_INPUT = 0x10001,
  ERRCODE_TTS_LONG_INPUT = 0x10002,
  ERRCODE_TTS_INIT_MODEL = 0x10003,
  ERRCODE_TTS_MODEL_FILE = 0x10004,
  ERRCODE_TTS_INVALID_OUTPUT = 0x10005,
};

struct audio_info {
  int sample_rate;
  int bit_depth;
  int num_channels;
  int max_dur_ms;
  int max_len;
};

void * wetts_init(const char * top_dir, const char * file, int * errcode)
  __attribute__((visibility("default")));
void wetts_free(void * tts) __attribute__((visibility("default")));
struct audio_info wetts_audio_info(void * tts)
  __attribute__((visibility("default")));
int wetts_synthesis(
  void * tts, const char * text, const int speaker_id,
  char * pcm_data, int * pcm_size)
  __attribute__((visibility("default")));

#ifdef __cplusplus
}
#endif

#endif  // DUAL_RATE_TTS__TTS_API_H_
