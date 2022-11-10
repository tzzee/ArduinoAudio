/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#ifndef LIB_ARDUINO_AUDIO_DUMMYAUDIO_H_
#define LIB_ARDUINO_AUDIO_DUMMYAUDIO_H_

#include "AudioImpl.h"

class DummyAudio : public AudioImpl {
  using super = AudioImpl;
 public:
  DummyAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint16_t bufferMsec, std::uint8_t channelNum);
  void begin() override;
  void start() override;
  void stop() override;
  void zero() override;
  int available() override;
  int availableForWrite() override;
  std::size_t read(uint8_t *buffer, std::size_t length) override;
  std::size_t write(const uint8_t *buffer, std::size_t length) override;
};

#endif  // LIB_ARDUINO_AUDIO_DUMMYAUDIO_H_