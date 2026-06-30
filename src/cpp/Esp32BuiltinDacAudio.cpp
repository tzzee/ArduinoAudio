/**
 * @copyright 2021 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../Esp32BuiltinDacAudio.h"
#include <assert.h>
#include <esp32-hal.h>
#include <algorithm>
#include <math.h>
#include <string.h>

#define CH_NUM 2

#ifdef IDF_VER
static const i2s_comm_format_t builtin_dac_comm_format = I2S_COMM_FORMAT_STAND_MSB;
#else
static const i2s_comm_format_t builtin_dac_comm_format = I2S_COMM_FORMAT_I2S_MSB;
#endif

#ifdef IDF_VER
static const int default_channel_index_rl = 0;
#else
static const int default_channel_index_rl = 1;
#endif

#ifdef ARDUINO_AUDIO_SWAPCHANNEL_DISABLED
static const int channelIndexRL = 1 - default_channel_index_rl;
#else
static const int channelIndexRL = default_channel_index_rl;
#endif

Esp32BuiltinDacAudio::Esp32BuiltinDacAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec,
  uint8_t bufferCount, i2s_dac_mode_t dac_mode, std::uint16_t dcCutOffFrequency, const Esp32BuiltinDacAudioConfig& config, uint8_t ringBufferCount):
    super(sampleRate, bitDepth, alignedBitLength, bufferMsec, CH_NUM,
    bufferCount,
    I2SAudioConfig{
      .port = config.port,
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .chFormat = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .comFormat = builtin_dac_comm_format,
      .txDescAutoClear = dcCutOffFrequency == 0,
      .pinConfig = config.pinConfig
    }, ringBufferCount), dac_mode(dac_mode), dcCutOffFrequency(dcCutOffFrequency) {
}

Esp32BuiltinDacAudio::~Esp32BuiltinDacAudio() {
  Esp32BuiltinDacAudio::stop();  // virtualではなく、自分を呼ぶ
}

void Esp32BuiltinDacAudio::begin() {
  super::begin();
}

//  start DAC
//  DAC_Start()後は、DMAバッファが空になる前にDAC_Write()で出力データを書き込むこと
void Esp32BuiltinDacAudio::start() {
  dcBlockPrevInput = 0.0f;
  dcBlockPrevOutput = 0.0f;
  super::start(); // 中でzeroが呼ばれる
  // i2s_set_dac_mode(dac_mode);
  dacStatus = DacStarting;
  zero();  // DACの出力を0から中立にする
  dacStatus = DacRunning;
  zero();  // DACの出力を中立にする
}

//  stop DAC
//  データ出力後、DMAバッファが空になる前にこの関数を呼び出すこと
void Esp32BuiltinDacAudio::stop() {
  dacStatus = DacStopping;
  zero();  // DACの出力を中立から0にする
  dacStatus = DacStopped;
  zero();  // DACの出力を0にする
  super::stop();
}

void Esp32BuiltinDacAudio::zero() {
  const int32_t bottom = INT16_MIN;  // DACの0V
  uint8_t tmp[getPayloadSize()];
  int16_t *t = reinterpret_cast<int16_t*>(tmp);
  switch (dacStatus)
  {
  case DacStarting: {
    for(size_t j = 0; j < getBufferCount(); j++) {
      for(size_t i = 0; i < getBufferLength(); i++) {
        const int16_t v = bottom-(bottom*(int32_t)(j*getBufferLength()+i)/(int32_t)(getBufferCount()*getBufferLength()));
        t[i] = v;
      }
      waitForWritable();
      write(reinterpret_cast<const uint8_t*>(tmp), getPayloadSize());
    }
  } break;
  case DacRunning: {
    fill(0);
  } break;
  case DacStopping: {
    for(size_t j = 0; j < getBufferCount(); j++) {
      for(size_t i = 0; i < getBufferLength(); i++) {
        const int16_t v = bottom*(1-(j*getBufferLength()+i)/(getBufferCount()*getBufferLength()));
        t[i] = v;
      }
      waitForWritable();
      write(reinterpret_cast<const uint8_t*>(tmp), getPayloadSize());
    }
  } break;
  case DacStopped: {
    fill(bottom);
  } break;
  }
}

bool Esp32BuiltinDacAudio::handleTxIdle() {
  if (dcCutOffFrequency == 0 || dacStatus != DacRunning) {
    return false;
  }
  uint8_t tmp[super::getPayloadSize()];
  memset(tmp, 0, sizeof(tmp));
  return writeTxDmaBuffer(tmp, sizeof(tmp), getBufferCount());
}

void Esp32BuiltinDacAudio::fill(int16_t v) {
  uint8_t tmp[getPayloadSize()];
  int16_t *t = reinterpret_cast<int16_t*>(tmp);
  for(size_t j = 0; j < getBufferCount(); j++) {
    for(size_t i = 0; i < getBufferLength(); i++) {
      t[i] = v;
    }
    waitForWritable();
    write(reinterpret_cast<const uint8_t*>(tmp), getPayloadSize());
  }
}

std::size_t const Esp32BuiltinDacAudio::getPayloadSize() const {
  return super::getPayloadSize() / CH_NUM;
}

size_t Esp32BuiltinDacAudio::read(std::uint8_t *buffer, std::size_t length) {
  return 0;
}

size_t Esp32BuiltinDacAudio::write(const std::uint8_t *buffer, std::size_t length) {
  // length==getPayloadSize()
  uint8_t tmp[super::getPayloadSize()];
  if (length <= available()) {
    super::read(reinterpret_cast<uint8_t*>(tmp), super::getPayloadSize());
  }
  if (length <= availableForWrite()) {
    const int16_t *b = reinterpret_cast<const int16_t*>(buffer);
    uint16_t *t = reinterpret_cast<uint16_t*>(tmp);
    for (size_t i = 0; i < getBufferLength(); i++) {
      int16_t s = b[i];
      if (dcCutOffFrequency > 0) {
        const float rc = 1.0f / (2.0f * (float)M_PI * dcCutOffFrequency);
        const float dt = 1.0f / getSampRate();
        const float alpha = rc / (rc + dt);
        const float input = (float)s;
        const float filtered = alpha * (dcBlockPrevOutput + input - dcBlockPrevInput);
        dcBlockPrevInput = input;
        dcBlockPrevOutput = filtered;
        const int32_t clipped = std::max<int32_t>(INT16_MIN, std::min<int32_t>(INT16_MAX, (int32_t)lroundf(filtered)));
        s = (int16_t)clipped;
      }
      const uint16_t us = ((uint16_t)s)^0x8000U;  // XOR (signed -> unsigned)
      t[i * CH_NUM + channelIndexRL] = (dac_mode&I2S_DAC_CHANNEL_RIGHT_EN)?us:0;
      t[i * CH_NUM + (1-channelIndexRL)] = (dac_mode&I2S_DAC_CHANNEL_LEFT_EN)?us:0;
    }
    return super::write(reinterpret_cast<const uint8_t*>(tmp), super::getPayloadSize()) / CH_NUM;
  }
  return 0;
}

int Esp32BuiltinDacAudio::availableForWrite() {
  return super::availableForWrite() / CH_NUM;
}

int Esp32BuiltinDacAudio::available() {
  return 0;
}
