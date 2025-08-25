/**
 * @copyright 2021 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../Esp32BuiltinDacAudio.h"
#include <assert.h>
#include <esp32-hal.h>

#define CH_NUM 2

#if defined(ARDUINO_AUDIO_SWAPCHANNEL_DISABLED) || defined(PLATFORMIO)
static const int channelIndexRL = 1;
#else
static const int channelIndexRL = 0;
#endif

Esp32BuiltinDacAudio::Esp32BuiltinDacAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec,
  uint8_t bufferCount, const Esp32BuiltinDacAudioConfig& config): 
    super(sampleRate, bitDepth, alignedBitLength, bufferMsec, CH_NUM,
    bufferCount,
    {
      .port = config.port,
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .chFormat = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .comFormat = I2S_COMM_FORMAT_I2S_LSB,
      .pinConfig = config.pinConfig
    }){
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
  super::start(); // 中でzeroが呼ばれる
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
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
    int16_t *t = reinterpret_cast<int16_t*>(tmp);
    for (size_t i = 0; i < getBufferLength(); i++) {
      const uint16_t us = ((uint16_t)b[i])^0x8000U;
      t[i * CH_NUM + channelIndexRL] = (int16_t)us;
      t[i * CH_NUM + (1-channelIndexRL)] = 0;
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
