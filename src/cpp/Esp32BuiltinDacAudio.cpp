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
      .txDescAutoClear = false,
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
  if (dcCutOffFrequency != 0) {
    // 無音中は 0V で待つ。中点へは write() が音を書き始めたときに段階的に上げる。
    // super::start() の中の handleTxIdle() で 0V を書かせるため、先に DacRunning にする
    dacStatus = DacRunning;
    dcOffset = INT16_MIN;
    super::start();
  } else {
    super::start(); // 中でzeroが呼ばれる
    dacStatus = DacRunning;
    zero();  // DACの出力を中立にする
  }
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
    flush();
  } break;
  case DacRunning: {
    if (dcCutOffFrequency != 0) {
      // 中点で埋めると 0V から跳ね上がってボツになる。無音への移行(中点から 0V へ段階的に下げる)に任せる
      if (!purge()) {
        log_d("zero: purge skipped");
      }
    } else {
      fill(0);
    }
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
  const TickType_t tx_ticks_to_wait = pdMS_TO_TICKS(getBufferMsec() * getBufferCount());
  if (dacStatus != DacRunning) {
    log_d("handleTxIdle skip status=%d", dacStatus);
    return false;
  } else if (dcCutOffFrequency == 0) {
    uint8_t tmp[super::getPayloadSize()];
    uint16_t *t = reinterpret_cast<uint16_t*>(tmp);
    for (size_t i = 0; i < getBufferLength(); i++) {
      t[i * CH_NUM + channelIndexRL] = (dac_mode & I2S_DAC_CHANNEL_RIGHT_EN) ? 0x8000U : 0;
      t[i * CH_NUM + (1 - channelIndexRL)] = (dac_mode & I2S_DAC_CHANNEL_LEFT_EN) ? 0x8000U : 0;
    }
    const bool wrote = writeTxDmaBuffer(tmp, sizeof(tmp), 1, tx_ticks_to_wait);
    log_v("handleTxIdle midpoint wrote=%d", wrote);
    return wrote;
  } else {
    // 中点から 0V へ段階的に下げ、下げ終えたあとは 0V を書く。
    // 書けなかったときに次の呼び出しで同じ区間を書き直せるよう、書けてから dcOffset を進める
    uint8_t tmp[super::getPayloadSize()];
    uint16_t *t = reinterpret_cast<uint16_t*>(tmp);
    const float step = getDcOffsetStep();
    float offset = dcOffset;
    for (size_t i = 0; i < getBufferLength(); i++) {
      offset = std::max<float>(INT16_MIN, offset - step);
      const uint16_t us = ((uint16_t)(int16_t)lroundf(offset)) ^ 0x8000U;
      t[i * CH_NUM + channelIndexRL] = (dac_mode & I2S_DAC_CHANNEL_RIGHT_EN) ? us : 0;
      t[i * CH_NUM + (1 - channelIndexRL)] = (dac_mode & I2S_DAC_CHANNEL_LEFT_EN) ? us : 0;
    }
    const bool wrote = writeTxDmaBuffer(tmp, sizeof(tmp), 1, tx_ticks_to_wait);
    if (wrote) {
      dcOffset = offset;
      if (dcOffset <= INT16_MIN) {
        // 0V に下げ切ったら、次の音をハイパスの過去の値に引きずられずに始める
        dcBlockPrevInput = 0.0f;
        dcBlockPrevOutput = 0.0f;
      }
    }
    log_v("handleTxIdle ramp-down offset=%d wrote=%d", (int)offset, wrote);
    return wrote;
  }
}

std::uint8_t Esp32BuiltinDacAudio::getTxIdlePayloadCount() const {
  if (dcCutOffFrequency == 0) {
    return super::getTxIdlePayloadCount();
  }
  // 下げる分 + DMA 本数(DMA の記述子をすべて 0V で埋める)
  const std::uint32_t ramp_payloads = (kDcOffsetRampMsec + getBufferMsec() - 1) / getBufferMsec() + 1;
  return (std::uint8_t)std::min<std::uint32_t>(UINT8_MAX, ramp_payloads + getBufferCount());
}

float Esp32BuiltinDacAudio::getDcOffsetStep() const {
  // 1 サンプルあたりに動かす量。kDcOffsetRampMsec で 0V(INT16_MIN)と中点(0)の間を移る
  return 32768.0f * 1000.0f / ((float)kDcOffsetRampMsec * (float)getSampRate());
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
  flush();
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
        // 0V で待っていたときは、音を書き始めたところから中点へ段階的に上げる
        dcOffset = std::min(0.0f, dcOffset + getDcOffsetStep());
        const int32_t clipped = std::max<int32_t>(INT16_MIN, std::min<int32_t>(INT16_MAX, (int32_t)lroundf(filtered + dcOffset)));
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
