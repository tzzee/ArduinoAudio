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

// 片側の DAC だけを使うときはモノラル(ONLY_RIGHT/ONLY_LEFT)で送り、DMA のデータを半分にする。
// ESP32 の I2S は 16bit のモノラルで隣り合う 2 サンプルが入れ替わって出るため、書く位置(i ^ 1)で打ち消す。
// IDF 4.4(arduino-esp32 2.0.17)で確認済み。それ以外、または ARDUINO_AUDIO_BUILTIN_DAC_STEREO を定義したときはステレオで送る
#if defined(IDF_VER) && !defined(ARDUINO_AUDIO_BUILTIN_DAC_STEREO)
static const bool builtin_dac_mono_supported = true;
#else
static const bool builtin_dac_mono_supported = false;
#endif

/**
 * @brief DAC 出力チャンネル設定からモノラルで送るかを決める
 * @param [in] dac_mode DAC 出力チャンネル設定
 * @return モノラルで送るとき true
 */
static bool isMonoDacMode(i2s_dac_mode_t dac_mode) {
  return builtin_dac_mono_supported &&
      (dac_mode == I2S_DAC_CHANNEL_RIGHT_EN || dac_mode == I2S_DAC_CHANNEL_LEFT_EN);
}

/**
 * @brief DAC 出力チャンネル設定から I2S のチャンネル数を決める
 * @param [in] dac_mode DAC 出力チャンネル設定
 * @return チャンネル数(モノラル 1、ステレオ 2)
 */
static std::uint8_t channelNumOf(i2s_dac_mode_t dac_mode) {
  return isMonoDacMode(dac_mode) ? 1 : 2;
}

/**
 * @brief DAC 出力チャンネル設定から I2S のチャンネル形式を決める
 * @param [in] dac_mode DAC 出力チャンネル設定
 * @return チャンネル形式
 */
static i2s_channel_fmt_t channelFormatOf(i2s_dac_mode_t dac_mode) {
  if (!isMonoDacMode(dac_mode)) {
    return I2S_CHANNEL_FMT_RIGHT_LEFT;
  }
  return dac_mode == I2S_DAC_CHANNEL_RIGHT_EN ? I2S_CHANNEL_FMT_ONLY_RIGHT : I2S_CHANNEL_FMT_ONLY_LEFT;
}

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
    super(sampleRate, bitDepth, alignedBitLength, bufferMsec, channelNumOf(dac_mode),
    bufferCount,
    I2SAudioConfig{
      .port = config.port,
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .chFormat = channelFormatOf(dac_mode),
      .comFormat = builtin_dac_comm_format,
      .txDescAutoClear = false,
      .pinConfig = config.pinConfig
    }, ringBufferCount), dac_mode(dac_mode), dcCutOffFrequency(dcCutOffFrequency) {
}

void Esp32BuiltinDacAudio::putSample(std::uint16_t* t, std::size_t i, std::uint16_t us) const {
  if (getChannelNum() == 1) {
    // 隣り合う 2 サンプルが入れ替わって出るので、前後を入れ替えて書く(長さが奇数のときの末尾はそのまま)
    const std::size_t j = i ^ 1U;
    t[(j < getBufferLength()) ? j : i] = us;
  } else {
    t[i * 2 + channelIndexRL] = (dac_mode & I2S_DAC_CHANNEL_RIGHT_EN) ? us : 0;
    t[i * 2 + (1 - channelIndexRL)] = (dac_mode & I2S_DAC_CHANNEL_LEFT_EN) ? us : 0;
  }
}

esp_err_t Esp32BuiltinDacAudio::setPin(const i2s_pin_config_t* pin) {
  if (pin == NULL && getChannelNum() == 1) {
    return i2s_set_dac_mode(dac_mode);
  }
  return super::setPin(pin);
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
  // DMA が空くのを待たない。待つと呼び出し元(availableForWrite() など)が最大で DMA 本数ぶん止まり、
  // 同じタスクで回している処理(j1772 では ADC の緊急遮断の判定)が遅れる。
  // 書けなければ false を返し、呼び出し元が txIdlePendingCount を残して次の TX_DONE で書き直す。
  const TickType_t tx_ticks_to_wait = 0;
  if (dacStatus != DacRunning) {
    log_d("handleTxIdle skip status=%d", dacStatus);
    return false;
  } else if (dcCutOffFrequency == 0) {
    uint8_t tmp[super::getPayloadSize()];
    uint16_t *t = reinterpret_cast<uint16_t*>(tmp);
    for (size_t i = 0; i < getBufferLength(); i++) {
      putSample(t, i, 0x8000U);
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
      putSample(t, i, ((uint16_t)(int16_t)lroundf(offset)) ^ 0x8000U);
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
  return super::getPayloadSize() / getChannelNum();
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
      putSample(t, i, ((uint16_t)s)^0x8000U);  // XOR (signed -> unsigned)
    }
    return super::write(reinterpret_cast<const uint8_t*>(tmp), super::getPayloadSize()) / getChannelNum();
  }
  return 0;
}

int Esp32BuiltinDacAudio::availableForWrite() {
  return super::availableForWrite() / getChannelNum();
}

int Esp32BuiltinDacAudio::available() {
  return 0;
}
