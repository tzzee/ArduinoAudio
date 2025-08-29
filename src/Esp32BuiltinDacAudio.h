/**
 * @copyright 2021 NOEX inc.
 * @author Kenji Takahashi
 */

#ifndef LIB_ARDUINO_AUDIO_ESP32BUILTINDACAUDIO_H_
#define LIB_ARDUINO_AUDIO_ESP32BUILTINDACAUDIO_H_

#include "I2SAudio.h"

class Esp32BuiltinDacAudio : public I2SAudio {
  using super = I2SAudio;
 public:
  struct Esp32BuiltinDacAudioConfig {
    i2s_port_t port;
    i2s_pin_config_t pinConfig;
  };
  enum DacStatus {
    DacStarting, // 0->0.5
    DacRunning,  // 0.5
    DacStopping, // 0.5->0
    DacStopped,  // 0
  } dacStatus = DacStopped;
  const std::uint16_t dcCutOffFrequency;
  std::int16_t* highPassFilterArray;

  Esp32BuiltinDacAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec,
    uint8_t bufferCount, std::uint16_t dcCutOffFrequency = 0 /*0以上で有効、指定周波数以下をINT16_MINに貼り付け、スピーカーへ電圧がかかり続けるのを防止する*/,
    const Esp32BuiltinDacAudioConfig& config = {
      .port = (i2s_port_t)0,
      .pinConfig = {
        .bck_io_num = -1,
        .ws_io_num = -1,
        .data_out_num = -1,
        .data_in_num = -1
      },
    });

  /**
   * @brief デストラクタ
   * DSPの停止と、Codecの電源遮断
   */
  virtual ~Esp32BuiltinDacAudio();

  /**
   * @brief モジュールの準備。DSPへFWを書き込む。電源投入後一回だけ行うこと。
   */
  virtual void begin() override;

  /**
   * @brief 音声出力の開始。以降writeを途切れさせないこと
   */
  virtual void start() override;

  /**
   * @brief 音声出力の停止。
   */
  virtual void stop() override;

  virtual void zero() override;
  
  void fill(int16_t v);

  virtual const std::size_t getPayloadSize() const override;

  /**
   * @brief バッファを読み込み、音声を受け取る
   * @param [out] buf 録音データを保持するバッファ。
   * @param [in] size 録音データ長。
   * @return 読み込んだデータのバイト数。
   */
  std::size_t read(std::uint8_t* buf, std::size_t size) override;

  /**
   * @brief バッファを書き込み、音声を再生する
   * @param [in] buf 再生データを保持するバッファ。
   * @param [in] size 再生データ長。
   * @return 書き込んだデータのバイト数。
   */
  std::size_t write(const std::uint8_t* buf, std::size_t size) override;

  /**
   * @brief 読み込み可能長さを受け取る
   * @return 読み込み可能長さ
   */
  int available() override;

  /**
   * @brief 書き込み可能長さを受け取る
   * @return 書き込み可能長さ
   */
  int availableForWrite() override;
};

#endif  // LIB_ARDUINO_AUDIO_ESP32BUILTINDACAUDIO_H_
