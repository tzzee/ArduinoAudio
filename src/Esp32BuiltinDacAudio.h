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
  const i2s_dac_mode_t dac_mode;
  const std::uint16_t dcCutOffFrequency;
  float dcBlockPrevInput = 0.0f;
  float dcBlockPrevOutput = 0.0f;
  /// dcCutOffFrequency 有効時に、出力へ足す直流(signed 16bit)。INT16_MIN で 0V、0 で中点
  float dcOffset = INT16_MIN;
  /// dcCutOffFrequency 有効時に、0V と中点の間を移るのにかける時間
  static constexpr std::uint16_t kDcOffsetRampMsec = 100;

  /**
   * @brief ESP32内蔵DAC用I2S出力を初期化する
   * @param [in] sampleRate サンプリング周波数。
   * @param [in] bitDepth 実ビット深度。
   * @param [in] alignedBitLength アライン後のビット長。
   * @param [in] bufferMsec DMA 1 本あたりの時間長。
   * @param [in] bufferCount DMA バッファ本数。
   * @param [in] dac_mode DAC 出力チャンネル設定。片側だけのときはモノラル、両側のときはステレオで送る。
   * @param [in] dcCutOffFrequency 低域カット設定。
   * @param [in] config I2S ポート設定。
   * @param [in] ringBufferCount ソフトウェア TX リング本数。0 のときは bufferCount を使う。
   */
  Esp32BuiltinDacAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec,
    uint8_t bufferCount, i2s_dac_mode_t dac_mode = I2S_DAC_CHANNEL_RIGHT_EN, std::uint16_t dcCutOffFrequency = 0 /*0以上で有効、指定周波数以下をINT16_MINに貼り付け、スピーカーへ電圧がかかり続けるのを防止する*/,
    const Esp32BuiltinDacAudioConfig& config = {
      .port = (i2s_port_t)0,
      .pinConfig = {
        .bck_io_num = -1,
        .ws_io_num = -1,
        .data_out_num = -1,
        .data_in_num = -1
      }
    },
    uint8_t ringBufferCount = 0);

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
   * @details dcCutOffFrequency 有効時は 0V で待ち、write() で音を書き始めたときに
   * kDcOffsetRampMsec かけて中点へ上げる。無音に戻るときは handleTxIdle() で同じ時間をかけて 0V へ下げる
   */
  virtual void start() override;

  /**
   * @brief 音声出力の停止。
   */
  virtual void stop() override;

  /**
   * @brief DAC 状態に応じた無音データを書き込む
   * @details dcCutOffFrequency 有効時の DacRunning は中点で埋めず、purge() で
   * 中点から 0V へ段階的に下げる無音への移行に任せる
   */
  virtual void zero() override;

  /**
   * @brief 再生中に TX が空になったときの無音 payload を DMA へ直接補充する
   * DC カット無効時は、内蔵 DAC の中点を維持するデータを送る。
   */
  bool handleTxIdle() override;

  /**
   * @brief 1 サンプルを、DMA に送るバッファの決まった位置へ置く
   * @details モノラルでは隣り合う 2 サンプルの前後を入れ替えて置く。ステレオでは使う側に置き、反対側は 0 にする
   * @param [out] t DMA に送るバッファ(getBufferLength() サンプル × チャンネル数)
   * @param [in] i サンプル番号
   * @param [in] us unsigned 16bit のサンプル値(上位 8bit が DAC に出る)
   */
  void putSample(std::uint16_t* t, std::size_t i, std::uint16_t us) const;

  /**
   * @brief TX が空になったあとに handleTxIdle() で書く payload の本数
   * @details dcCutOffFrequency 有効時は、中点から 0V へ下げる分を DMA 本数に足す
   * @return payload 本数
   */
  std::uint8_t getTxIdlePayloadCount() const override;

  /**
   * @brief I2S のピンを設定する
   * @details モノラルで送るときは使う側の DAC だけを有効にする。モノラルでは左右に同じデータが出るため、
   * i2s_set_pin(NULL) で両方を有効にすると反対側のピンにも音が出る。反対側のピンには触れない
   * @param [in] pin ピン設定。NULL のときは内蔵 DAC
   * @return 設定の結果
   */
  esp_err_t setPin(const i2s_pin_config_t* pin) override;

  /**
   * @brief dcCutOffFrequency 有効時に、1 サンプルあたりに直流を動かす量
   * @return signed 16bit 単位の量
   */
  float getDcOffsetStep() const;
  
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
