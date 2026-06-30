/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#ifndef LIB_ARDUINO_AUDIO_I2SAUDIO_H_
#define LIB_ARDUINO_AUDIO_I2SAUDIO_H_

#include "AudioImpl.h"
#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>

// #define I2S_LEGACY_API_ENABLED

class I2SAudio: public AudioImpl{
  using super = AudioImpl;

 public:
  struct I2SAudioConfig {
    i2s_port_t port;
    i2s_mode_t mode;
    i2s_channel_fmt_t chFormat;
    i2s_comm_format_t comFormat; 
    bool txDescAutoClear;
    i2s_pin_config_t pinConfig;
  };
  /**
   * @brief I2S 音声入出力を初期化する
   * @param [in] sampleRate サンプリング周波数。
   * @param [in] bitDepth 実ビット深度。
   * @param [in] alignedBitLength アライン後のビット長。
   * @param [in] bufferMsec DMA 1 本あたりの時間長。
   * @param [in] channelNum チャンネル数。
   * @param [in] bufferCount DMA バッファ本数。
   * @param [in] config I2S ハードウェア設定。
   * @param [in] ringBufferCount ソフトウェア TX リング本数。0 のときは bufferCount を使う。
   */
  I2SAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec, std::uint8_t channelNum,
    std::uint8_t bufferCount, const I2SAudioConfig& config, std::uint8_t ringBufferCount = 0);
  virtual ~I2SAudio();
  virtual void begin() override;
  virtual void start() override;
  virtual void stop() override;
  virtual void zero() override;
  /**
   * @brief リングに積まれたフルスロット分の送出を明示的に開始する
   */
  virtual void flush() override;

  /**
   * @brief バッファを読み込み、音声を受け取る
   * @param [out] buf 録音データを保持するバッファ。
   * @param [in] size 録音データ長。
   * @return 読み込んだデータのバイト数。
   */
  virtual std::size_t read(std::uint8_t* buf, std::size_t size) override;

  /**
   * @brief バッファを書き込み、音声を再生する
   * @param [in] buf 再生データを保持するバッファ。
   * @param [in] size 再生データ長。
   * @return 書き込んだデータのバイト数。
   */
  virtual std::size_t write(const std::uint8_t* buf, std::size_t size) override;

  virtual int available() override;
  virtual int availableForWrite() override;

  uint8_t getBufferCount() const override;

  virtual bool waitForWritable(std::uint32_t maxWaitMsec = UINT32_MAX) override;
  virtual bool waitForReadable(std::uint32_t maxWaitMsec = UINT32_MAX) override;

 protected:
  enum I2SAudioStatus {
    I2SAudioStop,
    I2SAudioOscillation,
    I2SAudioStart
  };
  void _start(I2SAudioStatus s);
  /**
   * @brief TX リングが空になった直後に DMA へ low-level な無音データを書き込む
   */
  virtual void handleTxIdle();

  /**
   * @brief DMA バッファを low-level にゼロクリアする
   */
  void zeroTxDmaBuffer();

  /**
   * @brief 完成済み TX payload を low-level に DMA へ直接書き込む
   * @param [in] payload I2S 送信用 payload。
   * @param [in] length payload 長。
   * @param [in] repeatCount 同じ payload を繰り返す回数。
   * @return すべて書き込めたとき true。
   */
  bool writeTxDmaBuffer(const std::uint8_t* payload, std::size_t length, std::uint8_t repeatCount = 1);
  
 private:
  
  bool _eventQueue(TickType_t);
  bool _recvQueue(i2s_event_type_t type);
  std::uint8_t getRingBufferCount() const;

  const I2SAudioConfig audioConfig;
  const i2s_config_t i2sConfig;
  const std::uint8_t ringBufferCount;


  volatile I2SAudioStatus status;

  // TX リングバッファ: DMA への直接書き込みを廃止し、ソフトウェアバッファ経由でドレイン
  char *ringTxBuffer;    ///< getRingBufferCount() スロット分のヒープ領域
  int ringTxReadIdx;
  int ringTxWriteIdx;
  int ringTxCount;       ///< 現在リングにあるバッファ数
  bool txPrimed;         ///< true の間だけリングから DMA へドレインする
  bool handlingTxIdle;
  bool txIdleFilled;

  std::size_t rxFilled;
  bool rxDone;
  char *rxBuffer;

  xQueueHandle i2s_event_queue;
};

#endif  // LIB_ARDUINO_AUDIO_I2SAUDIO_H_
