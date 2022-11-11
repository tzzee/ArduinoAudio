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
    i2s_pin_config_t pinConfig;
  };
  I2SAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec, std::uint8_t channelNum,
    std::uint8_t bufferCount, const I2SAudioConfig& config);
  virtual ~I2SAudio();
  virtual void start() override;
  virtual void stop() override;
  virtual void zero() override;

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

  uint8_t getBufferCount() const;

  virtual bool waitForWritable(std::uint32_t maxWaitMsec = UINT32_MAX) override;
  virtual bool waitForReadable(std::uint32_t maxWaitMsec = UINT32_MAX) override;

 protected:
  enum I2SAudioStatus {
    I2SAudioStop,
    I2SAudioOscillation,
    I2SAudioStart
  };
  void _start(I2SAudioStatus s);
  
 private:
  
  bool _eventQueue(TickType_t);
  bool _recvQueue(i2s_event_type_t type);

  const I2SAudioConfig audioConfig;
  const i2s_config_t i2sConfig;


  volatile I2SAudioStatus status;

  std::size_t txEmpty;
  std::size_t rxFilled;
  bool txDone;
  bool rxDone;
  char *txBuffer;
  char *rxBuffer;

  xQueueHandle i2s_event_queue;
};

#endif  // LIB_ARDUINO_AUDIO_I2SAUDIO_H_
