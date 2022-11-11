/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#ifndef LIB_ARDUINO_AUDIO_AUDIOIMPL_H_
#define LIB_ARDUINO_AUDIO_AUDIOIMPL_H_

#include "Audio.h"
#include <cstdint>

class AudioImpl: public virtual Audio{
 public:

 /**
   * @brief コンストラクタ
   * Codecの電源投入
   * @param [in] sampleRate sampling rate (Hz)
   * @param [in] bitDepth bit length (bit)
   * @param [in] bitLength aligned bit length (>=bitDepth)
   * @param [in] bufferMsec buffer length (msec)
   */
  AudioImpl(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec, std::uint8_t channelNum);
  AudioImpl(Audio* audio);

  /**
   * @return sampling rate (Hz)
   */
  std::uint16_t getSampRate() const override final;
  
  /**
   * @return bit length (bit)
   */
  std::uint8_t getBitDepth() const override final;
  
  /**
   * @return aligned bit length (bit)
   */
  std::uint8_t getAlignedBitLength() const override final;

  std::uint16_t getBufferMsec() const override final;

  std::uint8_t getChannelNum() const override final;

  /**
   * @return buffer length (samples)
   */
  std::size_t getBufferLength() const override final;

  virtual const std::size_t getPayloadSize() const override;

  virtual bool waitForWritable(std::uint32_t maxWaitMsec = UINT32_MAX) override;
  virtual bool waitForReadable(std::uint32_t maxWaitMsec = UINT32_MAX) override;

 private:
  const std::uint16_t sampleRate;
  const std::uint8_t bitDepth;
  const std::uint8_t bitLength;
  const std::uint16_t bufferMsec;
  const std::uint8_t channelNum;

  const std::size_t bufferLength;
  const std::size_t payloadLength;
};

#endif  // LIB_ARDUINO_AUDIO_AUDIOIMPL_H_
