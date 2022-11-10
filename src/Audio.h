/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#ifndef LIB_ARDUINO_AUDIO_AUDIO_H_
#define LIB_ARDUINO_AUDIO_AUDIO_H_

#include <cstdint>

class Audio {
 public:
  virtual void begin() = 0;
  virtual void start() = 0;
  virtual void stop() = 0;
  virtual void zero() = 0;
  virtual int availableForWrite() = 0;
  virtual int available() = 0;

  /**
   * @brief read from mic
   * @param [in] buffer audio buffer
   * @param [in] byteLength buffer length (bytes)
   * @return buffer length (bytes)
   */
  virtual std::size_t read(std::uint8_t *buffer, std::size_t byteLength) = 0;

  /**
   * @brief write to speaker
   * @param [in] buffer audio buffer
   * @param [in] byteLength buffer length (bytes)
   * @return buffer length (bytes)
   */
  virtual std::size_t write(const std::uint8_t *buffer, std::size_t byteLength) = 0;

  /**
   * @return sampling rate (Hz)
   */
  virtual std::uint16_t getSampRate() const = 0;
  
  /**
   * @return bit length (bit)
   */
  virtual std::uint8_t getBitDepth() const = 0;
  
  /**
   * @return aligned bit length (bits)
   */
  virtual std::uint8_t getAlignedBitLength() const = 0;

  /**
   * @return buffer length (msecs)
   */
  virtual std::uint16_t getBufferMsec() const = 0;

  virtual std::uint8_t getChannelNum() const = 0;

  /**
   * @return buffer length (samples)
   */
  virtual std::size_t getBufferLength() const = 0;

  /**
   * @return buffer length (bytes)
   */
  virtual const std::size_t getPayloadSize() const = 0;
};

#endif  // LIB_ARDUINO_AUDIO_AUDIO_H_
