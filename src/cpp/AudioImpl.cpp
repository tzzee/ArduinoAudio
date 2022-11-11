/**
 * @copyright 2022 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../AudioImpl.h"
#include <Arduino.h>

AudioImpl::AudioImpl(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t bitLength, std::uint16_t bufferMsec, std::uint8_t channelNum):
  sampleRate(sampleRate), bitDepth(bitDepth), bitLength(bitLength), bufferMsec(bufferMsec), channelNum(channelNum),
  bufferLength(((std::uint32_t)sampleRate * bufferMsec) / 1000),
  payloadLength((channelNum*bufferLength)*((bitLength+7)/8)) {
}

AudioImpl::AudioImpl(Audio* audio) :
  sampleRate(audio->getSampRate()), bitDepth(audio->getBitDepth()), bitLength(audio->getAlignedBitLength()), bufferMsec(audio->getBufferMsec()), channelNum(audio->getChannelNum()),
  bufferLength(audio->getBufferLength()),
  payloadLength(audio->getPayloadSize()){
}

std::uint16_t AudioImpl::getSampRate() const {
  return sampleRate;
}

std::uint8_t AudioImpl::getBitDepth() const {
  return bitDepth;
}

std::uint8_t AudioImpl::getAlignedBitLength() const {
  return bitLength;
}

std::uint16_t AudioImpl::getBufferMsec() const {
  return bufferMsec;
}

std::uint8_t AudioImpl::getChannelNum() const {
  return channelNum;
}

std::size_t AudioImpl::getBufferLength() const {
  return bufferLength;
}

std::size_t const AudioImpl::getPayloadSize() const {
  return payloadLength;
}

bool AudioImpl::waitForWritable(std::uint32_t /*maxWaitMsec*/) {
  const bool writable = getPayloadSize() <= availableForWrite();
  if(!writable) {
    delay(1);
  }
  return writable;
}

bool AudioImpl::waitForReadable(std::uint32_t /*maxWaitMsec*/) {
  const bool readable = getPayloadSize() <= available();
  if(!readable) {
    delay(1);
  }
  return readable;
}
