/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../DummyAudio.h"

DummyAudio::DummyAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint16_t bufferMsec, std::uint8_t channelNum):
  super(sampleRate, bitDepth, bitDepth, bufferMsec, channelNum) {
}
void DummyAudio::begin() {}
void DummyAudio::start() {}
void DummyAudio::stop() {}
void DummyAudio::zero() {}
int DummyAudio::availableForWrite() {
  return getPayloadSize();
}
int DummyAudio::available() {
  return getPayloadSize();
}
std::size_t DummyAudio::read(std::uint8_t *buffer, std::size_t length) {
  return length;
}
std::size_t DummyAudio::write(const uint8_t *buffer, std::size_t length) {
  return length;
}