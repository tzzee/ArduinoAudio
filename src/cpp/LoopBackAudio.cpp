/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../LoopBackAudio.h"
#include <Arduino.h>
#include <string.h>
#include <esp32-hal.h>

static xSemaphoreHandle xSemaphore = xSemaphoreCreateMutex();

LoopBackAudio::LoopBackAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint16_t bufferMsec, std::uint8_t channelNum):
  super(sampleRate, bitDepth, bitDepth, bufferMsec, channelNum),
  echoBuffer(new std::uint8_t[getPayloadSize()]) {
}

LoopBackAudio::~LoopBackAudio() {
  delete[] echoBuffer;
}

void LoopBackAudio::begin() {}
void LoopBackAudio::start() {
  readable = false;
  writeTargetMsec = millis();
}
void LoopBackAudio::stop() {}
void LoopBackAudio::zero() {
  memset(echoBuffer, 0, getPayloadSize());
}
int LoopBackAudio::availableForWrite() {
  if ((int)writeTargetMsec - (int)millis() <= 0) {
    return getPayloadSize();
  } else {
    return 0;
  }
}
int LoopBackAudio::available() {
  if (readable) {
    return getPayloadSize();
  } else {
    return 0;
  }
}
size_t LoopBackAudio::read(std::uint8_t *buffer, std::size_t length) {
  log_v("%d %d", length, available());
  // length==getBufferLength()*bitRate/8
  xSemaphoreTake(xSemaphore, portMAX_DELAY);
  if (readable) {
    readable = false;
    memcpy(buffer, echoBuffer, length);
    xSemaphoreGive(xSemaphore);
    return length;
  } else {
    xSemaphoreGive(xSemaphore);
    return 0;
  }
}
size_t LoopBackAudio::write(const uint8_t *buffer, size_t length) {
  log_v("%d %d", length, availableForWrite());
  // length==getBufferLength()*bitRate/8
  if (length <= availableForWrite()) {
    const uint32_t bufferMsec = 1000 * length * 8 / (getAlignedBitLength() * getSampRate());
    writeTargetMsec += bufferMsec;
    xSemaphoreTake(xSemaphore, portMAX_DELAY);
    memcpy(echoBuffer, buffer, length);
    readable = true;
    xSemaphoreGive(xSemaphore);
    return length;
  } else {
    return 0;
  }
}
