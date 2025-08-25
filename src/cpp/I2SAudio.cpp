/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../I2SAudio.h"
#include <driver/rtc_io.h>
#include <string.h>

#include <Arduino.h>

static void initRtcPin(int pin) {
  if(0<=pin) {
    rtc_gpio_init((gpio_num_t)pin);
    rtc_gpio_set_direction((gpio_num_t)pin, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level((gpio_num_t)pin, 1);
  }
}

static void deinitRtcPin(int pin) {
  if(0<=pin) {
    rtc_gpio_set_direction((gpio_num_t)pin, RTC_GPIO_MODE_DISABLED);
    rtc_gpio_deinit((gpio_num_t)pin);
  }
}

I2SAudio::I2SAudio(std::uint16_t sampleRate, std::uint8_t bitDepth, std::uint8_t alignedBitLength, std::uint16_t bufferMsec, std::uint8_t channelNum,
    uint8_t bufferCount, const I2SAudioConfig& audioConfig)
    : super(sampleRate, bitDepth, alignedBitLength, bufferMsec, channelNum),
      audioConfig(audioConfig),
      i2sConfig({
        .mode = audioConfig.mode,
        .sample_rate = getSampRate(),
        .bits_per_sample = (i2s_bits_per_sample_t)getBitDepth(),
        .channel_format = audioConfig.chFormat,
        .communication_format = audioConfig.comFormat,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = bufferCount,
        .dma_buf_len = (int)getBufferLength() ,
        // .use_apll = false
      }),
      status(I2SAudioStop) {
  txBuffer = new char[I2SAudio::getPayloadSize()];  // virtualではなく、自分を呼ぶ
  rxBuffer = new char[I2SAudio::getPayloadSize()];  // virtualではなく、自分を呼ぶ
  initRtcPin(audioConfig.pinConfig.bck_io_num);
  initRtcPin(audioConfig.pinConfig.ws_io_num);
  initRtcPin(audioConfig.pinConfig.data_out_num);
}

I2SAudio::~I2SAudio() {
  I2SAudio::stop();  // virtualではなく、自分を呼ぶ
  delete[] txBuffer;
  delete[] rxBuffer;
}

std::uint8_t I2SAudio::getBufferCount() const {
  return i2sConfig.dma_buf_count;
}

void I2SAudio::begin() {
  ESP_ERROR_CHECK(i2s_driver_install(audioConfig.port, &i2sConfig, getBufferCount()*2, &i2s_event_queue));
}

void I2SAudio::start() {
  _start(I2SAudioStart);
  if (((uint8_t)i2sConfig.mode & (uint8_t)I2S_MODE_TX) == (uint8_t)I2S_MODE_TX) {
    txEmpty = getBufferCount();
  }
  if (((uint8_t)i2sConfig.mode & (uint8_t)I2S_MODE_RX) == (uint8_t)I2S_MODE_RX) {
    rxFilled = getBufferCount();
  }
}

//  start DAC
//  DAC_Start()後は、DMAバッファが空になる前にDAC_Write()で出力データを書き込むこと
void I2SAudio::_start(I2SAudioStatus s) {
  I2SAudio::stop();  // virtualではなく、自分を呼ぶ
  deinitRtcPin(audioConfig.pinConfig.bck_io_num);
  deinitRtcPin(audioConfig.pinConfig.ws_io_num);
  deinitRtcPin(audioConfig.pinConfig.data_out_num);

  const bool hasPinConfig = 0<=audioConfig.pinConfig.bck_io_num ||
                      0<=audioConfig.pinConfig.ws_io_num ||
                      0<=audioConfig.pinConfig.data_out_num ||
                      0<=audioConfig.pinConfig.data_in_num;

  i2s_set_pin(audioConfig.port, hasPinConfig?&(audioConfig.pinConfig):NULL);
  switch (i2sConfig.channel_format) {
    case I2S_CHANNEL_FMT_RIGHT_LEFT:
    case I2S_CHANNEL_FMT_ALL_RIGHT:
    case I2S_CHANNEL_FMT_ALL_LEFT: {
      i2s_set_clk(audioConfig.port, i2sConfig.sample_rate, i2sConfig.bits_per_sample, I2S_CHANNEL_STEREO);
    } break;
    case I2S_CHANNEL_FMT_ONLY_RIGHT:
    case I2S_CHANNEL_FMT_ONLY_LEFT: {
      i2s_set_clk(audioConfig.port, i2sConfig.sample_rate, i2sConfig.bits_per_sample, I2S_CHANNEL_MONO);
    } break;
  }
  ESP_ERROR_CHECK(i2s_start(audioConfig.port));
  txDone = false;
  rxDone = false;
  txEmpty = 0;
  rxFilled = 0;
  zero();
  status = s;
}

//  stop DAC
//  データ出力後、DMAバッファが空になる前にこの関数を呼び出すこと
void I2SAudio::stop() {
  if (status != I2SAudioStop) {
    i2s_stop(audioConfig.port);
    initRtcPin(audioConfig.pinConfig.bck_io_num);
    initRtcPin(audioConfig.pinConfig.ws_io_num);
    initRtcPin(audioConfig.pinConfig.data_out_num);
    status = I2SAudioStop;
  }
}

void I2SAudio::zero() {
  i2s_zero_dma_buffer(audioConfig.port);
  txEmpty = getBufferCount();
}

bool I2SAudio::_recvQueue(i2s_event_type_t type) {
  switch (type) {
    case I2S_EVENT_DMA_ERROR: {
      log_e("I2C: Error");
    } break;
    case I2S_EVENT_TX_DONE: {
      // All buffers are empty. This means we have an underflow on our hands.
      txEmpty = getBufferCount();
#if 0
      if(!txDone) {
        memset(txBuffer, 0, I2SAudio::getPayloadSize());
        txDone = true;
      }
#endif
      return true;
    } break;
    case I2S_EVENT_RX_DONE: {
      // All buffers are full. This means we have an overflow.
      rxFilled = getBufferCount();
      return true;
    } break;
    default: { } break;
  }
  return false;
}

bool I2SAudio::_eventQueue(TickType_t ticks_to_wait) {
  const std::uint32_t startMsec = millis();
  static std::uint32_t lastMsec = millis();
  i2s_event_t event;
  log_v("%d", uxQueueMessagesWaiting(i2s_event_queue));

  if(txEmpty || rxFilled) {
    ticks_to_wait = 0;
  }
  const std::uint32_t elapsedMsec = startMsec - lastMsec;
  if (xQueueReceive(i2s_event_queue, &event, std::min((TickType_t)getBufferMsec()*getBufferCount(), ticks_to_wait)) == pdTRUE) {
    bool done = false;
    done |= _recvQueue(event.type);
    while (xQueueReceive(i2s_event_queue, &event, 0) == pdTRUE) {
      // recv all queue
      done |= _recvQueue(event.type);
    }
    if(done) lastMsec = millis();
  } else if((std::uint32_t)getBufferMsec()*getBufferCount()<=elapsedMsec){
    // must be empty
    log_w("i2s: event timeout");
    if (((uint8_t)i2sConfig.mode & (uint8_t)I2S_MODE_TX) == (uint8_t)I2S_MODE_TX) {
      txEmpty = getBufferCount();
    }
    if (((uint8_t)i2sConfig.mode & (uint8_t)I2S_MODE_RX) == (uint8_t)I2S_MODE_RX) {
      rxFilled = getBufferCount();
    }
    lastMsec = millis();
  }

  // write
  if(txEmpty && txDone) {
#ifdef I2S_LEGACY_API_ENABLED
    int bytesWritten = i2s_write_bytes(audioConfig.port, (const char *)txBuffer, I2SAudio::getPayloadSize(), ticks_to_wait);
#else
    std::size_t bytesWritten = 0;
    esp_err_t ret = i2s_write(audioConfig.port, (const char *)txBuffer, I2SAudio::getPayloadSize(), &bytesWritten, ticks_to_wait);
    if (ret != ESP_OK) {
      bytesWritten = 0;
    }
#endif
    if (I2SAudio::getPayloadSize() <= bytesWritten) {
      txEmpty--;
      txDone = false;
      lastMsec = millis();
    } else {
      txEmpty = 0;
    }
  }

  // read
  if(rxFilled && !rxDone) {
#ifdef I2S_LEGACY_API_ENABLED
    int bytesRead = i2s_read_bytes(audioConfig.port, (char *)rxBuffer, I2SAudio::getPayloadSize(), ticks_to_wait);
#else
    std::size_t bytesRead = 0;
    esp_err_t ret = i2s_read(audioConfig.port, reinterpret_cast<char *>(rxBuffer), I2SAudio::getPayloadSize(), &bytesRead, ticks_to_wait);
    if (ret != ESP_OK) {
      bytesRead = 0;
    }
#endif
    if (I2SAudio::getPayloadSize() <= bytesRead) {
      rxFilled--;
      rxDone = true;  // バッファにreadが入っている
      lastMsec = millis();
    } else {
      rxFilled = 0;
    }
  }

  return true;
}


size_t I2SAudio::read(std::uint8_t* buffer, std::size_t length) {
  size_t s = 0;
  if (!rxDone) {
    _eventQueue(0);
  }
  if (rxDone) {
    memcpy(buffer, rxBuffer, I2SAudio::getPayloadSize());
    rxDone = false;
    s = I2SAudio::getPayloadSize();
  }
  return s;
}

size_t I2SAudio::write(const std::uint8_t* buffer, std::size_t length) {
  size_t s = 0;
  if (length <= I2SAudio::getPayloadSize()) {
    memcpy(txBuffer, buffer, I2SAudio::getPayloadSize());
    txDone = true;
    s = I2SAudio::getPayloadSize();
  }
  _eventQueue(0);
  return s;
}

int I2SAudio::availableForWrite() {
  if(status != I2SAudioStart) {
    return 0;
  }
  _eventQueue(0);
  return txEmpty ? I2SAudio::getPayloadSize() : 0;
}

int I2SAudio::available() { /*ForRead*/
  if(status != I2SAudioStart) {
    return 0;
  }
  _eventQueue(0);
  return rxFilled ? I2SAudio::getPayloadSize() : 0;
}

bool I2SAudio::waitForWritable(std::uint32_t maxWaitMsec) {
  if(status != I2SAudioStart) {
    delay(1000);
    return false;
  }
  _eventQueue((TickType_t)maxWaitMsec);
  return txEmpty;
}

bool I2SAudio::waitForReadable(std::uint32_t maxWaitMsec) {
  if(status != I2SAudioStart) {
    delay(1000);
    return false;
  }
  _eventQueue((TickType_t)maxWaitMsec);
  return rxFilled;
}
