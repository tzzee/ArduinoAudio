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
        .intr_alloc_flags = 0,
        .dma_buf_count = bufferCount,
        .dma_buf_len = (int)getBufferLength() ,
        .use_apll = false
      }),
      installed(false) {
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


//  start DAC
//  DAC_Start()後は、DMAバッファが空になる前にDAC_Write()で出力データを書き込むこと
void I2SAudio::start() {
  I2SAudio::stop();  // virtualではなく、自分を呼ぶ
  deinitRtcPin(audioConfig.pinConfig.bck_io_num);
  deinitRtcPin(audioConfig.pinConfig.ws_io_num);
  deinitRtcPin(audioConfig.pinConfig.data_out_num);
#ifdef I2S_EVENT_ENABLED
  static const int i2s_event_queue_size = 3;
  i2s_driver_install(audioConfig.port, &i2sConfig, i2s_event_queue_size, &i2s_event_queue);
#else
  i2s_driver_install(audioConfig.port, &i2sConfig, 0, NULL);
#endif
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
  i2s_start(audioConfig.port);
  installed = true;
  zero();
  rxDone = !(i2sConfig.mode & I2S_MODE_RX);
}

//  stop DAC
//  データ出力後、DMAバッファが空になる前にこの関数を呼び出すこと
void I2SAudio::stop() {
  if (installed) {
    i2s_stop(audioConfig.port);
    i2s_driver_uninstall(audioConfig.port);  // stop & destroy i2s driver
    initRtcPin(audioConfig.pinConfig.bck_io_num);
    initRtcPin(audioConfig.pinConfig.ws_io_num);
    initRtcPin(audioConfig.pinConfig.data_out_num);
    installed = false;
  }
}

void I2SAudio::zero() {
  i2s_zero_dma_buffer(audioConfig.port);
  txDone = true;
}

void I2SAudio::_eventQueue() {
  static const TickType_t ticks_to_wait = 0;
#ifdef I2S_EVENT_ENABLED
  i2s_event_t event;
  if (xQueueReceive(i2s_event_queue, &event, ticks_to_wait) == pdTRUE) {
    switch (event.type) {
      case I2S_EVENT_DMA_ERROR: {
      } break;
      case I2S_EVENT_TX_DONE: {
        // All buffers are empty. This means we have an underflow on our hands.
        if (txDone) {
          memset(txBuffer, 0, I2SAudio::getPayloadSize());
          txDone = false;
        }
      } break;
      case I2S_EVENT_RX_DONE: {
        // All buffers are full. This means we have an overflow.
        rxDone = false;
      } break;
      default: { } break; }
  }
#endif
  if (!txDone) {
#ifdef I2S_LEGACY_API_ENABLED
    int bytesWritten = i2s_write_bytes(audioConfig.port, txBuffer, I2SAudio::getPayloadSize(), ticks_to_wait);
#else
    int bytesWritten = 0;
    size_t write_len = 0;
    while (1) {
      esp_err_t ret =
          i2s_write(audioConfig.port, (const char *)txBuffer + bytesWritten,
                    I2SAudio::getPayloadSize() - bytesWritten, &write_len, ticks_to_wait);
      bytesWritten += write_len;
      if (I2SAudio::getPayloadSize() == bytesWritten || ret != ESP_OK) {
        break;
      }
    }
#endif
    if (I2SAudio::getPayloadSize() <= bytesWritten) {
      txDone = true;
    }
  }
  if (!rxDone) {
#ifdef I2S_LEGACY_API_ENABLED
    int bytesRead = i2s_read_bytes(audioConfig.port, rxBuffer, I2SAudio::getPayloadSize(), ticks_to_wait);
#else
    int bytesRead = 0;
    size_t read_len = 0;
    while (1) {
      esp_err_t ret = i2s_read(audioConfig.port, reinterpret_cast<char *>(rxBuffer) + bytesRead,
                               I2SAudio::getPayloadSize() - bytesRead, &read_len, ticks_to_wait);
      bytesRead += read_len;
      if (I2SAudio::getPayloadSize() <= bytesRead || ret != ESP_OK) {
        break;
      }
    }
#endif
    if (I2SAudio::getPayloadSize() <= bytesRead) {
      rxDone = true;
    }
  }
}

size_t I2SAudio::_i2s_read_bytes(void *dest) {
  if (!rxDone) {
    _eventQueue();
  }
  if (rxDone) {
    memcpy(dest, rxBuffer, I2SAudio::getPayloadSize());
    rxDone = false;
    return I2SAudio::getPayloadSize();
  }
  return 0;
}

size_t I2SAudio::_i2s_write_bytes(const void *src) {
  size_t s = 0;
  if (txDone) {
    memcpy(txBuffer, src, I2SAudio::getPayloadSize());
    txDone = false;
    s = I2SAudio::getPayloadSize();
  }
  _eventQueue();
  return s;
}

  size_t I2SAudio::read(std::uint8_t* buffer, std::size_t length) {
    log_v("%d %d", length, I2SAudio::available());
    // length==I2SAudio::getPayloadSize()
    if (length <= I2SAudio::available()) {
      return _i2s_read_bytes((void*)buffer);
    }
    return 0;
  }
  size_t I2SAudio::write(const std::uint8_t* buffer, std::size_t length) {
    log_v("%d %d", length, I2SAudio::availableForWrite());
    // length==I2SAudio::getPayloadSize()
    if (length <= I2SAudio::availableForWrite()) {
      return _i2s_write_bytes((const void*)buffer);
    }
    return 0;
  }

int I2SAudio::availableForWrite() {
  if (!txDone) {
    _eventQueue();
  }
  return txDone ? I2SAudio::getPayloadSize() : 0;
}

int I2SAudio::available() { /*ForRead*/
  if (!rxDone) {
    _eventQueue();
  }
  return rxDone ? I2SAudio::getPayloadSize() : 0;
}
