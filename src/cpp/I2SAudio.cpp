/**
 * @copyright 2019 NOEX inc.
 * @author Kenji Takahashi
 */

#include "../I2SAudio.h"
#include <driver/rtc_io.h>
#include <string.h>

#define ARDUINO_AUDIO_SPIRAM_ENABLED

#ifdef ARDUINO_AUDIO_SPIRAM_ENABLED
#include <esp_heap_caps.h>
#endif

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
  ringTxBuffer   = nullptr;
  ringTxReadIdx  = 0;
  ringTxWriteIdx = 0;
  ringTxCount    = 0;
  txPrimed       = false;
  rxBuffer = nullptr;  // virtualではなく、自分を呼ぶ。begin()でPSRAM初期化後に確保する
  initRtcPin(audioConfig.pinConfig.bck_io_num);
  initRtcPin(audioConfig.pinConfig.ws_io_num);
  initRtcPin(audioConfig.pinConfig.data_out_num);
}

I2SAudio::~I2SAudio() {
  I2SAudio::stop();  // virtualではなく、自分を呼ぶ
#ifdef ARDUINO_AUDIO_SPIRAM_ENABLED
  heap_caps_free(ringTxBuffer);
  heap_caps_free(rxBuffer);
#else
  delete[] ringTxBuffer;
  delete[] rxBuffer;
#endif
}

std::uint8_t I2SAudio::getBufferCount() const {
  return i2sConfig.dma_buf_count;
}

void I2SAudio::begin() {
  // PSRAM 初期化後に呼ばれるため、ここで SPIRAM 優先確保する
  const std::size_t ringSize = getBufferCount() * I2SAudio::getPayloadSize();
#ifdef ARDUINO_AUDIO_SPIRAM_ENABLED
  ringTxBuffer = (char*)heap_caps_malloc(ringSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
  if (!ringTxBuffer) {
    log_e("I2SAudio: SPIRAM unavailable, fallback to internal RAM for ring buffer");
    ringTxBuffer = new char[ringSize];
  }
  const std::size_t rxSize = I2SAudio::getPayloadSize();  // virtualではなく、自分を呼ぶ
#ifdef ARDUINO_AUDIO_SPIRAM_ENABLED
  rxBuffer = (char*)heap_caps_malloc(rxSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
  if (!rxBuffer) {
    log_e("I2SAudio: SPIRAM unavailable, fallback to internal RAM for rx buffer");
    rxBuffer = new char[rxSize];
  }
  ESP_ERROR_CHECK(i2s_driver_install(audioConfig.port, &i2sConfig, getBufferCount()*2, &i2s_event_queue));
}

void I2SAudio::start() {
  _start(I2SAudioStart);
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
#if defined(IDF_VER)
  const uint32_t bits_cfg = ((uint32_t)I2S_BITS_PER_CHAN_32BIT << 16) | (uint32_t)i2sConfig.bits_per_sample;
#else
  const uint32_t bits_cfg = (uint32_t)i2sConfig.bits_per_sample;
#endif
  i2s_set_pin(audioConfig.port, hasPinConfig?&(audioConfig.pinConfig):NULL);
  i2s_channel_t clock_channel = I2S_CHANNEL_STEREO;
  switch (i2sConfig.channel_format) {
    case I2S_CHANNEL_FMT_RIGHT_LEFT:
    case I2S_CHANNEL_FMT_ALL_RIGHT:
    case I2S_CHANNEL_FMT_ALL_LEFT: {
      clock_channel = I2S_CHANNEL_STEREO;
      i2s_set_clk(audioConfig.port, i2sConfig.sample_rate, bits_cfg, clock_channel);
    } break;
    case I2S_CHANNEL_FMT_ONLY_RIGHT:
    case I2S_CHANNEL_FMT_ONLY_LEFT: {
      clock_channel = I2S_CHANNEL_MONO;
      i2s_set_clk(audioConfig.port, i2sConfig.sample_rate, bits_cfg, clock_channel);
    } break;
  }
  ESP_ERROR_CHECK(i2s_start(audioConfig.port));
  rxDone = false;
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
  // リングバッファをリセット（DMAをゼロクリアしたので未送信データは破棄）
  ringTxCount    = 0;
  ringTxReadIdx  = 0;
  ringTxWriteIdx = 0;
  txPrimed       = false;
}

bool I2SAudio::_recvQueue(i2s_event_type_t type) {
  switch (type) {
    case I2S_EVENT_DMA_ERROR: {
      log_e("I2S: Error");
    } break;
    case I2S_EVENT_TX_DONE: {
      // DMAバッファが1つ消費された。次のドレインをトリガーする。
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

  // リングにデータがあれば即ドレイン（イベント待ち不要）
  if(ringTxCount > 0 || rxFilled) {
    ticks_to_wait = 0;
  }
  const std::uint32_t elapsedMsec = startMsec - lastMsec;
  if (xQueueReceive(i2s_event_queue, &event, std::min((TickType_t)getBufferMsec()*getBufferCount(), ticks_to_wait)) == pdTRUE) {
    bool done = false;
    done |= _recvQueue(event.type);
    while (xQueueReceive(i2s_event_queue, &event, 0) == pdTRUE) {
      done |= _recvQueue(event.type);
    }
    if(done) lastMsec = millis();
  } else if((std::uint32_t)getBufferMsec()*getBufferCount()<=elapsedMsec){
    log_w("i2s: event timeout");
    if (((uint8_t)i2sConfig.mode & (uint8_t)I2S_MODE_RX) == (uint8_t)I2S_MODE_RX) {
      rxFilled = getBufferCount();
    }
    lastMsec = millis();
  }

  // TX: 一定量プリフィル後にリングバッファから DMA へドレイン
  while (txPrimed && ringTxCount > 0) {
    std::size_t bytesWritten = 0;
#ifdef I2S_LEGACY_API_ENABLED
    int bw = i2s_write_bytes(audioConfig.port,
      ringTxBuffer + ringTxReadIdx * I2SAudio::getPayloadSize(),
      I2SAudio::getPayloadSize(), 0);
    bytesWritten = (bw > 0) ? (std::size_t)bw : 0;
#else
    esp_err_t ret = i2s_write(audioConfig.port,
      ringTxBuffer + ringTxReadIdx * I2SAudio::getPayloadSize(),
      I2SAudio::getPayloadSize(), &bytesWritten, 0);
    if (ret != ESP_OK) bytesWritten = 0;
#endif
    if (I2SAudio::getPayloadSize() <= bytesWritten) {
      ringTxReadIdx = (ringTxReadIdx + 1) % getBufferCount();
      ringTxCount--;
      if (ringTxCount == 0) {
        // キューを使い切ったら DMA 側を無音で埋め直し、次回 write 群で再度プリフィルしてから再開する
        txPrimed = false;
        zero();
      }
      lastMsec = millis();
    } else {
      break;  // DMA が満杯なので次回へ
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
  if (length <= I2SAudio::getPayloadSize() && ringTxCount < getBufferCount()) {
    memcpy(ringTxBuffer + ringTxWriteIdx * I2SAudio::getPayloadSize(),
           buffer, I2SAudio::getPayloadSize());
    ringTxWriteIdx = (ringTxWriteIdx + 1) % getBufferCount();
    ringTxCount++;
    if (!txPrimed && ringTxCount >= getBufferCount()) {
      txPrimed = true;
    }
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
  return (getBufferCount() - ringTxCount) * I2SAudio::getPayloadSize();
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
    return false;  // 起動前は即リターン（delay不要）
  }
  if(ringTxCount < getBufferCount()) return true;  // リングに空きあり
  _eventQueue((TickType_t)maxWaitMsec);  // リングが満杯ならDMAドレインを待つ
  return ringTxCount < getBufferCount();
}

bool I2SAudio::waitForReadable(std::uint32_t maxWaitMsec) {
  if(status != I2SAudioStart) {
    return false;
  }
  _eventQueue((TickType_t)maxWaitMsec);
  return rxFilled;
}
