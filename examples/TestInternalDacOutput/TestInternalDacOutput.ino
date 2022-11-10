#include <Esp32BuiltinDacAudio.h>

#define AUDIO_SAMPLERATE 8000  // Output/Input Audio sampling rate
#define AUDIO_BITRATE 16       // Output/Input Audio bit rate
#define AUDIO_BIT_LENGTH 16
#define AUDIO_MSEC 20         // DMA Buffer time
#define AUDIO_BUFFER_COUNT 4  // DMA Buffer count

Esp32BuiltinDacAudio audio(AUDIO_SAMPLERATE, AUDIO_BITRATE, AUDIO_BIT_LENGTH, AUDIO_MSEC, AUDIO_BUFFER_COUNT);
uint8_t buffer[AUDIO_SAMPLERATE*AUDIO_MSEC/1000*(AUDIO_BIT_LENGTH/8)];

void setup() {
  //UART
  Serial.begin(115200);
  audio.begin();
  audio.start();
  log_d("start");
}

void loop() {
  static const int16_t frequency=440;
  static const int16_t volume=32767;
  static int s;
  if(audio.getPayloadSize()<=audio.availableForWrite()){
    int16_t* b = (int16_t*)buffer;
    for(int i=0;i<audio.getBufferLength();i++){
      b[i]=(int16_t)(volume*sin(2.0 * M_PI * frequency*(s++)/AUDIO_SAMPLERATE));
    }
    audio.write((const uint8_t *)buffer, audio.getPayloadSize());
  }

  //size_t r = audio.readB((uint8_t *)buffer, audio.getBufferLength()*AUDIO_BITRATE/8);
  //log_d("%d",r);
}
