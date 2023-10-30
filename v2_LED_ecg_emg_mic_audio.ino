#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "EMGFilters.h"
#include <list>

// GUItool: begin automatically generated code
AudioInputI2S mic_i2sLineIN; //xy=336,388

AudioFilterStateVariable mic_filter; //xy=514,388

AudioPlaySdWav playSDBeatWavNext; //xy=603,513
AudioPlaySdWav playSDBeatWavPrevious; //xy=616,565
AudioPlaySdWav playSdEMG_max; //xy=626,851
AudioPlaySdWav playSdEMG_med; //xy=631,790
AudioPlaySdWav playSdEMG_min; //xy=632,737

AudioEffectFreeverb mic_freeverb; //xy=671,250
AudioEffectDelay mic_delay; //xy=832,291

AudioMixer4 mic_filterMixer; //xy=690,396
AudioMixer4 beatOut_mixerR; //xy=874,658
AudioMixer4 beatOut_mixerL; //xy=884,553
AudioMixer4 emgOut_mixerR; //xy=889,883
AudioMixer4 emgOut_mixerL; //xy=899,778
AudioMixer4 micOut_mixerL; //xy=1137,275
AudioMixer4 micOut_mixerR; //xy=1139,359
AudioMixer4 out_mixerL; //xy=1456,406
AudioMixer4 out_mixerR; //xy=1458,497

AudioOutputI2S out_i2s; //xy=1689,453

AudioConnection patchCord1(mic_i2sLineIN, 0, mic_filter, 0);
AudioConnection patchCord2(mic_filter, 0, mic_filterMixer, 0);
AudioConnection patchCord3(mic_filter, 1, mic_filterMixer, 1);
AudioConnection patchCord4(mic_filter, 2, mic_filterMixer, 2);

AudioConnection patchCord5(playSDBeatWavNext, 0, beatOut_mixerL, 0);
AudioConnection patchCord6(playSDBeatWavNext, 1, beatOut_mixerR, 0);
AudioConnection patchCord7(playSDBeatWavPrevious, 0, beatOut_mixerL, 1);
AudioConnection patchCord8(playSDBeatWavPrevious, 1, beatOut_mixerR, 1);

AudioConnection patchCord9(playSdEMG_max, 0, emgOut_mixerL, 2);
AudioConnection patchCord10(playSdEMG_max, 1, emgOut_mixerR, 2);
AudioConnection patchCord11(playSdEMG_med, 0, emgOut_mixerL, 1);
AudioConnection patchCord12(playSdEMG_med, 1, emgOut_mixerR, 1);
AudioConnection patchCord13(playSdEMG_min, 0, emgOut_mixerL, 0);
AudioConnection patchCord14(playSdEMG_min, 1, emgOut_mixerR, 0);

AudioConnection patchCord15(mic_freeverb, mic_delay);
AudioConnection patchCord16(mic_filterMixer, mic_freeverb);
AudioConnection patchCord17(mic_delay, 0, micOut_mixerL, 0);
AudioConnection patchCord18(mic_delay, 1, micOut_mixerR, 0);
AudioConnection patchCord19(mic_delay, 2, micOut_mixerL, 1);
AudioConnection patchCord20(mic_delay, 3, micOut_mixerR, 1);
AudioConnection patchCord21(mic_delay, 4, micOut_mixerL, 2);
AudioConnection patchCord22(mic_delay, 5, micOut_mixerR, 2);
AudioConnection patchCord23(mic_delay, 6, micOut_mixerL, 3);
AudioConnection patchCord24(mic_delay, 7, micOut_mixerR, 3);

AudioConnection patchCord25(beatOut_mixerR, 0, out_mixerR, 1);
AudioConnection patchCord26(beatOut_mixerL, 0, out_mixerL, 1);
AudioConnection patchCord27(emgOut_mixerR, 0, out_mixerR, 2);
AudioConnection patchCord28(emgOut_mixerL, 0, out_mixerL, 2);
AudioConnection patchCord29(micOut_mixerL, 0, out_mixerL, 0);
AudioConnection patchCord30(micOut_mixerR, 0, out_mixerR, 0);

AudioConnection patchCord31(out_mixerL, 0, out_i2s, 0);
AudioConnection patchCord32(out_mixerR, 0, out_i2s, 1);


AudioControlSGTL5000     sgtl5000_1;

#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  11  // not actually used
#define SDCARD_SCK_PIN   13  // not actually used


std::list<int>  ecgQueue;

const int EMG_PIN = A17;
const int ECG_PIN = A16;

const int LT_PIN =  24;
const int LB_PIN =  25;
const int RT_PIN =  29;
const int RB_PIN =  28;

const int BTN_PIN =  39;

#define ARR_SIZE(a) (sizeof(a) / sizeof(a[0]))

// Cycle buffer
typedef struct
{
  uint8_t index;
  uint16_t buf[64]; /* Buffer for rectified AC value */
  uint32_t sum;     /* Sum for fast caculation of mean value */
} CycleBuf_t;

// Append to cycle buffer
#define CYCLE_BUF_ADD(cb, val)                    \
  {                                               \
    cb.sum -= cb.buf[cb.index];                   \
    cb.sum += (val);                              \
    cb.buf[cb.index] = (val);                     \
    cb.index = (cb.index + 1) % ARR_SIZE(cb.buf); \
  }

/* Get mean value of cycle buffer */
#define CYCLE_BUF_MEAN(cb) (cb.sum / ARR_SIZE(cb.buf))

CycleBuf_t rectifiedAcBuf;
EMGFilters myFilter;

// Set the input frequency.
//
// The filters work only with fixed sample frequency of
// `SAMPLE_FREQ_500HZ` or `SAMPLE_FREQ_1000HZ`.
// Inputs at other sample rates will bypass
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_500HZ;

// Time interval for processing the input signal. 
unsigned long long interval = 1000000ul / sampleRate;

// Set the frequency of power line hum to filter out.
//
// For countries with 60Hz power line, change to "NOTCH_FREQ_60HZ"
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

void setup() {
  /* add setup code here */
  
  //initialization 
  myFilter.init(sampleRate, humFreq, true, true, true);
  rectifiedAcBuf.sum = 0;
  rectifiedAcBuf.index = 0;
  for (int j = 0; j < ARR_SIZE(rectifiedAcBuf.buf); j++) {
      rectifiedAcBuf.buf[j] = 0;
  }

  // open serial
  Serial.begin(115200);

  //calibration
  pinMode(BTN_PIN, INPUT_PULLDOWN);

  pinMode(LT_PIN, OUTPUT);
  pinMode(LB_PIN, OUTPUT);
  pinMode(RT_PIN, OUTPUT);
  pinMode(RB_PIN, OUTPUT);

  analogWriteFrequency(LT_PIN, 36621.09);
  analogWriteFrequency(LB_PIN, 36621.09);
  analogWriteFrequency(RT_PIN, 36621.09);
  analogWriteFrequency(RB_PIN, 36621.09);

  analogWriteResolution(12);
  
  analogWrite(LT_PIN, 512);
  analogWrite(LB_PIN, 0);
  analogWrite(RT_PIN, 0);
  analogWrite(RB_PIN, 0);
  delay(250);
  
  analogWrite(LT_PIN, 0);
  analogWrite(LB_PIN, 512);
  analogWrite(RT_PIN, 0);
  analogWrite(RB_PIN, 0);
  delay(250);

  analogWrite(LT_PIN, 0);
  analogWrite(LB_PIN, 0);
  analogWrite(RT_PIN, 512);
  analogWrite(RB_PIN, 0);
  delay(250);

  analogWrite(LT_PIN, 0);
  analogWrite(LB_PIN, 0);
  analogWrite(RT_PIN, 0);
  analogWrite(RB_PIN, 512);
  delay(250);
  
  analogWrite(LT_PIN, 0);
  analogWrite(LB_PIN, 0);
  analogWrite(RT_PIN, 0);
  analogWrite(RB_PIN, 0);
  delay(500);

  //audio
  AudioMemory(150);  // Increase audio memory blocks for additional processing

  sgtl5000_1.enable();
  sgtl5000_1.volume(.5);

  sgtl5000_1.inputSelect(AUDIO_INPUT_LINEIN);
//  sgtl5000_1.micGain(36);

  mic_delay.delay(0, random(110, 300));
  mic_delay.delay(1, random(110, 300));
  mic_delay.delay(2, random(110, 300));
  mic_delay.delay(3, random(110, 300));
  mic_delay.delay(4, random(110, 300));
  mic_delay.delay(5, random(110, 300));
  mic_delay.delay(6, random(110, 300));
  mic_delay.delay(7, random(110, 300));

  // Initialize SD card
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  if (!(SD.begin(SDCARD_CS_PIN))) {
    // stop here if no SD card, but print a message
    while (1) {
      Serial.println("Unable to access the SD card");
      delay(500);
    }
  }
}

//emg and ecg
unsigned long long lastBeat = 0;

unsigned long long calibrationCounter;

float sin_time = 0;
float sin_period = 5;
float sin_amplitude = 1;
float sin_min = 0;
float sin_phase = 0;
float sin_duty = 0;
int sin_duty_int = 0; 

uint8_t calibrationStatus = 0;
unsigned long long calibrationStartTime = 0;
uint16_t emgCAL_envelopeMax = 0;
uint16_t emgCAL_envelopeMin = 0xFFFF;
uint16_t emgCAL_envelopeAmplitue = 0;


float emg_envelope_f = 0;
float emg_envelope_norm = 0;
float bpm_f = 70;
float ecgBeatThreshold = 0;
bool isBeat = false;
unsigned long long ibi;

//audio
float gain0 = 0;
float gain1 = 0;
float gain2 = 0;
float gain_beat0;
float gain_beat1;


float mapToRange(float rangeStart, float rangeEnd, float var){
  return (var-rangeStart) /(rangeEnd - rangeStart);
}


const float emg_point1 = 0.2f;
const float emg_point2 = 0.5f;
const float emg_point3 = 0.8f;


int mainLoopHZ = 500;
uint32_t counterMsec = 0;
int counterBeat = 1;
int counterInitialBeat = 0;
int previousBeatFile = 0;
int nextBeatFile = 0;

const char* filenames[10] = {"stb0.wav", "stb1.wav", "stb2.wav", "stb3.wav", "stb4.wav", "stb5.wav", "stb6.wav", "stb7.wav", "stb8.wav", "stb9.wav"};


void loop() {

  
  unsigned long long timeStamp = micros();

  //LED
  if(digitalReadFast(BTN_PIN)){
    calibrationStatus = 1;
    calibrationStartTime = micros();
    emgCAL_envelopeMax = 0;
    emgCAL_envelopeMin = 0xFF;
    delay(2000);
    return;
  }

  if(calibrationStatus == 1){
    if(micros() - calibrationStartTime > 10 *1000 *1000){
      calibrationStatus = 2;
    }
  }


  int data = 0, dataAfterFilter = 0;

  dataAfterFilter = myFilter.update(analogRead(EMG_PIN));
    // Rectification
  CYCLE_BUF_ADD(rectifiedAcBuf, abs(dataAfterFilter));
    // Simple envelope calculation, use 2 * rectified value
  uint16_t envelope = CYCLE_BUF_MEAN(rectifiedAcBuf) * 2;
  emg_envelope_f = emg_envelope_f*0.995 + envelope*0.005;

  if(calibrationStatus == 1){
    emg_envelope_norm = 0;
    if(emgCAL_envelopeMax < envelope){
      emgCAL_envelopeMax = envelope;
      if(emgCAL_envelopeMax > emgCAL_envelopeMin){
        emgCAL_envelopeAmplitue = emgCAL_envelopeMax - emgCAL_envelopeMin;
      }
    }
    if(emgCAL_envelopeMin > envelope){
      emgCAL_envelopeMin = envelope;
      if(emgCAL_envelopeMax > emgCAL_envelopeMin){
        emgCAL_envelopeAmplitue = emgCAL_envelopeMax - emgCAL_envelopeMin;
      }
    }
  } else if(calibrationStatus == 2){
    emg_envelope_norm = (emg_envelope_f - emgCAL_envelopeMin) / emgCAL_envelopeAmplitue;
    if(emg_envelope_norm < 0) emg_envelope_norm = 0;
    if(emg_envelope_norm > 1) emg_envelope_norm = 1;
  }

//   Serial.print(emg_envelope_norm*100);
   Serial.print(emg_envelope_f);
   Serial.print(" ,");
   Serial.print(envelope);
   
   Serial.println(" , 100, 0");


  int value = analogRead(ECG_PIN);
  ecgQueue.push_front(value);
  while (ecgQueue.size() > 1500){
        ecgQueue.pop_back();
  }

  int sample_maxVal = 0;
  int sample_minVal = 0xFFFF;

  for(std::list<int>::iterator it =ecgQueue.begin(); it != ecgQueue.end(); it++) {
//      rms += *it * *it;
        if(sample_maxVal < *it){
            sample_maxVal = *it;
        }
        if(sample_minVal > *it){
            sample_minVal = *it;
        }
    }

  uint16_t amplitude = sample_maxVal - sample_minVal;
  ecgBeatThreshold = ecgBeatThreshold * 0.93 + (sample_maxVal - amplitude *.25) *0.07;

  uint16_t flag = 0;
  if(isBeat){
      if(value < ecgBeatThreshold){
          flag = 0;
          isBeat = false;
      }
  } else {
      if(amplitude > 100){
          unsigned long long ibi = micros() - lastBeat;
          if((value > ecgBeatThreshold) && (ibi > 400*1000)){
              
              
              bpm_f = bpm_f* 0.5 +  (60000000.0 / (ibi))*0.5;
              lastBeat = micros();
              
              isBeat = true;
              flag = 3150;
          }
      }
  }

  
  
  sin_period = 60/bpm_f;
  sin_time += (360.0f/500.0f) / sin_period;
  if(sin_time > 360) sin_time = fmod(sin_time, 360);

  sin_phase = 0.5 + 0.5 * sin(sin_time *3.14159265/180.0);
  sin_duty = sin_phase * sin_amplitude + sin_min;
  sin_duty_int = round(sin_duty * 4095);




  analogWrite(LT_PIN, sin_duty_int*emg_envelope_norm);
  analogWrite(LB_PIN, sin_duty_int*emg_envelope_norm);
  analogWrite(RT_PIN, sin_duty_int*emg_envelope_norm);
  analogWrite(RB_PIN, sin_duty_int*emg_envelope_norm);



  // Serial.print(value);
  // Serial.print(", ");
  // Serial.print(ecgBeatThreshold);
  // Serial.print(", ");
  // Serial.print(flag);
  // Serial.println();



  // Serial.print(bpm_f);
  // Serial.println();
  

  // Serial.print(128 + dataAfterFilter); // Draw offset = 128
  // Serial.print(" ");
  // Serial.println(heartValue);
  // Serial.print(heartValue);
  // Serial.print(emg_envelope_f);
  // Serial.print(", ");
  // Serial.print(bpm_f);
  // Serial.print(" ");




  //ecg
  if(counterMsec >= 60000 / bpm_f){
    
      gain_beat0 = float(counterBeat)/float(counterInitialBeat);
      gain_beat1 = 1 - float(counterBeat)/float(counterInitialBeat);

      beatOut_mixerL.gain(0, gain_beat0*.5);
      beatOut_mixerL.gain(1, gain_beat1*.5);  
      beatOut_mixerR.gain(0, gain_beat0*.5);  
      beatOut_mixerR.gain(1, gain_beat1*.5); 
      
//      playSDBeatWavPrevious.play(filenames[previousBeatFile]);
//      playSDBeatWavNext.play(filenames[nextBeatFile]);
//      Serial.print(filenames[previousBeatFile]);
//      Serial.print(", ");
//      Serial.println(filenames[nextBeatFile]);
      counterInitialBeat ++;
      if((--counterBeat) <= 0){
        counterInitialBeat = counterBeat = random(10, 31);
        previousBeatFile = nextBeatFile;
        nextBeatFile = random(0,10);
      }
      counterMsec = 0;
    }
    counterMsec += 2;




  // EMG Below

  if(!playSdEMG_min.isPlaying()) playSdEMG_min.play("stloop1a.wav");
  if(!playSdEMG_med.isPlaying()) playSdEMG_med.play("stloop1b.wav");
  if(!playSdEMG_max.isPlaying()) playSdEMG_max.play("stloop1c.wav");


//  sin_time += (360.0f/500.0f) / sin_period;
//  if(sin_time > 360) sin_time = fmod(sin_time, 360);
//  
//  sin_phase = 0.5 + 0.5 * sin(sin_time *3.14159265/180.0);
//  sin_duty = sin_phase * sin_amplitude + sin_min;
  //Serial.print(sin_duty);
  //Serial.println();
  //sinwave generator
  

  float emg = emg_envelope_norm;
  if((emg >= 0) & (emg <= emg_point1)){
    gain0 = 0;
    gain2 = 0;
  } else if(emg <= emg_point2){
    gain0 = mapToRange(emg_point1, emg_point2, emg);
    gain2 = 0;
  } else if(emg <= emg_point3){
    gain0 = 1 - mapToRange(emg_point2, emg_point3, emg);
    gain2 = mapToRange(emg_point2, emg_point3, emg);
  } else if(emg <= 1.0f){
    gain0 = 0;
    gain2 = 1;
  }

    emgOut_mixerL.gain(0, gain0);  
    emgOut_mixerL.gain(1, gain1);  
    emgOut_mixerL.gain(2, gain2);  

    emgOut_mixerR.gain(0, gain0);  
    emgOut_mixerR.gain(1, gain1);  
    emgOut_mixerR.gain(2, gain2);





  unsigned long timeElapsed = micros() - timeStamp;
  // Serial.print("Filters cost time: ");
  // Serial.println(timeElapsed);
  if (interval > timeElapsed){
    delayMicroseconds(interval - timeElapsed);
  } else {
    Serial.println("WARNING! CPU SLOW");
  }

    
}
