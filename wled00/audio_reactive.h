/*
 * This file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * bytes 2400+ are currently ununsed, but might be used for future wled features
 */

// WARNING Sound reactive variables that are used by the animations or other asynchronous routines must NOT
// have interim values, but only updated in a single calculation. These are:
//
// sample     sampleAvg     sampleAgc       samplePeak    myVals[]
//
// fftBin[]   fftResult[]   FFT_MajorPeak   FFT_Magnitude
//
// Otherwise, the animations may asynchronously read interim values of these variables.
//

#include "wled.h"
#include <driver/i2s.h>
#include "audio_source.h"

AudioSource *audioSource;

// ALL AUDIO INPUT PINS DEFINED IN wled.h AND CONFIGURABLE VIA UI

// Comment/Uncomment to toggle usb serial debugging
// #define SR_DEBUG

#ifdef SR_DEBUG
  #define DEBUGSR_PRINT(x) Serial.print(x)
  #define DEBUGSR_PRINTLN(x) Serial.println(x)
  #define DEBUGSR_PRINTF(x...) Serial.printf(x)
#else
  #define DEBUGSR_PRINT(x)
  #define DEBUGSR_PRINTLN(x)
  #define DEBUGSR_PRINTF(x...)
#endif

// #define MIC_LOGGER
// #define MIC_SAMPLING_LOG
// #define FFT_SAMPLING_LOG

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 64;
const int SAMPLE_RATE = 16000;                  // Base sample rate in Hz

//Use userVar0 and userVar1 (API calls &U0=,&U1=, uint16_t)

#ifndef LED_BUILTIN     // Set LED_BUILTIN if it is not defined by Arduino framework
  #define LED_BUILTIN 3
#endif

#define UDP_SYNC_HEADER "00001"

uint8_t maxVol = 10;                            // Reasonable value for constant volume for 'peak detector', as it won't always trigger
uint8_t binNum;                                 // Used to select the bin for FFT based beat detection.
uint8_t targetAgc = 60;                         // This is our setPoint at 20% of max for the adjusted output
uint8_t myVals[32];                             // Used to store a pile of samples because WLED frame rate and WLED sample rate are not synchronized. Frame rate is too low.
bool samplePeak = 0;                            // Boolean flag for peak. Responding routine must reset this flag
bool udpSamplePeak = 0;                         // Boolean flag for peak. Set at the same tiem as samplePeak, but reset by transmitAudioData
int delayMs = 10;                               // I don't want to sample too often and overload WLED
int micIn;                                      // Current sample starts with negative values and large values, which is why it's 16 bit signed
int sample;                                     // Current sample. Must only be updated ONCE!!!
int tmpSample;                                  // An interim sample variable used for calculatioins.
int sampleAdj;                                  // Gain adjusted sample value
int sampleAgc;                                  // Our AGC sample
uint16_t micData;                               // Analog input for FFT
uint16_t micDataSm;                             // Smoothed mic data, as it's a bit twitchy
long timeOfPeak = 0;
long lastTime = 0;
float micLev = 0;                               // Used to convert returned value to have '0' as minimum. A leveller
float multAgc;                                  // sample * multAgc = sampleAgc. Our multiplier
float sampleAvg = 0;                            // Smoothed Average
double beat = 0;                                // beat Detection

float expAdjF;                                  // Used for exponential filter.
float weighting = 0.2;                          // Exponential filter weighting. Will be adjustable in a future release.


// FFT Variables
const uint16_t samples = 512;                   // This value MUST ALWAYS be a power of 2
// unsigned int sampling_period_us;
// unsigned long microseconds;

double FFT_MajorPeak = 0;
double FFT_Magnitude = 0;
// uint16_t mAvg = 0;

// These are the input and output vectors.  Input vectors receive computed results from FFT.
fftData_t vReal[samples];
fftData_t vImag[samples];
fftData_t fftBin[samples];
fftData_t windowWeighingFactors[samples];

// Try and normalize fftBin values to a max of 4096, so that 4096/16 = 256.
// Oh, and bins 0,1,2 are no good, so we'll zero them out.
// double fftCalc[16];
int fftResult[16];                              // Our calculated result table, which we feed to the animations.
// double fftResultMax[16];                        // A table used for testing to determine how our post-processing is working.
// float fftAvg[16];

// Table of linearNoise results to be multiplied by soundSquelch in order to reduce squelch across fftResult bins.
// int linearNoise[16] = { 34, 28, 26, 25, 20, 12, 9, 6, 4, 4, 3, 2, 2, 2, 2, 2 };

// Table of multiplication factors so that we can even out the frequency response.
// double fftResultPink[16] = {1.70,1.71,1.73,1.78,1.68,1.56,1.55,1.63,1.79,1.62,1.80,2.06,2.47,3.35,6.83,9.55};


struct audioSyncPacket {
  char header[6] = UDP_SYNC_HEADER;
  uint8_t myVals[32];     //  32 Bytes
  int sampleAgc;          //  04 Bytes
  int sample;             //  04 Bytes
  float sampleAvg;        //  04 Bytes
  bool samplePeak;        //  01 Bytes
  uint8_t fftResult[16];  //  16 Bytes
  double FFT_Magnitude;   //  08 Bytes
  double FFT_MajorPeak;   //  08 Bytes
};

double mapf(double x, double in_min, double in_max, double out_min, double out_max);

bool isValidUdpSyncVersion(char header[6]) {
  if (strncmp(header, UDP_SYNC_HEADER, 6) == 0) {
    return true;
  } else {
    return false;
  }
}

void getSample() {
  static long peakTime;
  //extern double FFT_Magnitude;                    // Optional inclusion for our volume routines // COMMENTED OUT - UNUSED VARIABLE COMPILER WARNINGS
  //extern double FFT_MajorPeak;                    // Same here. Not currently used though       // COMMENTED OUT - UNUSED VARIABLE COMPILER WARNINGS

  #ifdef WLED_DISABLE_SOUND
    micIn = inoise8(millis(), millis());          // Simulated analog read
  #else
    micIn = micDataSm;      // micDataSm = ((micData * 3) + micData)/4;
/*---------DEBUG---------*/
    DEBUGSR_PRINT("micIn:\tmicData:\tmicIn>>2:\tmic_In_abs:\tsample:\tsampleAdj:\tsampleAvg:\n");
    DEBUGSR_PRINT(micIn); DEBUGSR_PRINT("\t"); DEBUGSR_PRINT(micData);
/*-------END DEBUG-------*/
// We're still using 10 bit, but changing the analog read resolution in usermod.cpp
//    if (digitalMic == false) micIn = micIn >> 2;  // ESP32 has 2 more bits of A/D than ESP8266, so we need to normalize to 10 bit.
/*---------DEBUG---------*/
    DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(micIn);
/*-------END DEBUG-------*/
  #endif
  // micLev = ((micLev * 31) + micIn) / 32;          // Smooth it out over the last 32 samples for automatic centering
  // micIn -= micLev;                                // Let's center it to 0 now
  // micIn = abs(micIn);                             // And get the absolute value of each sample
/*---------DEBUG---------*/
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(micIn);
/*-------END DEBUG-------*/

// Using an exponential filter to smooth out the signal. We'll add controls for this in a future release.
  expAdjF = (weighting * micIn + (1.0-weighting) * expAdjF);
  // expAdjF = (expAdjF <= soundSquelch) ? 0: expAdjF;

  tmpSample = (int)expAdjF;

/*---------DEBUG---------*/
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(sample);
/*-------END DEBUG-------*/

  sampleAdj = tmpSample * sampleGain / 40 + tmpSample / 16; // Adjust the gain.
  sampleAdj = min(sampleAdj, 255);
  sample = sampleAdj;                             // ONLY update sample ONCE!!!!

  sampleAvg = ((sampleAvg * 15) + sample) / 16;   // Smooth it out over the last 16 samples.

/*---------DEBUG---------*/
  DEBUGSR_PRINT("\t"); DEBUGSR_PRINT(sample);
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(sampleAvg); DEBUGSR_PRINT("\n\n");
/*-------END DEBUG-------*/

  if (millis() - timeOfPeak > MIN_SHOW_DELAY) {   // Auto-reset of samplePeak after a complete frame has passed.
    samplePeak = 0;
    udpSamplePeak = 0;
    }

  if (userVar1 == 0) samplePeak = 0;
  // Poor man's beat detection by seeing if sample > Average + some value.
  //  Serial.print(binNum); Serial.print("\t"); Serial.print(fftBin[binNum]); Serial.print("\t"); Serial.print(fftAvg[binNum/16]); Serial.print("\t"); Serial.print(maxVol); Serial.print("\t"); Serial.println(samplePeak);
    if (fftBin[binNum] > ( maxVol) && millis() > (peakTime + 100)) {                     // This goe through ALL of the 255 bins
  //  if (sample > (sampleAvg + maxVol) && millis() > (peakTime + 200)) {
  // Then we got a peak, else we don't. The peak has to time out on its own in order to support UDP sound sync.
    samplePeak = 1;
    timeOfPeak = millis();
    udpSamplePeak = 1;
    userVar1 = samplePeak;
    peakTime=millis();
  }
} // getSample()

/*
 * A simple averaging multiplier to automatically adjust sound sensitivity.
 */
void agcAvg() {

  multAgc = (sampleAvg < 1) ? targetAgc : targetAgc / sampleAvg;  // Make the multiplier so that sampleAvg * multiplier = setpoint
  int tmpAgc = sample * multAgc;
  if (tmpAgc > 255) tmpAgc = 0;
  sampleAgc = tmpAgc;                             // ONLY update sampleAgc ONCE because it's used elsewhere asynchronously!!!!
  userVar0 = sampleAvg * 4;
  if (userVar0 > 255) userVar0 = 255;
} // agcAvg()


////////////////////
// Begin FHEilmann changes
/////////////////////
#define ALPHA_SIGNAL 0.005f
#define ALPHA_MINMAX 0.99f

typedef struct {
  uint16_t start_bin;
  uint16_t end_bin;
  uint16_t freq_value;
  float value;
  float value_db;
} frequency_bin;

typedef struct {
  float max;
  float min;
  float avg;
  bool initialized;
} spectrum_stats;

frequency_bin frequency_bins[16];
spectrum_stats stats[16];

void computeBinParams() {

  uint32_t max_freq = SAMPLE_RATE/2;
  uint16_t num_samples = samples >> 1;
  uint8_t num_bins = 16;
  uint16_t freq_per_bin = (max_freq/num_samples);
  uint16_t min_freq = (max_freq/num_samples) * 3;
  float multiplier = pow((max_freq / min_freq),  (1.0f/num_bins));
  uint16_t start_freq = 0;
  uint16_t end_freq = min_freq;
  uint16_t bin_counter = 3;
  uint16_t start_bin = 0;
  uint16_t end_bin = 0;
  for (int bin = 0; bin < num_bins; bin++) {
    start_freq = end_freq;
    end_freq = start_freq * multiplier;
    start_bin = max(uint16_t(floor(start_freq / freq_per_bin)), bin_counter);
    end_bin = min(num_samples - 1, max(int(ceil(end_freq / freq_per_bin)), bin_counter + 1));
    // On the last frequency, make sure we're using all bins.
    if ((bin == num_bins - 1) && (end_bin < num_samples - 1))
      end_bin = num_samples - 1;


    bin_counter = end_bin;
    uint16_t freq_value = (end_bin + start_bin) * freq_per_bin  / 2;

    frequency_bins[bin] = {
      .start_bin = start_bin,
      .end_bin = end_bin,
      .freq_value = freq_value,
      .value = 0.0,
      .value_db = 0.0,
    };
    stats[bin] = {
      .max = 0.0,
      .min = 0.0,
      .avg = 0.0,
      .initialized = false
    };
  }
}


////////////////////
// Begin FFT Code //
////////////////////
#define FFT_SQRT_APPROXIMATION
#define FFT_SPEED_OVER_PRECISION
#include "arduinoFFT.h"

void transmitAudioData() {
  if (!udpSyncConnected) return;
  extern uint8_t myVals[];
  extern int sampleAgc;
  extern int sample;
  extern float sampleAvg;
  extern bool udpSamplePeak;
  extern int fftResult[];
  extern double FFT_Magnitude;
  extern double FFT_MajorPeak;

  audioSyncPacket transmitData;

  for (int i = 0; i < 32; i++) {
    transmitData.myVals[i] = myVals[i];
  }

  transmitData.sampleAgc = sampleAgc;
  transmitData.sample = sample;
  transmitData.sampleAvg = sampleAvg;
  transmitData.samplePeak = udpSamplePeak;
  udpSamplePeak = 0;                              // Reset udpSamplePeak after we've transmitted it

  for (int i = 0; i < 16; i++) {
    transmitData.fftResult[i] = (uint8_t)constrain(fftResult[i], 0, 254);
  }

  transmitData.FFT_Magnitude = FFT_Magnitude;
  transmitData.FFT_MajorPeak = FFT_MajorPeak;

  fftUdp.beginMulticastPacket();
  fftUdp.write(reinterpret_cast<uint8_t *>(&transmitData), sizeof(transmitData));
  fftUdp.endPacket();
  return;
} // transmitAudioData()


// Create FFT object
ArduinoFFT<fftData_t> FFT = ArduinoFFT<fftData_t>( vReal, vImag, samples, SAMPLE_RATE, windowWeighingFactors);

// FFT main code
void FFTcode( void * parameter) {
  DEBUG_PRINT("FFT running on core: "); DEBUG_PRINTLN(xPortGetCoreID());
  computeBinParams();
  NoiseGate gate(SAMPLE_RATE, 200, 1000, 500);
  TickType_t xLastWakeTime;
  for(;;) {
    xLastWakeTime = xTaskGetTickCount();
    delay(1);           // DO NOT DELETE THIS LINE! It is needed to give the IDLE(0) task enough time and to keep the watchdog happy.
                        // taskYIELD(), yield(), vTaskDelay() and esp_task_wdt_feed() didn't seem to work.

    // Only run the FFT computing code if we're not in Receive mode
    if (audioSyncEnabled & (1 << 1))
      continue;
    audioSource->getSamples(vReal, samples);

    for (int i=0; i < samples; i++)
    {
      vImag[i] = 0;
    }

    FFT.dcRemoval();

    gate.processSamples(vReal, samples, soundSquelch);

    NoiseGateState ngState = gate.getState();
    if ((ngState == NG_HOLD_CLOSED) || (ngState == NG_CLOSED) || (ngState == NG_ATTACK)) {
      // Last sample in vReal is our current mic sample
    } else {
      micDataSm = 0;
    }

    FFT.windowing( FFTWindow::Hamming, FFTDirection::Forward);   // Weigh data
    FFT.compute( FFTDirection::Forward );                             // Compute FFT
    FFT.complexToMagnitude();                               // Compute magnitudes

    //
    // vReal[3 .. 255] contain useful data, each a 20Hz interval (60Hz - 5120Hz).
    // There could be interesting data at bins 0 to 2, but there are too many artifacts.
    //
    fftData_t peak, magnitude;
    FFT.majorPeak(peak, magnitude);      // let the effects know which freq was most dominant
    FFT_MajorPeak = double(peak);
    FFT_Magnitude = double(magnitude);
    /* This FFT post processing is a DIY endeavour. What we really need is someone with sound engineering expertise to do a great job here AND most importantly, that the animations look GREAT as a result.
    *
    *
    * Andrew's updated mapping of 256 bins down to the 16 result bins with Sample Freq = 10240, samples = 512 and some overlap.
    * Based on testing, the lowest/Start frequency is 60 Hz (with bin 3) and a highest/End frequency of 5120 Hz in bin 255.
    * Now, Take the 60Hz and multiply by 1.320367784 to get the next frequency and so on until the end. Then detetermine the bins.
    * End frequency = Start frequency * multiplier ^ 16
    * Multiplier = (End frequency/ Start frequency) ^ 1/16
    * Multiplier = 1.320367784
    */

    // Only adjust AGC when noise-gate is open
    if (!((ngState == NG_HOLD_CLOSED) || (ngState == NG_CLOSED))) {
      fftData_t fft_old, fft_new, avg_target;
      for (int bin = 0; bin < 16; bin++ ) {
            frequency_bin* freq_bin = &frequency_bins[bin];

            // We want the average to sit at ~10% of the spectrum between min and max
            avg_target = stats[bin].min + (stats[bin].max - stats[bin].min + 0.0001) * 0.1;

            // Grow min and decay max if the average moves away from the setpoint
            if (stats[bin].initialized) {
              if (stats[bin].avg > avg_target + 3.0) {
                stats[bin].min = min( stats[bin].min / ALPHA_MINMAX, stats[bin].avg - 1.0f);
              }

              if (stats[bin].avg < avg_target - 3.0) {
                stats[bin].max = max( stats[bin].max * ALPHA_MINMAX, stats[bin].avg + 1.0f);
              }
            }

            // Compute new FFT value for this bin
            fft_old = freq_bin->value;
            fft_new = 0.0;
            for (int cnt = freq_bin->start_bin; cnt <= freq_bin->end_bin; cnt++) {
              fft_new += vReal[cnt];
            }
            fft_new /= freq_bin->end_bin - freq_bin->start_bin + 1;

            // Smoothly apporach the new FFT value
            if (fft_new > fft_old) {
              fft_new = ALPHA_SIGNAL * fft_old + (1 - ALPHA_SIGNAL) * fft_new;
            } else {
              fft_new = ALPHA_SIGNAL * fft_old + (1 - ALPHA_SIGNAL) * fft_new;
            }
            freq_bin->value = fft_new;

            // Compute logarithmic value of bin
            fft_new = 10 * log10(pow(fft_new, 2));
            freq_bin->value_db = fft_new;

            // If new value is outside min<->max, update the respective limits
            if (!stats[bin].initialized) {
              stats[bin].min = fft_new - 1.0f;
              stats[bin].max = fft_new + 1.0f;
              stats[bin].avg = fft_new;
              stats[bin].initialized = true;
            } else {
              if (stats[bin].min > fft_new) {
                stats[bin].min = fft_new;
              }
              if (stats[bin].max < fft_new) {
                stats[bin].max = fft_new;
              }

              // Update rolling average
              stats[bin].avg = (1.0 - 1.0/300) * stats[bin].avg + fft_new / 300;
            }
        }
    }

    // If the Noise-Gate is closed, all final data should be 0 (fftBin simply doesn't get updated)
    if ((ngState == NG_HOLD_CLOSED) || (ngState == NG_CLOSED) || (ngState == NG_ATTACK)) {

      // FFTResult
      for (int i=0; i < 16; i++) {
        fftResult[i] = 0;
      }

      // micDataSm
      micDataSm = 0;

    // Noise-Gate is open, store actual results in their respective variables.
    } else {

      // FFTResult
      for (int i=0; i < 16; i++) {
        fftResult[i] =  map(frequency_bins[i].value_db, stats[i].min, stats[i].max, 0, 254);
      }

      // MicDataSm
      micDataSm = audioSource->getSampleWithoutDCOffset();

      // fftBin
      for (int i = 0; i < samples; i++) {
        fftData_t t = 0.0;
        t = abs(vReal[i]);
        t = t / 16.0; // Reduce magnitude. Want end result to be linear and ~4096 max.
        fftBin[i] = t;
      }
    }
    // We sample 512 samples at SAMPLE_RATE Khz. This takes, e.g. 20ms for 512 samples @ 10240 khz
    // So repeating the loop before SAMPLE_RATE/samples doesn't make sense, so pause the task for that time.
    vTaskDelayUntil(&xLastWakeTime, (SAMPLE_RATE/samples) / portTICK_RATE_MS);

  } // for(;;)
} // FFTcode()


void logAudio() {
#ifdef MIC_LOGGER


//  Serial.print(micIn);      Serial.print(" ");
//  Serial.print(sample); Serial.print(" ");
//  Serial.print(sampleAvg); Serial.print(" ");
//  Serial.print(sampleAgc);  Serial.print(" ");
//  Serial.print(micData);    Serial.print(" ");
//  Serial.print(micDataSm);  Serial.print(" ");
  Serial.println(" ");

#endif

#ifdef MIC_SAMPLING_LOG
  //------------ Oscilloscope output ---------------------------
  Serial.print(targetAgc); Serial.print(" ");
  Serial.print(multAgc); Serial.print(" ");
  Serial.print(sampleAgc); Serial.print(" ");

  Serial.print(sample); Serial.print(" ");
  Serial.print(sampleAvg); Serial.print(" ");
  Serial.print(micLev); Serial.print(" ");
  Serial.print(samplePeak); Serial.print(" ");    //samplePeak = 0;
  Serial.print(micIn); Serial.print(" ");
  Serial.print(100); Serial.print(" ");
  Serial.print(0); Serial.print(" ");
  Serial.println(" ");
#endif

#ifdef FFT_SAMPLING_LOG
  #if 0
    for(int i=0; i<16; i++) {
      Serial.print(fftResult[i]);
      Serial.print("\t");
    }
    Serial.println("");
  #endif

  // OPTIONS are in the following format: Description \n Option
  //
  // Set true if wanting to see all the bands in their own vertical space on the Serial Plotter, false if wanting to see values in Serial Monitor
  const bool mapValuesToPlotterSpace = false;
  // Set true to apply an auto-gain like setting to to the data (this hasn't been tested recently)
  const bool scaleValuesFromCurrentMaxVal = false;
  // prints the max value seen in the current data
  const bool printMaxVal = false;
  // prints the min value seen in the current data
  const bool printMinVal = false;
  // if !scaleValuesFromCurrentMaxVal, we scale values from [0..defaultScalingFromHighValue] to [0..scalingToHighValue], lower this if you want to see smaller values easier
  const int defaultScalingFromHighValue = 256;
  // Print values to terminal in range of [0..scalingToHighValue] if !mapValuesToPlotterSpace, or [(i)*scalingToHighValue..(i+1)*scalingToHighValue] if mapValuesToPlotterSpace
  const int scalingToHighValue = 256;
  // set higher if using scaleValuesFromCurrentMaxVal and you want a small value that's also the current maxVal to look small on the plotter (can't be 0 to avoid divide by zero error)
  const int minimumMaxVal = 1;

  int maxVal = minimumMaxVal;
  int minVal = 0;
  for(int i = 0; i < 16; i++) {
    if(fftResult[i] > maxVal) maxVal = fftResult[i];
    if(fftResult[i] < minVal) minVal = fftResult[i];
  }
  for(int i = 0; i < 16; i++) {
    Serial.print(i); Serial.print(":");
    Serial.printf("%04d ", map(fftResult[i], 0, (scaleValuesFromCurrentMaxVal ? maxVal : defaultScalingFromHighValue), (mapValuesToPlotterSpace*i*scalingToHighValue)+0, (mapValuesToPlotterSpace*i*scalingToHighValue)+scalingToHighValue-1));
  }
  if(printMaxVal) {
    Serial.printf("maxVal:%04d ", maxVal + (mapValuesToPlotterSpace ? 16*256 : 0));
  }
  if(printMinVal) {
    Serial.printf("%04d:minVal ", minVal);  // printed with value first, then label, so negative values can be seen in Serial Monitor but don't throw off y axis in Serial Plotter
  }
  if(mapValuesToPlotterSpace)
    Serial.printf("max:%04d ", (printMaxVal ? 17 : 16)*256); // print line above the maximum value we expect to see on the plotter to avoid autoscaling y axis
  else
    Serial.printf("max:%04d ", 256);
  Serial.println();
#endif // FFT_SAMPLING_LOG
} // logAudio()
