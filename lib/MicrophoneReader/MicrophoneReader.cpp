#include "Arduino.h"
#include "MicrophoneReader.h"

MicrophoneReader::MicrophoneReader(float _alpha) {
  pinMode(MIC_IN, INPUT);
  Aint = -0.1 - calibrateMicrophone() / A_LIM;
  dA = 1.0 / A_LIM;
  alpha = _alpha;
  audio_in = 0;
}

float MicrophoneReader::read() {
  audio_in = ((float) analogRead(MIC_IN)) * (1.0 - alpha) + audio_in * alpha;
  // Serial.println(audio_in);
  return convert(audio_in);
}

float MicrophoneReader::convert(float a) {
  float ret = Aint + dA * a;
  if (ret < 0) ret = 0;
  if (ret > 1) ret = 1;
  return ret;
}

float MicrophoneReader::calibrateMicrophone() {
  long sampleSum = 0;

  // Gather data points
  for (int i = 0; i < CALIBRATE_SAMPLES; i++) {
    sampleSum += analogRead(MIC_IN);
  }
  return (float) ((float) sampleSum / (float)CALIBRATE_SAMPLES);
}
