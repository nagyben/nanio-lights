#ifndef MicrophoneReader_H
#define MicrophoneReader_H
#include "Arduino.h"

const int MIC_IN = A0;
const int CALIBRATE_SAMPLES = 50;
const float A_LIM = 100;

class MicrophoneReader {
  private:
    float audio_in;
    float alpha;
    float Aint;
    float dA;
    float convert(float);
    float calibrateMicrophone();

  public:
    MicrophoneReader(float _alpha);
    float read();
};

#endif
