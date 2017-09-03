#ifndef LEDController_H
#define LEDController_H

#include "Arduino.h"
#include <ArduinoSTL.h>
#include "MicrophoneReader.h"

const byte CYCLE_HUE    = 72; // H
const byte CYCLE_HUE_B  = 73; // I
const byte SET_RGB      = 99; // c
const byte SET_HSV      = 104; // h
const byte OFF          = 120; // x
const byte PACKET_START = 123; // {
const byte PACKET_END   = 125; // }
const byte BLINK        = 98; // b
const byte PULSE        = 112; // p
const byte MUSIC        = 77; // M
const byte SET_INT      = 105; // i
const long HEARTBEAT    = 1000; // heartbeat interval (ms)

const unsigned long BT_BUFFER_TIMEOUT = 5000; // 5 second buffer clear timer

class LEDController {
private:
  HardwareSerial *serial;
  String BTName;

  std::vector<byte> BTBuffer;  // stores the serial packets

  float hue;
  float saturation;
  float intensity;

  float hueFrequency;
  float hueMin;
  float hueMax;

  float blinkFrequency;

  unsigned long relativeTime; // keeps track of relative time for time-based functions
  unsigned long lastSerialTime; // keeps track of the last time serial data was received (used for auto-clearing buffer after a defined time period)
  unsigned long lastHeartbeat; // keeps track of the heartbeat timer
  unsigned long btnDebounce; // button debouncer
  int LEDMode;  // keeps track of what the brightness is doing
  int LEDColor; // keeps track of what the colors are doing

  MicrophoneReader *mReader;

  void readBT(); // if data is available on the serial port, it will read it and determine what to do when the buffer has been filled
  std::vector<int> hsvToRgb(float h, float s, float v);
  std::vector<float> rgbToHsv(int r, int g, int b);
  std::vector<int> hsiToRgb(float H, float S, float I);
  std::vector<float> rgbToHsi(float r, float g, float b);
  void setRGB();
  void setHSI();
  void doLED();
  void setBlinkFrequency();
  void setHueFrequency();
  void setIntensity();
  void setMusic();
  void heartbeat();
  float convertAudioToLED(float);
  float sinWave(long t, float f, float HUE_MIN, float HUE_MAX);
  float sawtooth(long t, float f, float HUE_MIN, float HUE_MAX);

public:
  int redBrightness;
  int greenBrightness;
  int blueBrightness;
  LEDController(HardwareSerial *SerialToUse, String name);
  void begin(int baud);
  void doTheThings();
  void init();        // initialize BT module
  void setIntensity(float);
};

#endif
