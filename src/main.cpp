#include "Arduino.h"
#include <ArduinoSTL.h>
#include <LEDController.h>
#include <MicrophoneReader.h>

const int LED_R = 5; // digital output pin for LED red channel
const int LED_G = 3; // digital output pin for LED green channel
const int LED_B = 6; // digital output pin for LED blue channel

unsigned int redBrightness = 0;
unsigned int greenBrightness = 0;
unsigned int blueBrightness = 0;

LEDController led(&Serial, "NanIO-TV");

void setup() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  led.begin(9600);
  led.init();
}

void loop() {
  pinMode(4, INPUT_PULLUP);
  led.doTheThings();

  // now led contains the rgb output

  analogWrite(LED_R, led.redBrightness);
  analogWrite(LED_G, led.greenBrightness);
  analogWrite(LED_B, led.blueBrightness);
}
