#include "Arduino.h"
#include <ArduinoSTL.h>
#include "LEDController.h"
#include "Math.h"

#define SERIAL_DEBUG 1  // set this to 1 to get echo, 2 to get debug info

// LEDMode definitions
#define OFF 0         // LEDs are OFF
#define ON 1          // LEDs are ON at constant intensity (i.e. were set using setRGB or setHSI)
#define PULSING 3     // LEDs are pulsing
#define BLINKING 4    // LEDs are blinking
#define MUSICING 5   // LEDs are lazorbeems

// LEDColor definitions
#define CONSTANT 0      // LEDs are constant color
#define FORWARD_HUE 1   // LEDs are cycling through hue
#define BACKWARD_HUE 2  // LEDs are cycling through hue backwards

#define BUTTON_DEBOUNCE 500 // button debouncer

#define SWITCH_PIN 4 // digital D4

using namespace std;

LEDController::LEDController(HardwareSerial *SerialToUse, String name) {
  BTName            = name;
  serial            = SerialToUse;
  LEDMode           = ON;  // default to ON
  LEDColor          = FORWARD_HUE;  // default to FORWARD HUE
  relativeTime      = 0;
  redBrightness     = 0;
  greenBrightness   = 0;
  blueBrightness    = 0;

  hue               = 0.0;
  saturation        = 1.0;
  intensity         = 1.0;

  hueFrequency      = 1.0 / 60.0 / 30.0; // Hz (30 minutes)
  hueMin            = 0;
  hueMax            = 360;

  blinkFrequency    = 1.0; // Hz;

  lastSerialTime    = 0;
  lastHeartbeat     = 0;

  btnDebounce       = 0;
}

void LEDController::begin(int baud) {
  serial->begin(baud);
}

void LEDController::init() {
  delay(200);
  String ATName = "AT+NAME" + BTName;
  serial->write(ATName.c_str());
  delay(1000);
  serial->write("AT+PIN1234");
  delay(1000);
  mReader = new MicrophoneReader(0.6); // 0.6 is alpha for first-order filter
}

void LEDController::doTheThings() {
  //readBT();
  doLED();
  // if (millis() - lastHeartbeat > HEARTBEAT) {
  //   heartbeat();
  //   lastHeartbeat = millis();
  // }
  if (millis() - btnDebounce > BUTTON_DEBOUNCE && digitalRead(SWITCH_PIN) == LOW) {
    switch (LEDMode) {
      case ON:
        LEDMode = MUSICING;
        serial->println("Button pressed! Switching music ON");
        break;
      case MUSICING:
        LEDMode = ON;
        serial->println("Button pressed! Switching music OFF");
        intensity = 1.0;
        break;
    }
    btnDebounce = millis();
  }
}

void LEDController::doLED() {
  unsigned long t = millis();
  unsigned long dt = millis() - relativeTime;

  /* COLOR */
  if (LEDColor == FORWARD_HUE) {
    hue = sawtooth(dt, hueFrequency, hueMin, hueMax);
  } else if (LEDColor == BACKWARD_HUE) {
    hue = sawtooth(dt, hueFrequency, hueMax, hueMin);
  }

  /* BRIGHTNESS */
  switch (LEDMode) {
    case OFF:
      intensity = 0.0;
      break;
    case ON: // intensity should not be set to 1.0 here because then HSI control doesn't work
      // intensity = 1.0;
      break;
    case PULSING:
      intensity = sinWave(t, blinkFrequency, 0.0, 1.0);
      break;
    case BLINKING:
      {
        long period = 1000 / blinkFrequency; // period in milliseconds
        intensity = (float) (((int)(t / period)) % 2); // this works because black magic
      }
      break;
    case MUSICING:
      intensity = mReader->read();
      break;
  } // switch (LEDMode)
  vector<int> rgb = hsiToRgb(hue, saturation, intensity);
  redBrightness   = rgb[0];
  greenBrightness = rgb[1];
  blueBrightness  = rgb[2];
}

void LEDController::readBT() {
  // check if timeout has been reached
  if (millis() - lastSerialTime > BT_BUFFER_TIMEOUT && BTBuffer.size() > 0) {
    BTBuffer.clear();
    #if SERIAL_DEBUG >= 1
      Serial.println("Timeout reached - clearing buffer");
    #endif
  }

  // first check if anything is available on the serial connection
  if (Serial.available() > 0) {
    // save current time
    lastSerialTime = millis();
    // read incoming byte
    byte inByte = Serial.read();

    // add byte to buffer
    BTBuffer.push_back(inByte);

    #if SERIAL_DEBUG >= 2
      Serial.print("[");
      Serial.print(BTBuffer.size());
      Serial.print("]");
      for (int i = 0; i < BTBuffer.size(); i++) {
        if (i == BTBuffer.size() - 1) {
          Serial.println(BTBuffer[i]);
        } else {
          Serial.print(BTBuffer[i]);
          Serial.print('.');
        }
      }
    #endif

  } else {
    // delimiter reached, check if the packet is valid
    if (BTBuffer.size() == 7 // { i 1 2 3 4 } 7 bytes
        && BTBuffer[0] == PACKET_START
        && BTBuffer[6] == PACKET_END) {
      #if SERIAL_DEBUG >= 2
        Serial.println("Delimiter reached!");
      #endif
      // packet is valid
      // determine what to do based on identifier
      switch (BTBuffer[1]) {
        case CYCLE_HUE:
          setHueFrequency();
          break;
        case SET_RGB:
          setRGB();
          break;
        case SET_HSV:
          setHSI();
          break;
        case BLINK:
          LEDMode = BLINKING;
          setBlinkFrequency();
          break;
        case PULSE:
          LEDMode = PULSING;
          setBlinkFrequency();
          break;
        case SET_INT:
          setIntensity();
          break;
        case MUSIC:
          setMusic();
          break;
        default:
          // do nothing
          break;
      } // switch (BTBuffer[1])

      BTBuffer.clear();
    } else if (BTBuffer.size() > 7){
      // filled with shit, clear it
      BTBuffer.clear();
      #if SERIAL_DEBUG >= 1
        Serial.println("Clearing buffer");
      #endif
    }
  }
}

void LEDController::setRGB() {
  // LEDMode = ON;
  // LEDColor = CONSTANT;

  redBrightness = (int) BTBuffer[2];
  blueBrightness = (int) BTBuffer[3];
  greenBrightness = (int) BTBuffer[4];

  vector<float> hsi = rgbToHsi((float) redBrightness, (float) greenBrightness, (float) blueBrightness);
  hue = hsi[0];
  saturation = hsi[1];
  intensity = hsi[2];

  #if SERIAL_DEBUG >= 1
    char buffer[100];
    sprintf(buffer, "Setting RGB to R:%d G:%d B:%d --- H:%d S:%d I:%d",
            redBrightness,
            blueBrightness,
            greenBrightness,
            (int) hue,
            (int) (saturation * 100.0),
            (int) (intensity * 100.0));
    Serial.println(buffer);
  #endif
}

void LEDController::setHSI() {
  // LEDMode = ON;
  LEDColor = CONSTANT;

  hue = ((float)(int) BTBuffer[2]) * 10.0 + ((float)(int) BTBuffer[3]) / 10.0;
  hue = constrain(hue, 0, 360.0);

  saturation = ((float)(int) BTBuffer[4]) / 100.0;
  saturation = constrain(saturation, 0, 1.0);

  intensity = ((float)(int) BTBuffer[5]) / 100.0;
  intensity = constrain(intensity, 0, 1.0);

  if ((int) BTBuffer[5] == 255) LEDMode = OFF; // send 255 on intensity to turn LEDs off

  vector<int> rgb = hsiToRgb(hue, saturation, intensity);
  redBrightness = rgb[0];
  greenBrightness = rgb[1];
  blueBrightness = rgb[2];
  #if SERIAL_DEBUG >= 1
    char buffer[100];
    sprintf(buffer, "Setting HSV to H:%d S:%d I:%d --- R:%d G:%d B:%d",
            (int) hue,
            (int) (saturation * 100.0),
            (int) (intensity * 100.0),
            redBrightness,
            blueBrightness,
            greenBrightness);
    Serial.println(buffer);
  #endif
}

void LEDController::setBlinkFrequency() {
  long blinkPeriodMS = (long) BTBuffer[2] * 100 + (long) BTBuffer[3];
  if (blinkPeriodMS < 100) blinkPeriodMS = 100; // keep it safe

  #if SERIAL_DEBUG >= 1
    char buffer[100];
    sprintf(buffer, "Blinking LEDs with period %ldms", blinkPeriodMS);
    Serial.println(buffer);
  #endif
  blinkFrequency = 1000.0 / (float) blinkPeriodMS;
}

void LEDController::setHueFrequency() {
  LEDColor = FORWARD_HUE;
  relativeTime = millis();
  LEDMode = ON;

  long huePeriodMS = (long) BTBuffer[2] * 1000 * 60;
  hueFrequency = 1000.0 / (float) huePeriodMS;

  hue = 0;
  saturation = ((float)(int) BTBuffer[3]) / 100.0;
  saturation = constrain(saturation, 0, 1.0);

  intensity = ((float)(int) BTBuffer[4]) / 100.0;
  intensity = constrain(intensity, 0, 1.0);

  #if SERIAL_DEBUG >= 1
    char buffer[100];
    sprintf(buffer, "Cycling hue forwards with period %lds, intensity %d, saturation %d", huePeriodMS / 1000, int(saturation * 100), int(intensity * 100));
    Serial.println(buffer);
  #endif
}

void LEDController::setIntensity() {
  intensity = ((float)(int) BTBuffer[2]) / 100.0;
  intensity = constrain(intensity, 0, 1.0);

  if (intensity > 0) {
    LEDMode = ON;
  } else {
    LEDMode = OFF;
  }

  #if SERIAL_DEBUG >= 1
    char buffer[100];
    sprintf(buffer, "Setting intensity to %d", int(intensity * 100));
    Serial.println(buffer);
  #endif
}

void LEDController::heartbeat() {
  // send current status
  #if SERIAL_DEBUG >= 1
    Serial.write(123);
    Serial.write(int(hue));
    Serial.write(int(saturation * 100));
    Serial.write(int(intensity * 100));
    Serial.write(125);
  #endif
}

void LEDController::setMusic() {
  if ((int) BTBuffer[2] == 1) {
    LEDMode = MUSICING;
    #if SERIAL_DEBUG >= 1
      Serial.println("Turning music mode on");
    #endif
  } else {
    LEDMode = ON;
    #if SERIAL_DEBUG >= 1
      Serial.println("Turning music mode off");
    #endif
  }
}


float LEDController::sinWave(long t, float f, float HUE_MIN, float HUE_MAX) {
  return (float) (HUE_MAX - HUE_MIN) / 2 * (1 + sin(2 * M_PI * f * t / 1000 - M_PI_2));
}

float LEDController::sawtooth(long t, float f, float HUE_MIN, float HUE_MAX) {
  long period = 1000 / f; // period in milliseconds
  return (float) HUE_MIN + (float) (HUE_MAX-HUE_MIN) * (float) (t % period) / (float) period;
}

vector<int> LEDController::hsvToRgb(float h, float s, float v) {
  //https://en.wikipedia.org/wiki/HSL_and_HSV#From_HSV
  float r, g, b;

  float c = v * s;
  float h_ = (float) h / 60;
  float x = c * (1 - abs(fmodf(h_, 2) - 1));

  if      (h_ < 1) { r = c, g = x, b = 0; }
  else if (h_ < 2) { r = x, g = c, b = 0; }
  else if (h_ < 3) { r = 0, g = c, b = x; }
  else if (h_ < 4) { r = 0, g = x, b = c; }
  else if (h_ < 5) { r = x, g = 0, b = c; }
  else if (h_ < 6) { r = c, g = 0, b = x; }

  vector<int> rgb;
  rgb.push_back((int) (r * 255));
  rgb.push_back((int) (g * 255));
  rgb.push_back((int) (b * 255));

  return rgb;
}

vector<float> LEDController::rgbToHsv(int r, int g, int b) {
  // http://www.rapidtables.com/convert/color/rgb-to-hsv.htm
	float h = 0, s, v;

  float r_ = r / 255;
  float g_ = g / 255;
  float b_ = b / 255;

  float Cmax = max(max(r_, g_), b_);
  float Cmin = min(min(r_, g_), b_);
  float delta = Cmax - Cmin;

  if (delta == 0) h = 0;
  else if (Cmax == r_) h = 60 * fmodf((g_ - b_) / delta, 6);
  else if (Cmax == g_) h = 60 * ((b_ - r_) / delta + 2);
  else if (Cmax == b_) h = 60 * ((r_ - g_) / delta + 4);

  if (Cmax == 0) s = 0;
  else s = delta / Cmax;

  v = Cmax;

  vector<float> hsv;
  hsv.push_back(h);
  hsv.push_back(s);
  hsv.push_back(v);

  return hsv;
}

vector<int> LEDController::hsiToRgb(float H, float S, float I) {
  // http://blog.saikoled.com/post/43693602826/why-every-led-light-should-be-using-hsi
  float r, g, b;
  H = fmodf(H, 360); // cycle H around to 0-360 degrees
  H = 3.14159 * H / 180.0; // Convert to radians.
  S = S > 0 ? (S < 1 ? S : 1) : 0; // clamp S and I to interval [0,1]
  I = I > 0 ? (I < 1 ? I : 1) : 0;

  // Math! Thanks in part to Kyle Miller.
  if(H < 2.09439) {
    r = 255 * I / 3 * (1 + S * cos(H) / cos(1.047196667 - H));
    g = 255 * I / 3 * (1 + S*(1 - cos(H) / cos(1.047196667 - H)));
    b = 255 * I / 3 * (1 - S);
  } else if(H < 4.188787) {
    H = H - 2.09439;
    g = 255 * I / 3 * (1 + S * cos(H) / cos(1.047196667 - H));
    b = 255 * I / 3 * (1 + S * (1 - cos(H) / cos(1.047196667 - H)));
    r = 255 * I / 3 * (1 - S);
  } else {
    H = H - 4.188787;
    b = 255 * I / 3 * (1 + S * cos(H) / cos(1.047196667 - H));
    r = 255 * I / 3 * (1 + S * (1 - cos(H) / cos(1.047196667 - H)));
    g = 255 * I / 3 * (1 - S);
  }
  vector<int> rgb;
  rgb.push_back((int) r);
  rgb.push_back((int) g);
  rgb.push_back((int) b);

  return rgb;
}

vector<float> LEDController::rgbToHsi(float r, float g, float b) {
  // http://www.had2know.com/technology/hsi-rgb-color-converter-equations.html
  // http://fourier.eng.hmc.edu/e161/lectures/ColorProcessing/node2.html
  r /= 255.0;
  g /= 255.0;
  b /= 255.0;

  float I = r + g + b / 3;
  I = constrain(I, 0, 255);

  float m = min(min(r, g), b);
  float S;
  if (I == 0) S = 0;
  else S = 1 - m/I;

  float H = acos(0.5 * (r - g + r - b) / sqrt((r-g)*(r-g)+(r-b)*(g-b)));

  vector<float> hsi;
  hsi.push_back(H);
  hsi.push_back(S);
  hsi.push_back(I);

  return hsi;
}
