/*
 * Dec05, 2021, ms
 * modify FSR (Force Sensitive Resistor) testing sketch from Adafruit page
 * ref: https://learn.adafruit.com/force-sensitive-resistor-fsr/using-an-fsr
 * ref: circuitplayground demo code for pixel control
 */

#include <Adafruit_CircuitPlayground.h>

// we light one pixel at a time, this is our counter
uint8_t pixeln = 0;
uint8_t i = 0;

// pin usage on circuitplaygroud classic
int fsrAnalogPin = 9; // 0; // FSR is connected to analog 0
int LEDpin = 6;       // 11;      // connect Red LED to pin 11 (PWM pin)
int fsrReading;       // the analog reading from the FSR resistor divider
int LEDbrightness;

 
void setup(void) {
  Serial.begin(9600);
  pinMode(LEDpin, OUTPUT);

  CircuitPlayground.begin();
}


void loop(void) {
  fsrReading = analogRead(fsrAnalogPin);
  Serial.print("Analog reading = ");
  Serial.println(fsrReading);
 
  // we'll need to change the range from the analog reading (0-1023) down to the range
  // used by analogWrite (0-255) with map!
  LEDbrightness = map(fsrReading, 0, 1023, 0, 255);
  // LED gets brighter the harder you press
  analogWrite(LEDpin, LEDbrightness);

  // neopixel
  pixeln =map(fsrReading, 0, 1023, 0, 11);  // 0 to 11, looks end exclusive
  Serial.print("maped value: ");
  Serial.println(pixeln);

  /************* 10 NEOPIXELS */
  CircuitPlayground.clearPixels();

  if (pixeln > 0) {  // no pixcel when 0
    for (i=1;i<pixeln+1;i++) {
      CircuitPlayground.setPixelColor(i-1, CircuitPlayground.colorWheel(25 * i));
    }
  }

}
