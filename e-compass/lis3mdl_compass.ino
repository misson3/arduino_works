// Nov01, 2020, ms
// lis3mdl_compass
// modified from lis3mdl_demo-compass.ino

// NeoPixel part
// Adafruit NeoPixel/simpe.ino

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3MDL lis3mdl;  // I use I2C.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN 1 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 16

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


void setup(void) {
  Serial.begin(115200);
  //while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Compass with Adafruit LIS3MDL!");
  
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  // NeoPixel
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  // use this totest color and brightness
  startPixel(0, 0, 20);
  startPixel(0, 20, 0);
  startPixel(20, 0, 0);
}


void loop() {
  lis3mdl.read();      // get X Y and Z data at once
  // Then print out the raw data
  //  Serial.print("\nX:  "); Serial.print(lis3mdl.x); 
  //  Serial.print("  \tY:  "); Serial.print(lis3mdl.y); 
  //  Serial.print("  \tZ:  "); Serial.println(lis3mdl.z); 

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  //  Serial.print("\tX: "); Serial.print(event.magnetic.x);
  //  Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
  //  Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
  //  Serial.println(" uTesla ");

  float Pi = 3.14159;
  // offset I got with Adafruit Sensor Lab/cariblation/mag_hardiron_simplecal
  float x_os = 51.21;
  float y_os = -36.21;
  float z_os = -4.44;
  
  // Calculate the angle of the vector y,x
  float heading = (atan2(event.magnetic.y - y_os, event.magnetic.x - x_os) * 180) / Pi;

  Serial.print("Compass Heading: ");
  Serial.println(heading);

  displayHeading(heading);
  delay(100); 

}


void displayHeading(float heading) {
  pixels.clear();
  int i, j; // to point N
  int m, n; // I am heading to
  if (heading >= -11.25 && heading < 11.25) {
    i = 0;
    j = 15;
    m = 0;
    n = 15;
  }else if (heading >= 11.25 && heading < 33.75) {
    i = 0;
    j = 1;
    m = 14;
    n = 15;
  }else if (heading >= 33.75 && heading < 56.25) {
    i = 1;
    j = 2;
    m = 13;
    n = 14; 
  }else if (heading >= 56.25 && heading < 78.75) {
    i = 2;
    j = 3;
    m = 12;
    n = 13; 
  }else if (heading >= 78.75 && heading < 101.25) {
    i = 3;
    j = 4;
    m = 11;
    n = 12; 
  }else if (heading >= 101.25 && heading < 123.75) {
    i = 4;
    j = 5;
    m = 10;
    n = 11; 
  }else if (heading >= 123.75 && heading < 146.25) {
    i = 5;
    j = 6;
    m = 9;
    n = 10; 
  }else if (heading >= 146.25 && heading < 168.75) {
    i = 6;
    j = 7;
    m = 8;
    n = 9; 
  }else if (heading >= 168.75 && heading < 191.25) {
    i = 7;
    j = 8;
    m = 7;
    n = 8; 
  }else if (heading >= -191.25 && heading < -168.75) {
    i = 7;
    j = 8;
    m = 7;
    n = 8;
  }else if (heading >= -168.75 && heading < -146.25) {
    i = 8;
    j = 9;
    m = 6;
    n = 7;
  }else if (heading >= -146.25 && heading < -123.75) {
    i = 9;
    j = 10;
    m = 5;
    n = 6;
  }else if (heading >= -123.75 && heading < -101.25) {
    i = 10;
    j = 11;
    m = 4;
    n = 5;
  }else if (heading >= -101.25 && heading < -78.75) {
    i = 11;
    j = 12;
    m = 3;
    n = 4;
  }else if (heading >= -78.75 && heading < -56.25) {
    i = 12;
    j = 13;
    m = 2;
    n = 3;
  }else if (heading >= -56.25 && heading < -33.75) {
    i = 13;
    j = 14;
    m = 1;
    n = 2;
  }else if (heading >= -33.75 && heading < -11.25) {
    i = 14;
    j = 15;
    m = 0;
    n = 1;
  }else{
    // ???
  }

  int A = 10;
  if ((i == 0 && j == 15) || (i == 7 && j == 8)) {
    // North: i = 0, j = 15, m = 0, n = 15
    // South: i = 7, j = 8, i = 7, j = 8
    pixels.setPixelColor(i, pixels.Color(A, 0, A));
    pixels.setPixelColor(j, pixels.Color(A, 0, A));
  } else {
    pixels.setPixelColor(i, pixels.Color(A, 0, 0));
    pixels.setPixelColor(j, pixels.Color(A, 0, 0));
    pixels.setPixelColor(m, pixels.Color(0, 0, A));
    pixels.setPixelColor(n, pixels.Color(0, 0, A));
  }

  pixels.show();
}


void startPixel(int r, int g, int b) {
  pixels.clear(); // Set all pixel colors to 'off'

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<NUMPIXELS; i++) { // For each pixel...

    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    pixels.setPixelColor(i, pixels.Color(r, g, b));

    pixels.show();  // Send the updated pixel colors to the hardware.

    delay(20);
  }
  delay(100);
  pixels.clear();
  pixels.show();
}
