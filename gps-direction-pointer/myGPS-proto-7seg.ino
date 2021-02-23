/*
 * Feb07, 2021, correction when angle is bigger than half turn
 * change some pin usage
 * Jan30-2021
 * destination pointer proto
 * 7 seg version
 */
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>  // metro mini: data->A4, clock->A5
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
Adafruit_7segment matrix = Adafruit_7segment();
Adafruit_7segment matrix2 = Adafruit_7segment();

/*
 * gps
 */
// Connect the GPS TX (transmit) pin to Digital 8
// Connect the GPS RX (receive) pin to Digital 9
// you can change the pin numbers to match your wiring
SoftwareSerial mySerial(8, 9); // Rx, Tx of arduino
Adafruit_GPS GPS(&mySerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

double mov_lat;
double mov_lon;
double dest_lat;
double dest_lon;

// (0) home
// (1) school1
// (2) work
// (3) school2
// (4) dojo
// (5) station
const double dest_lats[] = {
  // list of latitudes
};
const double dest_lons[] = {
  // list of longitudes
};
int max_dest_num = 5;

/*
 * button
 */
const int buttonPin = 2;
int buttonState = 0;
int dest_num = 0;

/*
 * LEDs
 */
const int greenPin = A1;  // was 5.  changed to 1, 12, not working, 0 dim all the time.
const int redPin = 3;
int greenState = LOW;
int redState = LOW;

/*
 * single 7 seg on SPI
 */
// SCK D13
// LATCH D10
// SDI D11
#include<SPI.h>
int sck = 13;
int latch = 10;
int sdi = 11;
int scroll_speed = 200;
const byte digits[] = {
  0b11111100, // 0
  0b01100000, // 1
  0b11011010, // 2
  0b11110010, // 3
  0b01100110, // 4
  0b10110110, // 5
  0b10111110, // 6
  0b11100000, // 7
  0b11111110, // 8
  0b11110110, // 9
};

void setup()
{
  /*
   * button
   */
  pinMode(buttonPin, INPUT);

  /*
   * single 7 seg
   */
  pinMode(latch, OUTPUT);
  pinMode(sck, OUTPUT);
  pinMode(sdi, OUTPUT);
  SPI.begin();
  SPI.setBitOrder(LSBFIRST);
  SPI.setDataMode(0);

  for (int i = 0; i <= max_dest_num; i++) {
    digitalWrite(latch, 0);
    SPI.transfer (digits[i]); // display initial dest num
    digitalWrite(latch, 1);
    delay(500);
  }

  digitalWrite(latch, 0);
  SPI.transfer (digits[dest_num]); // display initial dest num
  digitalWrite(latch, 1);
  delay(1000);

  /*
   * LEDs
   */
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);
  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, HIGH);
  delay(1000);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, LOW);

  /*
   * set initial dest lat, lon
   */
  dest_lat = dest_lats[dest_num];
  dest_lon = dest_lons[dest_num];

  /*
   * Serial
   */
  // connect at 115200 so we can read the GPS fast enough and echo without
  // dropping chars
  Serial.begin(115200);
  delay(1000);
  Serial.println("myGPS-proto");

  /*
   * 7seg
   */
  matrix.begin(0x70); // green
  matrix2.begin(0x74); // amber
  //matrix2.println(10.23);
  //matrix2.writeDisplay();
  delay(2000);
  // print with print/println
  for (uint16_t counter = 0; counter < 1001; counter++) {
    matrix.println(counter);
    matrix.writeDisplay();
    matrix2.println(counter);
    matrix2.writeDisplay();
    delay(5);
  }
  delay(1000);

  /*
   * GPS
   */
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

}


uint32_t timer = millis();
void loop()
{  // --------------------------------------------------------- loop()
  /*
   * button
   */
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    // increment dest_num
    if (dest_num == max_dest_num) {
      dest_num = 0;
    } else {
      dest_num += 1;
    }
    // update dest_num display
    delay(100);
    digitalWrite(latch, 0);
    SPI.transfer (digits[dest_num]);//1の桁 // display initial dest num
    digitalWrite(latch, 1);
    delay(500);
    // update dest_lat, dest_lon
    dest_lat = dest_lats[dest_num];
    dest_lon = dest_lons[dest_num];
  }

  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO)) {
    Serial.write(c);
  }

  // ======== DO I NEED THIS PART??? ==========
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) {
    timer = millis(); // reset the timer

    Serial.print("\nFix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);

    if (GPS.fix) {  //---------------------------------------------(1)
      mov_lat = GPS.latitudeDegrees;
      mov_lon = GPS.longitudeDegrees;
      long d_to_dest = CalcDistance(mov_lat, mov_lon, dest_lat, dest_lon);
      int bearing = CalcBearing(mov_lat, mov_lon, dest_lat, dest_lon);
      int angle_to_dest = bearing - (int) GPS.angle;
      // when bigger than half turn
      if (angle_to_dest > 180) {
        angle_to_dest = angle_to_dest - 360;
      } else if (angle_to_dest < -180) {
        angle_to_dest = angle_to_dest + 360;
      }

      /*
       * 7 seg: angle
       */
      matrix.print(angle_to_dest);
      matrix.writeDisplay();
      if (d_to_dest < 10000) {
        matrix2.print(d_to_dest);
      } else {
        matrix2.print(d_to_dest/1000.0);
      }
      matrix2.writeDisplay();

      Serial.print("Location: ");
      Serial.print(GPS.latitudeDegrees, 6); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitudeDegrees, 6); Serial.println(GPS.lon);
      Serial.print(mov_lat, 10); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(mov_lon, 10); Serial.println(GPS.lon);

      Serial.print("# Distance: ");
      Serial.print(d_to_dest);
      Serial.println(" m");
      Serial.print("# Bearing: ");
      Serial.print(bearing);
      Serial.println(" degree");

      Serial.print("### Speed (knots): "); Serial.println(GPS.speed);

      if (GPS.speed >= 1){ // moving fast enough
        if (greenState == LOW) {
          greenState = HIGH;
          redState = LOW;
        }
      } else { // moving too slow or not moving
        if (greenState == HIGH) {
          greenState = LOW;
          redState = HIGH;
        }
        if (redState == LOW){
          redState = HIGH;
          greenState = LOW;
        }
      }
      digitalWrite(greenPin, greenState);
      digitalWrite(redPin, redState);

      Serial.print("### moving! ::: Angle: "); Serial.println(GPS.angle);
      // Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("### Satellites: "); Serial.println((int)GPS.satellites);
    } else { // ------------------- else of GPS.fix
      // fix is 0
      // try to print a number thats too long
      matrix.print(10000, DEC);
      matrix.writeDisplay();
      matrix2.print(10000, DEC);
      matrix2.writeDisplay();
      // off both LEDs
      greenState = LOW;
      redState = LOW;
      digitalWrite(greenPin, greenState);
      digitalWrite(redPin, redState);
    }
  }  // ---------------------- end of (millis() - timer > 2000)
} // ------------------------- end of loop()


/*
 * functions
 */
// function from https://forum.arduino.cc/index.php?topic=45760.0
//convert degrees to radians
double dtor(double fdegrees)
{
  return(fdegrees * M_PI / 180);
}

//Convert radians to degrees
double rtod(double fradians)
{
  return(fradians * 180.0 / M_PI);
}

//Calculate distance form lat1/lon1 to lat2/lon2 using haversine formula
//Note lat1/lon1/lat2/lon2 must be in radians
//Returns distance in feet
long CalcDistance(double lat1, double lon1, double lat2, double lon2)
{
  double dlon, dlat, a, c;
  double dist = 0.0;
  dlon = dtor(lon2 - lon1);
  dlat = dtor(lat2 - lat1);
  a = pow(sin(dlat/2),2) + cos(dtor(lat1)) * cos(dtor(lat2)) * pow(sin(dlon/2),2);
  c = 2 * atan2(sqrt(a), sqrt(1-a));

  // dist = 20925656.2 * c;
  // radius of the earth (6378140 meters) in feet 20925656.2
  dist = 6378140.0 * c;
  return( (long) dist + 0.5);
}

// Calculate bearing from lat1/lon1 to lat2/lon2
// Note lat1/lon1/lat2/lon2 must be in radians
// Returns bearing in degrees
int CalcBearing(double lat1, double lon1, double lat2, double lon2)
{
  lat1 = dtor(lat1);
  lon1 = dtor(lon1);
  lat2 = dtor(lat2);
  lon2 = dtor(lon2);

  // determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtod(bearing);
  //use mod to turn -90 = 270
  bearing = fmod((bearing + 360.0), 360);
  return (int) bearing + 0.5;
}

// void ComputeDestPoint(double lat1, double lon1, int iBear, int iDist, double *lat2, double *lon2)
//{
//  double bearing = dtor((double) iBear);
//  double dist = (double) iDist / 20925656.2;
//  lat1 = dtor(lat1);
//  lon1 = dtor(lon1);
//  *lat2 = asin(sin(lat1)* cos(dist)+ cos(lat1)* sin(dist)*cos(bearing));
//  *lon2 = lon1 + atan2(sin(bearing)*sin(dist)*cos(lat1), cos(dist)-sin(lat1)*sin(*lat2));
//  *lon2 = fmod( *lon2 + 3 * M_PI, 2*M_PI )- M_PI;
//  *lon2 = rtod( *lon2);
//  *lat2 = rtod( *lat2);
//}
