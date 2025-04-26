/*
 * Jun 19, 2021, ms
 * wiinunchuk.h is required.
 * download it from Adafruit learning page.
 * https://learn.adafruit.com/nunchuket-kung-fu-pc-video-game-fighting-stick/program
*/

#include <Wire.h>
#include <ProTrinketKeyboard.h>
#include "wiinunchuk.h"

int loop_count = 0;
int range = 40;
int center = range/2;
int th_y = 5;
int th_x = 15;

void setup() {
  // open the serial port
  Serial.begin(9600);
  Serial.println("setup started."); 
  // initialize control over the keyboard:
  TrinketKeyboard.begin();
  nunchuk_setpowerpins();    // set power pins for Wii Nunchuk.  A2: GND, A3: 3.3V
  nunchuk_init(); 
  Serial.println("setup done!");
}

int x_skip = 0;
int y_skip = 0;

void loop() {
  TrinketKeyboard.poll();
  // the poll function must be called at least once every 10 ms
  // or cause a keystroke
  // if it is not, then the computer may think that the device
  // has stopped working, and give errors

  if (loop_count > 100) {
    loop_count = 0;

    if (nunchuk_get_data()) {
      // buttons
      int c_btn = nunchuk_cbutton();  // for PAGE_UP
      int z_btn = nunchuk_zbutton();  // for PAGE_DOWN

      if (c_btn) {
        TrinketKeyboard.pressKey(0, KEYCODE_PAGE_UP);
        TrinketKeyboard.pressKey(0, 0);
      } else if (z_btn) {
        TrinketKeyboard.pressKey(0, KEYCODE_PAGE_DOWN);
        TrinketKeyboard.pressKey(0, 0);
      }

      int xReading = nunchuk_joy_x();
      int yReading = nunchuk_joy_y();

      xReading = map(xReading, 26, 223, 0, range);

      int xDistance = xReading - center;
      if (abs(xDistance) < th_x) {
        xDistance = 0;
      }

      // to skip very first reading
      if (x_skip == 0) {
        xDistance = 0;
        x_skip = 1;
      }

      yReading = map(yReading, 35, 230, 0, range);
      int yDistance = yReading - center;
      if (abs(yDistance) < th_y) {
        yDistance = 0;
      }

      // to skip very first reading
      if (y_skip == 0) {
        yDistance = 0;
        y_skip = 1;
      }

      //Serial.print(xDistance);
      //Serial.print(", ");
      //Serial.println(yDistance);

      if (yDistance > 0) {
        TrinketKeyboard.pressKey(0, KEYCODE_ARROW_UP);
        TrinketKeyboard.pressKey(0, 0);
      } else if (yDistance < 0) {
        TrinketKeyboard.pressKey(0, KEYCODE_ARROW_DOWN);
        TrinketKeyboard.pressKey(0, 0);
      } else {
        // do nothing
      }

      if (xDistance > 0) {
        // TrinketKeyboard.pressKey(KEYCODE_MOD_LEFT_ALT, KEYCODE_ARROW_RIGHT);  // page moves as expected, but focus goes to "customize and control google chrome" and arrows moves focus on the menu.
        TrinketKeyboard.pressKey(KEYCODE_MOD_LEFT_CONTROL, KEYCODE_TAB);
        TrinketKeyboard.pressKey(0, 0);
      } else if (xDistance < 0) {
        //TrinketKeyboard.pressKey(KEYCODE_MOD_LEFT_ALT, KEYCODE_ARROW_LEFT);  // see above comment in xDistance > 0
        //TrinketKeyboard.pressKey(KEYCODE_MOD_LEFT_CONTROL, KEYCODE_MOD_LEFT_SHIFT, KEYCODE_TAB);  // does not do anything
        TrinketKeyboard.pressKey(KEYCODE_MOD_LEFT_CONTROL, KEYCODE_LEFT_SHIFT, KEYCODE_TAB); // BUG: shift does not work
        TrinketKeyboard.pressKey(0, 0, 0);
      } else {
        // donothing
      }

    }

  }

  loop_count ++;
  delay(1);
}
