/*
my_nunchuk_02
wii-nunchun-mouse
Apr24, 25, 2025, ms
key names are listed here,
https://docs.arduino.cc/language-reference/en/functions/usb/Keyboard/keyboardModifiers/
*/

#include <Mouse.h>
#include <WiiChuck.h>

Accessory nunchuck;

// parameters for reading the joystick:
int range = 54;              // output range of X or Y movement  //大きくすれば早くなる
int loopDelay = 20;          // loop delay in ms
int threshold = range / 18;  // resting threshold  //rangeを割って3になるぐらいにしたらいい感じ

// my black nunchuk typical reads
// 個体差あるのでそれに合わせる
int x_min = 27;
int x_max = 222;
int x_rest = 121;
int y_min = 35;
int y_max = 231;
int y_rest = 136;


void setup() {
	Serial.begin(115200);
	nunchuck.begin();
	if (nunchuck.type == Unknown) {
		nunchuck.type = NUNCHUCK;
	}
}


void loop() {
  nunchuck.readData();    // Read inputs and update maps

  // buttons
  Serial.print("Button: ");
  bool z_pressed = false;
  if (nunchuck.getButtonZ())
  {
    Serial.print(" Z ");
    z_pressed = true;
  }

  if (nunchuck.getButtonC())
  {
    Serial.print(" C ");
    // mouse left click
    Mouse.click();
    delay(130);
  }
  Serial.println();

  // joy lever
  Serial.print("Joy: ("); 
  int xReading = nunchuck.getJoyX();
  int yReading = nunchuck.getJoyY();
  Serial.print(xReading);
  Serial.print(", "); 
  Serial.print(yReading);
  Serial.println(")");
  Serial.println();

  // mouse cursor move
  int x_dist = read_to_mapped_dist(xReading, x_rest, x_min, x_max);
  int y_dist = read_to_mapped_dist(yReading, y_rest, y_min, y_max) * (-1);
  Serial.print("mapped_distances: ("); 
  Serial.print(x_dist);
  Serial.print(", "); 
  Serial.print(y_dist);
  Serial.println(")");
  Serial.println();

  if (!z_pressed)  // z押してるときcursor動かすなら　!　をはずす
  {
    Mouse.move(x_dist, y_dist, 0);
  }
  else
  {
    if (y_dist > 0)
    {
      Mouse.move(0, 0, -1);
      delay(20);
    }
    else if (y_dist < 0)
    {
      Mouse.move(0, 0, 1);
      delay(20);
    }
  }

  delay(loopDelay);
}


int read_to_mapped_dist(int a_read, int a_rest, int a_min, int a_max) {
  // map the reading from the analog input range to the output range:
  int mapped_d = map(a_read, a_min, a_max, 0, range);
  int mapped_c = map(a_rest, a_min, a_max, 0, range);
  int distance = mapped_d - mapped_c;
  if (abs(distance) < threshold) {
    distance = 0;
  }
  // return the distance for this axis:
  return distance;
}
