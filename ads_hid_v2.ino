/*
Created by Schatz by using libraries provided by BendLab
email: rubensschatzii(at)yahoo.com.br
*/
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "ads_multi_device.h"

#include "Wire.h"
#include <Mouse.h>

#define touchbutton A1
//#define startbutton A1
#define FrameID 0x02

bool FirstTouch = true;
bool SecondTouch = false;
bool ButtonisPressed = false;

int range = 1;
int axis;
uint8_t currentx;
uint8_t currenty;
uint8_t X1;
uint8_t Y1;
uint8_t X2;
uint8_t Y2;
uint8_t previousx;
uint8_t previousy;
uint8_t checksum;
uint8_t sendthisshitfornow = 0;
const uint8_t bytestobechecked = 5;
uint8_t tocheck[5];
float angle[4];
float angleoff[4];

void populateLPC(void);
void signal_filter(float *angle);
void deadzone_filter(float *angle);
void filterADS(void);
void error(void);
void senddata(void);
void senddatadebug(void);
void receivedata(void);
uint8_t LPC(uint8_t tocheck[bytestobechecked]);
int movecursor(int axis, float angle);
uint8_t writebooleantobyte(bool *FirstTouch, bool *SecondTouch,
                           bool *ButtonisPressed);
void checktouch(void);
void calculatecoordinates(void);

ads_port_t p[NUM_OF_PORTS];

void signal_filter(float *angle) {
  static float filter_samples[4][6];

  for (uint8_t i = 0; i < 4; i++) {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)angle[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[i][0] = filter_samples[i][1] * (0.36952737735124147f) -
                           0.19581571265583314f * filter_samples[i][2] +
                           0.20657208382614792f * (filter_samples[i][3] +
                                                   2 * filter_samples[i][4] +
                                                   filter_samples[i][5]);

    angle[i] = filter_samples[i][0];
  }
}
void deadzone_filter(float *angle) {
  static float prev_sample[4];
  float dead_zone = 0.5f;

  for (uint8_t i = 0; i < 4; i++) {
    if (fabs(angle[i] - prev_sample[i]) > dead_zone)
      prev_sample[i] = angle[i];
    else
      angle[i] = prev_sample[i];
  }
}
void error(uint8_t errno) {
  while (1) {
    uint8_t i;
    for (i = 0; i < errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i = errno; i < 10; i++) {
      delay(200);
    }
  }
}

void filterADS(void) {
  angle[0] = p[0].ang[0] - angleoff[0];
  angle[1] = p[0].ang[1] - angleoff[1];
  angle[2] = p[1].ang[0] - angleoff[2];
  // angle[3] = p[1].ang[1];
  signal_filter(angle);
  deadzone_filter(angle);
}

int movecursor(int axis, float angle) {
  int output;
  float temp;
  temp = angle;
  if (axis == 0) {
    // output = map(temp, -60, -range, 60, range);
    if (temp > 10) {
      output = 1;
    } else if (temp < -10) {
      output = -1;
    } else {
      output = 0;
    }
  } else {
    // output = map(temp, -90, -range, 90, range);
    if (temp > 20) {
      output = -1;
    } else if (temp < -20) {
      output = 1;
    } else {
      output = 0;
    }
  }
  return output;
}
void calculatecoordinates(void) {
  int movex;
  int movey;
  movex = movecursor(0, angle[0]);
  movey = movecursor(1, angle[1]);
  if (((previousx == 0) && (movex < 0)) ||
      ((previousx == 255) && (movex > 0))) {
    // currentx = previousx;
    movex = 0;
  }
  // else {
  //   currentx = previousx + movex;
  // }
  if (((previousy == 0) && (movey < 0)) ||
      ((previousy == 255) && (movey > 0))) {
    // currenty = previousy;
    movey = 0;
  }
  //  else {
  //   currenty = previousy + movey;
  // }
  currentx = movex + previousx;
  currenty = movey + previousy;
}

void checktouch(void) {
  int buttonstate;
  buttonstate = digitalRead(touchbutton);
  if (!buttonstate) {
    ButtonisPressed = true;
  }
}

uint8_t writebooleantobyte(bool FirstTouch, bool SecondTouch,
                           bool ButtonisPressed) {
  uint8_t output = 0x00;
  if (FirstTouch == true) {
    output |= 1UL << 0;
  }
  if (SecondTouch == true) {
    output |= 1UL << 1;
  }
  if (ButtonisPressed == true) {
    output |= 1UL << 2;
  }
  return output;
}

void populateLPC(void) {
  tocheck[0] = sendthisshitfornow;
  tocheck[1] = X1;
  tocheck[2] = Y1;
  tocheck[3] = X2;
  tocheck[4] = Y2;
}

uint8_t summing(uint8_t *tocheck) {
  uint8_t sum;
  if (sendthisshitfornow + X1 + X2 + Y1 + Y2 > 255) {
    sum = 255;
  } else {
    sum = sendthisshitfornow + X1 + X2 + Y1 + Y2;
  }
  return sum;
}

uint8_t LPC(uint8_t *tocheck) {
  uint8_t checked = 0x00;
  for (uint8_t i = 0; i < 5; i++) {
    checked = checked ^ tocheck[i];
  }
  return checked;
}

void receivedata(void) {
  while (Serial.available()) {
    Serial.read();
  }
  // uint8_t receivebytes[3];
  // if (Serial.available() > 0) {
  //   for (uint8_t i = 0; i < 3; i++) {
  //     receivebytes[i] = Serial.read();
  //  }
  //}
}

void senddata(void) {
  Serial.write(FrameID);
  Serial.write(sendthisshitfornow);
  Serial.write(X1);
  Serial.write(Y1);
  Serial.write(X2);
  Serial.write(Y2);
  Serial.write(checksum);
  ButtonisPressed = false;
}

void senddatadebug(void) {
  Serial.print(FrameID, HEX);
  Serial.print(",");
  // Serial.write(FirstTouch);
  // Serial.write(SecondTouch);
  // Serial.write(ButtonisPressed);
  Serial.print(sendthisshitfornow, BIN);
  Serial.print(",");
  Serial.print(X1);
  Serial.print(",");
  Serial.print(Y1);
  Serial.print(",");
  Serial.print(X2);
  Serial.print(",");
  Serial.print(Y2);
  Serial.print(",");
  Serial.println(checksum);
  ButtonisPressed = false;
}

void setup() {
  Serial.begin(38400);
  // Serial.setTimeout(500);
  // pinMode(startbutton, INPUT_PULLUP);
  pinMode(touchbutton, INPUT_PULLUP);
  delay(100);

  // wait for the button to be pressed to start the routine
  // while (1) {
  //   int buttonstate;
  //   buttonstate = digitalRead(startbutton);
  //   if (!buttonstate) {
  //     break;
  //   }
  // }
  // while (!Serial.available())
  //   ;
  delay(100);
  // Fills the angular displacement sensor port structures with default values
  ads_multi_get_defaults(p);

  /* Update the values below to match the sensors connected
   *  If sensor is connected en = true, if no sensor en = false
   */
  p[0].en = true;
  p[0].ads_dev_id = ADS_TWO_AXIS;

  p[1].en = false;
  p[1].ads_dev_id = ADS_ONE_AXIS;

  p[2].en = false;
  p[2].ads_dev_id = ADS_TWO_AXIS;

  p[3].en = false;
  p[3].ads_dev_id = ADS_TWO_REGION;

  p[4].en = false;
  p[4].ads_dev_id = ADS_TWO_REGION;

  p[5].en = false;
  p[5].ads_dev_id = ADS_TWO_AXIS;

  p[6].en = false;
  p[6].ads_dev_id = ADS_ONE_AXIS;

  p[7].en = false;
  p[7].ads_dev_id = ADS_TWO_REGION;

  p[8].en = false;
  p[8].ads_dev_id = ADS_ONE_AXIS;

  p[9].en = false;
  p[9].ads_dev_id = ADS_ONE_AXIS;

  // Initialize the sensors
  ads_multi_init(p);
  delay(1000);

  // Set sample rate
  int check = ads_multi_set_sample_rate(ADS_100_HZ);
  if (check != ADS_OK) {
    error(check);
  }

  // Start collecting data
  check = ads_multi_run(true);
  if (check != ADS_OK) {
    error(check);
  }
  delay(1000);
  angleoff[0] = p[0].ang[0];
  angleoff[1] = p[0].ang[1];
  angleoff[2] = p[0].ang[0];
  delay(500);
  delay(1000);
  currentx = 0;
  currenty = 0;
}

void loop() {
  previousx = currentx;
  previousy = currenty;
  filterADS();
  calculatecoordinates();
  checktouch();
  sendthisshitfornow =
      writebooleantobyte(FirstTouch, SecondTouch, ButtonisPressed);
  // Serial.print(angle[0]);
  // Serial.print(",");
  // Serial.println(angle[1]);
  X1 = currentx;
  Y1 = currenty;
  populateLPC();
  // checksum = LPC(tocheck);
  checksum = summing(tocheck);
  receivedata();
  // senddata();
  senddatadebug();
  delay(10);
}
