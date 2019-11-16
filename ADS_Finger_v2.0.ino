/*
Created by Schatz by using libraries provided by BendLab
email: rubensschatzii(at)yahoo.com.br
*/
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "ads_multi_device.h"

#include "Wire.h"
#include <SD.h>
#include <SPI.h>
#include <Servo.h>
#include <math.h>

/*
Version 2.0 - Added support to more sensors and one axis ADS
New port interrupt enumeration:
Port 1 - Pin (5)
Port 2 - Pin (6)
Port 3 - Pin (9)
Port 4 - Pin (10)
Port 5 - Pin (11)
Port 6 - Pin (12)
Port 7 - Pin (13)

*/

// Version 1.2.2.1 - slight modification to always samples data on a stable
// frame rate when not logging on SD

// Version 1.2.2 - Fixed Forward Kinematics - there was an error on the
// calculation for the x coordinate of the Tip of the Finger, which now became
// the variable xp;

/* ads_port_t data members
 * ADS_DEV_ID_T ads_dev_id;   Device ID, ADS_ONE_AXIS, ADS_TWO_AXIS,
 * ADS_TWO_REGION float ang[2];              Most recent values from the sensor
 * connected to port uint8_t addr;              I2C address for the sensor
 * connected to the port bool  en;                  en = true if used, en =
 * false if no sensor present uint32_t int_pin;          Interrupt pin for port,
 * do not change uint8_t xfer_size;         Size of I2C transaction, do not
 * change
 */
#define looptime 20
#define startbutton A1
#define sdSelect 4
#define PI 3.14159265
#define l1 44  // Lenght of Proximal Phalanx (PIP) (Base of the finger)
#define l2 22  // Lenght of the Metacarpo Phalanx (MCP) (Middle of the Finger)
#define l3 24  // Lenght of the Distal Phalanx (DIP) (Tip of the Finger)
#define l4 55  // Lenght of the first link of the Robotic Finger (Middle)
#define l5 42  // Lenght of the second link of the Robotic Finger (Tip)
#define ofst 0 // Offset in Y plane between RF origin and Finger origin
#define loff 25
#define DPconst                                                                \
  0.99 // Constant between DIP and PIP joints. Literature points to values
       // between 0.6 and 0.88. Try and adjust accordingly

uint8_t dataOnOff = 0;
// uint8_t dataOnOff = 1; // uncomment to log to the SD card

void signal_filter(float *sample);
void deadzone_filter(float *sample);
void filterADS(void);
void ForwardKinematics(void);
void InverseKinematics(void);
void mvservo(void);
void getOffset(void);
void logData(void);
void error(void);
float offsetmean(float angleoffset);

const uint8_t ofstbutton = 13;
float angle[4];
float th1;
float th2;
float th3;
float th4;
float th5;
float th1off;
float th2off;
float th3off;
float thbaseoff;
float thb;
float L;
float xp;
float x;
float y;
float z;
float xpoff;
float yoff;
float xoff;
float zoff;
float thbdiff;
float thbdiffoff;
int waittime;

char filename[15];

long starttime;
long runtime = 0;
long previousruntime = 0;

File logfile;

Servo tip;
Servo middle;
Servo base;

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
  angle[0] = p[0].ang[0];
  angle[1] = p[0].ang[1];
  angle[2] = p[1].ang[0];
  angle[3] = p[1].ang[1];
  signal_filter(angle);
  deadzone_filter(angle);
}
// Function to get offset to adjust the initial measured angles to zero.
float offsetmean(float angleoffset) {
  float ofsts[5];
  int i;
  for (i = 0; i < 5; i++) {
    ofsts[i] = angleoffset;
    delay(20);
  }
  float mean = (ofsts[0] + ofsts[1] + ofsts[2] + ofsts[3] + ofsts[4]) / 5;
  return mean;
}
void getOffset(void) {
  float temp1;
  float temp2;

  temp1 = offsetmean(p[1].ang[1]);
  temp2 = offsetmean(p[0].ang[1]);
  th3off = temp2;
  th1off = temp1;

  thbaseoff = offsetmean(p[1].ang[0]);
  thbdiffoff = offsetmean(p[0].ang[0]);
}

void ForwardKinematics(void) {
  th1 = angle[3] - th1off;
  th3 = ((angle[1] - th3off) - (angle[3] - th1off)) / (1 + (1 / DPconst));
  th2 = (th3 / DPconst);
  thb = angle[2] - thbaseoff;
  thbdiff = angle[0] - thbdiffoff;
  // Foward Kinematics
  xp = l3 * cos((th1 - th2 - th3) * PI / 180) +
       l2 * cos((th1 - th2) * PI / 180) + l1 * cos((th1)*PI / 180);
  y = l3 * sin((th1 - th2 - th3) * PI / 180) +
      l2 * sin((th1 - th2) * PI / 180) + l1 * sin((th1)*PI / 180);
  L = sqrt(pow(y - ofst, 2) + pow(xp, 2));
  x = xp * cos(thb * PI / 180);
  z = xp * sin(thb * PI / 180);

  xpoff = loff * cos((90 + th1 - th2 - th3) * PI / 180) + xp;
  yoff = loff * sin((90 + th1 - th2 - th3) * PI / 180) + y;
  xoff = xpoff * cos(thb * PI / 180);
  zoff = xpoff * sin(thb * PI / 180);
}

// Function to calculate the Forward Kinematics to find the position of the
// tip of the finger and find the inverse kinematics to the servo angles th4
// and th5
void InverseKinematics(void) {

  // Inverse Kinematics
  th5 = PI - acos((pow(l4, 2) + pow(l5, 2) - pow(L, 2)) / (2 * l4 * l5));
  if (y < ofst) {
    th4 = acos((pow(L, 2) + pow(l4, 2) - pow(l5, 2)) / (2 * L * l4)) -
          acos(x / L);
  } else {
    th4 = acos((pow(L, 2) + pow(l4, 2) - pow(l5, 2)) / (2 * L * l4)) +
          acos(x / L);
  }
}

// Function to move servo to latest measured angles by the ADS
void mvservo(void) {
  int t;
  int m;
  int b;
  int ti;
  int mi;
  int ba;
  t = int(((th5)*180 / PI) + 90);
  m = int(((th4)*180 / PI) + 90);
  // b = int((p[0].ang[0] - thbaseoff) + 90);
  b = int((angle[0] - thbaseoff) + 90);
  ti = constrain(t, 0, 180);
  mi = constrain(m, 0, 180);
  ba = constrain(b, 0, 180);
  // These are the values being sent to the servo Ti - Tip; Mi- Middle; Ba -
  // Base
  // Serial.print(ti);
  // Serial.print(",");
  // Serial.print(mi);
  // Serial.print(",");
  // Serial.println(ba);
  ti = map(ti, 0, 180, 180, 0);
  // mi = map(mi, 0, 180, 180, 0);
  tip.write(ti);
  middle.write(mi);
  base.write(ba);
}
void setup() {
  Serial.begin(115200);

  // while (!Serial.available())
  //   ;
  // Serial.read();
  pinMode(startbutton, INPUT_PULLUP);
  delay(100);
  while (1) {
    int buttonstate;
    buttonstate = digitalRead(startbutton);
    if (!buttonstate) {
      break;
    }
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);

  tip.attach(A3);
  middle.attach(A4);
  base.attach(A5);

  if (dataOnOff == 1) {
    Serial.println("Initializing the SD card for Datalogging");
    delay(100);
    if (!SD.begin(sdSelect)) {
      Serial.println("Failed to start SD Card.");
      error(2);
      delay(200);
    } else {
      Serial.println("SD initialization successful.");
    }

    // Create datalogging file with different name not to overwrite last used
    // file
    strcpy(filename, "/ADSLOG00.TXT");
    for (uint8_t i = 0; i < 100; i++) {
      filename[7] = '0' + i / 10;
      filename[8] = '0' + i % 10;
      // create if does not exist, do not open existing, write, sync after write
      if (!SD.exists(filename)) {
        break;
      }
    }
    logfile = SD.open(filename, FILE_WRITE);
    if (!logfile) {
      Serial.print("Couldnt create ");
      Serial.println(filename);
      error(2);
      delay(200);
    } else {
      pinMode(8, OUTPUT);
      pinMode(8, HIGH);
      Serial.print("Writing to file ");
      Serial.println(filename);
      logfile.println("Time, x, y, z");
      logfile.close();
      pinMode(8, LOW);
    }
  }

  // pinMode(ofstbutton, INPUT);
  // Fills the angular displacement sensor port structures with default values
  ads_multi_get_defaults(p);

  /* Update the values below to match the sensors connected
   *  If sensor is connected en = true, if no sensor en = false
   */
  p[0].en = true;
  p[0].ads_dev_id = ADS_TWO_AXIS;

  p[1].en = true;
  p[1].ads_dev_id = ADS_TWO_AXIS;

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
  Serial.println("Initializing Multiple Sensors...");
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
  Serial.println("Getting offset from finger position.");
  delay(500);
  getOffset();
  Serial.print(th1off);
  Serial.print(",");
  Serial.print(th2off);
  Serial.print(",");
  Serial.print(th3off);
  Serial.print(",");
  Serial.println(thbaseoff);
  starttime = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10);
  previousruntime = runtime;
  filterADS();
  ForwardKinematics();
  runtime = (millis() - starttime); /// 1000;
  mvservo();
  // Print the latest ADS sensor data
  // Uncomment the Next lines to print on the serial filtered samples
  // Serial.print(angle[0]);
  // Serial.print(",");
  // Serial.print(angle[1]);
  // Serial.print(",");
  // Serial.print(angle[2]);
  // Serial.print(",");
  // Serial.println(angle[3]);

  // Uncomment the next lines to print on the serial raw samples
  // Serial.print(p[0].ang[0]);
  // Serial.print(",");
  // Serial.print(p[0].ang[1]);
  // Serial.print(",");
  // Serial.print(p[1].ang[0]);
  // Serial.print(",");
  // Serial.println(p[1].ang[1]);

  // Uncomment the Next lines to print the calculated Forward Kinematics
  Serial.print(",");
  Serial.print(runtime);
  Serial.print(",");
  // // Serial.print(L);
  // // Serial.print(",");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(z);
  Serial.print(",");
  // Serial.print(xoff);
  // Serial.print(",");
  // Serial.print(yoff);
  // Serial.print(",");
  // Serial.print(zoff);
  // Serial.print(",");
  Serial.print(th1);
  Serial.print(",");
  Serial.print(th2);
  Serial.print(",");
  Serial.print(th3);
  Serial.print(",");
  Serial.print(thb);
  Serial.print(",");
  Serial.print(thbdiff);
  Serial.println(",");

  ////////********Output of the Angles themselves instead of calculated Angles
  // Serial.print(angle[0]);
  // Serial.print(", ");
  // Serial.print(angle[1]);
  // Serial.print(", ");
  // Serial.print(angle[2]);
  // Serial.print(", ");
  // Serial.print(angle[3]);
  // Serial.println(" ");
}
