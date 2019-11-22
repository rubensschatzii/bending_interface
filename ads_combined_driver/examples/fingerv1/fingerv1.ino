#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "ads_multi_device.h"

#include "Wire.h"
#include <Servo.h>

#define l1
#define l2
#define l3
#define l4
#define l5

void fk(void);
void mvservo(void);
int th1;
int th2;
int th3;
int th4;
int th5;

uint8_t clockFrequency = 1000000; // set I2C speed to 1 MHz

Servo tip;
Servo middle;
Servo base;

/* ads_port_t data members
 * ADS_DEV_ID_T ads_dev_id;   Device ID, ADS_ONE_AXIS, ADS_TWO_AXIS,
 * ADS_TWO_REGION float ang[2];              Most recent values from the sensor
 * connected to port uint8_t addr;              I2C address for the sensor
 * connected to the port bool  en;                  en = true if used, en =
 * false if no sensor present uint32_t int_pin;          Interrupt pin for port,
 * do not change uint8_t xfer_size;         Size of I2C transaction, do not
 * change
 */
ads_port_t p[NUM_OF_PORTS];

void fk(void) {
  // x1 = l1 * cosd(th1);
  // y1 = l1 * sind(th1);
  //
  // x2 = x1 + l2 * cosd(th1 - th2);
  // y2 = y1 + l2 * sind(th1 - th2);
  //
  // x3 = x2 + l3 * cosd(th1 - th2 - th3);
  // y3 = y2 + l3 * sind(th1 - th2 - th3);
}

void mvservo(void) {
  tip.write(int((p[1].ang[1] - p[0].ang[1]) / -1.5) + 90);
  middle.write(int((p[1].ang[1] - p[0].ang[1]) / 1.5) + 90);
  base.write(int(p[1].ang[0]));
}

void setup() {
  Serial.begin(115200);

  while (!Serial.available())
    ;
  Serial.read();
  tip.attach(A3);
  middle.attach(A4);
  base.attach(A5);

  // Wire.setClock(clockFrequency);
  // Fills the angular displacement sensor port structures with default values
  ads_multi_get_defaults(p);

  /* Update the values below to match the sensors connected
   *  If sensor is connected en = true, if no sensor en = false
   */
  p[0].en = true;
  p[0].ads_dev_id = ADS_TWO_AXIS;

  p[1].en = true;
  p[1].ads_dev_id = ADS_TWO_AXIS;

  // Initialize the sensors
  ads_multi_init(p);
  Serial.println("Initializing Multiple Sensors...");
  delay(1000);

  // Set sample rate
  ads_multi_set_sample_rate(ADS_100_HZ);
  delay(5000);

  // Start collecting data
  // ads_multi_poll(true);
  // ads_multi_run(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);

  // Poll data from all enabled sensors
  ads_multi_read(p);

  // char buffer[100];

  // Print all 20 values
  // sprintf(buffer, "%.1f,%.1f,%.1f,%.1f\r\n", p[0].ang[0], p[0].ang[1],
  //        p[1].ang[0], p[1].ang[1]);
  Serial.println("Y[1], P[1], Y[2], P[2]");
  Serial.print(p[0].ang[0]);
  Serial.print(",");
  Serial.print(p[0].ang[1]);
  Serial.print(",");
  Serial.print(p[1].ang[0]);
  Serial.print(",");
  Serial.println(p[1].ang[1]);

  mvservo();
}
