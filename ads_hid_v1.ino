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

#define startbutton A1
float angle[4];
float angleoff[4];

void signal_filter(float *angle);
void deadzone_filter(float *angle);
void filterADS(void);
void error(void);

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

void setup() {
  Serial.begin(115200);
  pinMode(startbutton, INPUT_PULLUP);
  delay(100);
  // wait for the button to be pressed to start the routine
  while (1) {
    int buttonstate;
    buttonstate = digitalRead(startbutton);
    if (!buttonstate) {
      break;
    }
  }
  Serial.println("Getting Defaults...");
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
  angleoff[0] = p[0].ang[0];
  angleoff[1] = p[0].ang[1];
  angleoff[2] = p[0].ang[0];
  delay(500);
  Serial.println("Initializing Mouse");
  delay(1000);
  Mouse.begin();
}

void loop() {
  filterADS();
  somethingelse();
}
