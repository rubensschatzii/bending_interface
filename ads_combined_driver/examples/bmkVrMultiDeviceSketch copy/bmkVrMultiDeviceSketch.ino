#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
#define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "ads_multi_device.h"

#include "Wire.h"

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

void setup() {
  Serial.begin(115200);

  while (!Serial.available())
    ;
  Serial.read();

  // Fills the angular displacement sensor port structures with default values
  ads_multi_get_defaults(p);

  /* Update the values below to match the sensors connected
   *  If sensor is connected en = true, if no sensor en = false
   */
  p[0].en = true;
  p[0].ads_dev_id = ADS_TWO_AXIS;

  p[1].en = true;
  p[1].ads_dev_id = ADS_TWO_AXIS;

  p[2].en = true;
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
  ads_multi_set_sample_rate(ADS_100_HZ);

  // Start collecting data
  ads_multi_run(true);
  // ads_multi_run(true);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(8);

  // Poll data from all enabled sensors
  Serial.print(p[0].ang[0]);
  Serial.print(",");
  Serial.print(p[0].ang[1]);
  Serial.print(",");
  Serial.print(p[1].ang[0]);
  Serial.print(",");
  Serial.print(p[1].ang[1]);
  Serial.print(",");
  Serial.print(p[2].ang[0]);
  Serial.print(",");
  Serial.println(p[2].ang[1]);
}
