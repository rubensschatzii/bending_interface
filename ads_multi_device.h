/**
 * Created by cottley on 12/16/2018.
 */

#ifndef ADS_MULTI_DEVICE_H_
#define ADS_MULTI_DEVICE_H_

#include "ads_combined.h"
#include "stdint.h"

#define NUM_OF_PORTS (10)

#define PORT1_INTERRUPT (5)
#define PORT2_INTERRUPT (6)
#define PORT3_INTERRUPT (9)
#define PORT4_INTERRUPT (10)
#define PORT5_INTERRUPT (11)
#define PORT6_INTERRUPT (12)
#define PORT7_INTERRUPT (13)
#define PORT8_INTERRUPT (A0)
#define PORT9_INTERRUPT (A0)
#define PORT10_INTERRUPT (A0)

#define PORT_RESET_PIN (A2)
#define PORT_INT_PIN PORT1_INTERRUPT

typedef enum {
  PORT1_ADDRESS = 0x14,
  PORT2_ADDRESS = 0x15,
  PORT3_ADDRESS = 0x16,
  PORT4_ADDRESS = 0x17,
  PORT5_ADDRESS = 0x18,
  PORT6_ADDRESS = 0x19,
  PORT7_ADDRESS = 0x20,
  PORT8_ADDRESS,
  PORT9_ADDRESS,
  PORT10_ADDRESS
} ADS_PORT_ADDRESSES_T;

// typedef enum {
//   PORT1_ADDRESS = 0x14,
//   PORT2_ADDRESS = 0x16,
//   PORT3_ADDRESS = 0x17,
//   PORT4_ADDRESS = 0x18,
//   PORT5_ADDRESS = 0x19,
//   PORT6_ADDRESS = 0x20,
//   PORT7_ADDRESS = 0x21,
//   PORT8_ADDRESS,
//   PORT9_ADDRESS,
//   PORT10_ADDRESS
// } ADS_PORT_ADDRESSES_T;

typedef struct {
  ADS_DEV_IDS_T
  ads_dev_id;        // Device ID, ADS_ONE_AXIS, ADS_TWO_AXIS, ADS_TWO_REGION
  float ang[2];      // Most recent values from the sensor connected to port
  uint8_t addr;      // I2C address for the sensor connected to the port
  bool en;           // en = true if used, en = false if no sensor present
  uint32_t int_pin;  // Interrupt pin for port do not change
  uint8_t xfer_size; // Size of I2C transfer
} ads_port_t;

/* Initializes the ads_port_t structure with default values.
 *  ads_dev_id = ADS_ONE_AXIS
 *  _addr gets values in ADS_PORT_ADDRESSES_T enumeration
 *  en = false
 *  int_pin gets wired values, DO NOT CHANGE
 */
void ads_multi_get_defaults(ads_port_t *p);

int ads_multi_init(ads_port_t *p);

int ads_multi_run(bool run);

int ads_multi_poll(bool poll);

int ads_multi_read(void);

int ads_multi_calibrate(ADS_CALIBRATION_STEP_T ads_calibration_step,
                        uint8_t degrees);

float *ads_multi_pointer(void);

int ads_multi_set_sample_rate(ADS_SPS_T sps);

#endif /* ADS_MULTI_DEVICE_H_ */
