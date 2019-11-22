/**
 * Created by cottley on 12/16/2018.
 */

#include "ads_multi_device.h"

#include "Arduino.h"

void pair_one_interrupt(void);
// void pair_two_interrupt(void);
// void pair_three_interrupt(void);
// void pair_four_interrupt(void);
// void pair_five_interrupt(void);

static uint8_t read_buffer[ADS_TRANSFER_SIZE];

ads_port_t *_p; // Pointer to ads_port_t structure in main

uint32_t pair_int[1];

// Stub function not used. Needed for ads_combined driver
void ads_data_callback(float *sample) {}

/* Initializes the ads_port_t structure with default values.
 *  ads_dev_id = ADS_ONE_AXIS
 *  _addr gets values in ADS_PORT_ADDRESSES_T enumeration
 *  en = false
 *  int_pin gets wired values, DO NOT CHANGE
 */
void ads_multi_get_defaults(ads_port_t *p) {
  for (uint8_t i = 0; i < NUM_OF_PORTS; i++) {
    p[i].ads_dev_id = ADS_ONE_AXIS;
    p[i].en = false;
    p[i].xfer_size = 3;
  }

  p[0].addr = (uint8_t)PORT1_ADDRESS;
  p[0].int_pin = PORT1_INTERRUPT;

  p[1].addr = (uint8_t)PORT2_ADDRESS;
  p[1].int_pin = PORT2_INTERRUPT;
}

int ads_multi_init(ads_port_t *p) {
  // Get a pointer to the ads_port_t in sketch
  _p = p;

  ads_init_t ads_init_struct;

  ads_init_struct.sps = ADS_100_HZ;
  ads_init_struct.ads_sample_callback = &ads_data_callback;
  ads_init_struct.reset_pin = PORT_RESET_PIN; // Pin connected to ADS reset line
  ads_init_struct.datardy_pin =
      _p[0].int_pin; // Pin connected to ADS data ready interrupt
  ads_init_struct.addr = 0;

  // Initialize P1 and hardware abststraction layer
  ads_hal_set_address(_p[0].addr);
  if (_p[0].ads_dev_id == ADS_ONE_AXIS) {
    ads_init_struct.ads_dev_id = ADS_ONE_AXIS;
    ads_init_struct.xfer_size = 3;
    _p[0].xfer_size = 3;
  } else if (_p[0].ads_dev_id == ADS_TWO_AXIS) {
    ads_init_struct.ads_dev_id = ADS_TWO_AXIS;
    ads_init_struct.xfer_size = 5;
    _p[0].xfer_size = 5;
  } else {
    ads_init_struct.ads_dev_id = ADS_TWO_REGION;
    ads_init_struct.xfer_size = 5;
    _p[0].xfer_size = 5;
  }
  ads_init(&ads_init_struct);

  Serial.println("Made it past the first init");

  // Find the interrupt pin for each interrupt pair
  for (uint8_t i = 0; i < NUM_OF_PORTS; i += 2) {
    if (_p[i].en) {
      pair_int[i / 2] = _p[i].int_pin;
    } else if (_p[i + 1].en) {
      pair_int[i / 2] = _p[i + 1].int_pin;
    } else {
      pair_int[i / 2] = 0;
    }
  }

  // Initialize the remaining ports
  for (uint8_t i = 0; i < NUM_OF_PORTS; i++) {
    if (_p[i].en) {
      // Initialize sensor connected to P2
      if (_p[i].ads_dev_id == ADS_ONE_AXIS) {
        _p[i].xfer_size = 3;
      } else {
        _p[i].xfer_size = 5;
      }
      ads_hal_set_address(_p[i].addr);
      ads_set_xfer_size(_p[i].xfer_size);
      ads_hal_set_xfer_size(_p[i].xfer_size);
      ads_set_sample_rate(ADS_100_HZ);
    }
  }

  ads_hal_pin_int_enable(false); // Disable the pin interrupt in the HAL
}

int ads_multi_set_sample_rate(ADS_SPS_T sps) {
  // Send the command to all enabled ports
  for (uint8_t i = 0; i < NUM_OF_PORTS; i++) {
    if (_p[i].en) {
      ads_hal_set_address(_p[i].addr);
      ads_set_xfer_size(_p[i].xfer_size);
      ads_hal_set_xfer_size(_p[i].xfer_size);
      if (ads_set_sample_rate(sps) != ADS_OK) {
        Serial.print("Sensor failed to acknowledge sps command: ");
        Serial.println(i);
      }
    }
  }
  return ADS_OK;
}

int ads_multi_poll(bool poll) {
  // Send the command to all enabled ports
  for (uint8_t i = 0; i < NUM_OF_PORTS; i++) {
    if (_p[i].en) {
      ads_hal_set_address(_p[i].addr);
      ads_set_xfer_size(_p[i].xfer_size);
      ads_hal_set_xfer_size(_p[i].xfer_size);
      if (ads_polled(poll) != ADS_OK) {
        Serial.print("Sensor failed to acknowledge run command: ");
        Serial.println(i);
      } else if (ads_polled(poll) == ADS_OK) {
        Serial.print("ADS is okay it seems.");
      }
    }
  }

  return ADS_OK;
}

int ads_multi_run(bool run) {
  noInterrupts();
  // Send the command to all enabled ports
  for (uint8_t i = 0; i < NUM_OF_PORTS; i += 2) {
    // delay(1);
    if (_p[i].en) {
      ads_hal_set_address(_p[i].addr);
      ads_set_xfer_size(_p[i].xfer_size);
      ads_hal_set_xfer_size(_p[i].xfer_size);
      if (ads_run(run) != ADS_OK) {
        Serial.print("Sensor failed to acknowledge run command: ");
        Serial.println(i);
      }
    }
    if (_p[i + 1].en) {
      ads_hal_set_address(_p[i + 1].addr);
      ads_set_xfer_size(_p[i + 1].xfer_size);
      ads_hal_set_xfer_size(_p[i + 1].xfer_size);
      if (ads_run(run) != ADS_OK) {
        Serial.print("Sensor failed to acknowledge run command: ");
        Serial.println(i);
      }
    }
  }
  interrupts();

  pinMode(_p[0].int_pin, INPUT_PULLUP);
  pinMode(_p[1].int_pin, INPUT_PULLUP);

  // Create pin change interrupts that will be used for reading data from the
  // sensors
  if (pair_int[0]) {
    attachInterrupt(digitalPinToInterrupt(pair_int[0]), pair_one_interrupt,
                    FALLING);
  }
  return ADS_OK;
}

int ads_multi_calibrate(ADS_CALIBRATION_STEP_T ads_calibration_step,
                        uint8_t degrees) {}

int ads_multi_read(ads_port_t *p) {
  int16_t temp;

  for (uint8_t i = 0; i < NUM_OF_PORTS; i++) {
    if (_p[i].en) {
      ads_hal_set_address(_p[i].addr);
      if (_p[i].ads_dev_id == ADS_ONE_AXIS) {
        if (ads_hal_read_buffer(read_buffer, 3) == ADS_OK) {
          temp = ads_int16_decode(&read_buffer[1]);
          _p[i].ang[0] = (float)temp / 64.0f;
          _p[i].ang[1] = 0.0f;
        }
      } else {
        if (ads_hal_read_buffer(read_buffer, 5) == ADS_OK) {
          temp = ads_int16_decode(&read_buffer[1]);
          _p[i].ang[0] = (float)temp / 32.0f;
          p[i].ang[0] = _p[i].ang[0];
          // Serial.print(_p[i].ang[0]);
          // Serial.print(",");

          temp = ads_int16_decode(&read_buffer[3]);
          _p[i].ang[1] = (float)temp / 32.0f;
          p[i].ang[1] = _p[i].ang[1];
          // Serial.println(_p[i].ang[1]);
        }
      }
    }
  }
  return ADS_OK;
}

// Interrupt for Port1 and Port2
void pair_one_interrupt(void) {
  int16_t temp;

  if (_p[0].en) {
    ads_hal_set_address(_p[0].addr);
    if (_p[0].ads_dev_id == ADS_ONE_AXIS) {
      if (ads_hal_read_buffer(read_buffer, 3) == ADS_OK) {
        temp = ads_int16_decode(&read_buffer[1]);
        _p[0].ang[0] = (float)temp / 64.0f;
        _p[0].ang[1] = 0.0f;
      }
    } else {
      if (ads_hal_read_buffer(read_buffer, 5) == ADS_OK) {
        temp = ads_int16_decode(&read_buffer[1]);
        _p[0].ang[0] = (float)temp / 32.0f;

        temp = ads_int16_decode(&read_buffer[3]);
        _p[0].ang[1] = (float)temp / 32.0f;
      }
    }
  }

  if (_p[1].en) {
    ads_hal_set_address(_p[1].addr);
    if (_p[1].ads_dev_id == ADS_ONE_AXIS) {
      if (ads_hal_read_buffer(read_buffer, 3) == ADS_OK) {
        temp = ads_int16_decode(&read_buffer[1]);
        _p[1].ang[0] = (float)temp / 64.0f;
        _p[1].ang[1] = 0.0f;
      }
    } else {
      if (ads_hal_read_buffer(read_buffer, 5) == ADS_OK) {
        temp = ads_int16_decode(&read_buffer[1]);
        _p[1].ang[0] = (float)temp / 32.0f;

        temp = ads_int16_decode(&read_buffer[3]);
        _p[1].ang[1] = (float)temp / 32.0f;
      }
    }
  }
}

// Interrupt for Port3 and Port4

// Interrupt for Port5 and Port6

// Interrupt for Port7 and Port8

// Interrupt for Port9 and Port10s
