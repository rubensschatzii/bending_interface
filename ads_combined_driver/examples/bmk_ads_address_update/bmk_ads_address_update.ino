#include "ads_combined.h"

#define ADS_RESET_PIN (A2)
#define ADS_INTERRUPT_PIN                                                      \
  (7) // Interrupt line purposefully not set to a correct value

#define PORT1_INTERRUPT (5)
#define PORT2_INTERRUPT (6)
#define PORT3_INTERRUPT (A0)
#define PORT4_INTERRUPT (0)
#define PORT5_INTERRUPT (A1)
#define PORT6_INTERRUPT (1)
#define PORT7_INTERRUPT (A3)
#define PORT8_INTERRUPT (2)
#define PORT9_INTERRUPT (A4)
#define PORT10_INTERRUPT (3)

/* This Sketch initializes the ads sensor that currently has the default
 * I2C address and updates the I2C address based on which PORT the sensor is
 * connected to.
 *
 * Attach one sensor to the break out board at a time and then apply
 * power to the board.
 *
 * P1 --> 0x14
 * P2 --> 0x15
 * P3 --> 0x16
 * P4 --> 0x17
 * P5 --> 0x18
 * P6 --> 0x19
 * P7 --> 0x20
 * P8 --> 0x21
 * P9 --> 0x22
 * P10--> 0x23
 *
 */

ads_init_t ads_init_struct;

uint8_t new_address = 0x14;

// This value is updated via the ads_get_device_type function
ADS_DEV_IDS_T ads_dev_id = ADS_ONE_AXIS;
uint8_t ads_xfer_size = 3;

// Gets the type of sensor connected over the COM port
void ads_get_device_type(void) {
  char key = '0';

  while (true) {
    Serial.println(
        "Select the type of sensor attached to the Bend Maker Kit...");
    Serial.println("Enter '1' for One Axis Sensor");
    Serial.println("Enter '2' for Two Axis Sensor");
    Serial.println("Enter '3' for Two Region Sensor");

    while (!Serial.available())
      ;

    key = Serial.read();

    if (key == '1') {
      ads_dev_id = ADS_ONE_AXIS;
      ads_xfer_size = 3;
      Serial.println(key);

      Serial.print("One Axis Sensor selected.. Is this correct [y/n]: ");

      while (!Serial.available())
        ;

      key = Serial.read();
      Serial.println(key);

      if (key == 'y' || key == 'Y') {
        break;
      }
    } else if (key == '2') {
      ads_dev_id = ADS_TWO_AXIS;
      ads_xfer_size = 5;
      Serial.println(key);

      Serial.print("Two Axis Sensor selected.. Is this correct [y/n]: ");

      while (!Serial.available())
        ;

      key = Serial.read();
      Serial.println(key);

      if (key == 'y' || key == 'Y') {
        break;
      }
    } else if (key == '3') {
      ads_dev_id = ADS_TWO_REGION;
      ads_xfer_size = 5;
      Serial.println(key);

      Serial.print("Two Region Sensor selected.. Is this correct [y/n]: ");

      while (!Serial.available())
        ;

      key = Serial.read();
      Serial.println(key);

      if (key == 'y' || key == 'Y') {
        break;
      }
    }
  }
}

/* Stub function here for initialization code to run */
void ads_data_callback(float *sample) {}

void setup() {
  delay(3000);
  Serial.begin(115200);

  Serial.println(" ");
  Serial.println("Press any key to auto update the I2C address of the "
                 "connected Soft Flex Sensor");

  // Wait for a keypress
  while (!Serial.available())
    ;

  Serial.read();

  // Get sensor type connected
  ads_get_device_type();

  // Configuring all interrupt lines as digital inputs
  pinMode(PORT1_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT2_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT3_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT4_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT5_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT6_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT7_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT8_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT9_INTERRUPT, INPUT_PULLUP);
  pinMode(PORT10_INTERRUPT, INPUT_PULLUP);

  // Populate the ads_init structure with typical values.
  ads_init_struct.sps = ADS_100_HZ;
  ads_init_struct.ads_sample_callback = &ads_data_callback;
  ads_init_struct.reset_pin = ADS_RESET_PIN; // Pin connected to ADS reset line
  ads_init_struct.datardy_pin =
      ADS_INTERRUPT_PIN; // Pin connected to ADS data ready interrupt
  ads_init_struct.ads_dev_id =
      ads_dev_id; // Defines the device type NEEDS TO BE UPDATED
  ads_init_struct.xfer_size =
      ads_xfer_size; // Defines the size of the standard i2c transfer
  ads_init_struct.addr =
      0; // Update value if non-default i2c address currently on sensor

  // Initialize ADS hardware abstraction layer, and set the sample rate
  int ret_val = ads_init(&ads_init_struct);

  // Check if initialization succeeded. If initialization fails updating address
  // does not continue
  if (ret_val == ADS_OK) {
    Serial.println("ADS initialization succeeded");
  } else {
    Serial.print("ADS initialization failed with reason: ");
    Serial.println(ret_val);
    Serial.println("I2C Address not updated due to initialization failure "
                   "press reset to reattempt");

    while (1)
      ; // Initialization failed so wait here
  }

  Serial.println("Updating I2C address...\n");

  // Enable
  ads_run(true);

  // Wait for an interrupt line to go low to detect which port the sensor is
  // attached to
  while (true) {
    if (digitalRead(PORT1_INTERRUPT) == LOW) {
      new_address = 0x13;
      break;
    } else if (digitalRead(PORT2_INTERRUPT) == LOW) {
      new_address = 0x15;
      break;
    } else if (digitalRead(PORT3_INTERRUPT) == LOW) {
      new_address = 0x16;
      break;
    } else if (digitalRead(PORT4_INTERRUPT) == LOW) {
      new_address = 0x17;
      break;
    } else if (digitalRead(PORT5_INTERRUPT) == LOW) {
      new_address = 0x18;
      break;
    } else if (digitalRead(PORT6_INTERRUPT) == LOW) {
      new_address = 0x19;
      break;
    } else if (digitalRead(PORT7_INTERRUPT) == LOW) {
      new_address = 0x20;
      break;
    } else if (digitalRead(PORT8_INTERRUPT) == LOW) {
      new_address = 0x21;
      break;
    } else if (digitalRead(PORT9_INTERRUPT) == LOW) {
      new_address = 0x22;
      break;
    } else if (digitalRead(PORT10_INTERRUPT) == LOW) {
      new_address = 0x23;
      break;
    }
  }

  // Take the sensor out of free run mode - stop sampling
  ads_run(false);

  // Delay here to allow previous command to be executed
  delay(10);

  // Update the connected soft flex sensor with the new I2C address
  ret_val = ads_update_device_address(0, new_address);

  if (ret_val == ADS_OK) {
    Serial.println("I2C Device Address updated successfully");
    Serial.print("New device address: 0x");
    Serial.println(new_address, HEX);
  } else {
    Serial.print("Updating I2C Device Address failed with reason: ");
    Serial.println(ret_val);
  }
}

void loop() {
  // No code here. Everything is done in the setup function
}
