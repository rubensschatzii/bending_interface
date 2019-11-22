#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

#include "Arduino.h"
#include "ads_combined.h"

#define ADS_RESET_PIN      (A2)//(6)//(4)          // Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN  (D1)//(7)//(3)         // Pin number attached to the ads data ready line. 

ads_init_t ads_init_struct;

ADS_DEV_IDS_T ads_dev_id = ADS_ONE_AXIS; 
uint8_t ads_xfer_size = 3;

void ads_data_callback(float * sample);
void deadzone_filter(float * sample);
void signal_filter(float * sample);
void parse_com_port(void);
void ads_get_device_type(void);

void signal_filter(float * sample)
{
    static float filter_samples[2][6];

    for(uint8_t i=0; i<2; i++)
    {
      filter_samples[i][5] = filter_samples[i][4];
      filter_samples[i][4] = filter_samples[i][3];
      filter_samples[i][3] = (float)sample[i];
      filter_samples[i][2] = filter_samples[i][1];
      filter_samples[i][1] = filter_samples[i][0];
  
      // 20 Hz cutoff frequency @ 100 Hz Sample Rate
      filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
        0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);   

      sample[i] = filter_samples[i][0];
    }
}

void deadzone_filter(float * sample)
{
  static float prev_sample[2];
  float dead_zone = 0.5f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}

/* Receives new samples from the ADS library */
void ads_data_callback(float * sample)
{
  // Low pass IIR filter
  signal_filter(sample);

  // Deadzone filter
  deadzone_filter(sample);

  Serial.print(sample[0]); 
  Serial.print(","); 
  Serial.println(sample[1]);
  
  // Use this line to prevent serial plotter from autoscaling and comment out above print
  //Serial.print(sample);Serial.print(","); Serial.print(-300); Serial.print(",");Serial.println(300);
}

void setup() {
  Serial.begin(115200);

  while(!Serial.available());
  Serial.read();

  // Get the type of sensor connected
  ads_get_device_type();

  Serial.println("Initializing ADS flex sensor");
  
  // Populate the ads_init structure with typical values.
  ads_init_struct.sps = ADS_100_HZ;
  ads_init_struct.ads_sample_callback = &ads_data_callback;
  ads_init_struct.reset_pin = ADS_RESET_PIN;                 // Pin connected to ADS reset line
  ads_init_struct.datardy_pin = ADS_INTERRUPT_PIN;           // Pin connected to ADS data ready interrupt
  ads_init_struct.ads_dev_id = ads_dev_id;                   // Defines the device type NEEDS TO BE UPDATED
  ads_init_struct.xfer_size = ads_xfer_size;                 // Defines the size of the standard i2c transfer
  ads_init_struct.addr = 0;	                                 // If default I2C address is used leave 0, otherwise put address

  // Initialize ADS hardware abstraction layer, and set the sample rate
  int ret_val = ads_init(&ads_init_struct);

  // Check if initialization succeeded. If initialization fails updating address does not continue
  if(ret_val == ADS_OK)
  {
    Serial.println("ADS initialization succeeded");
  }
  else
  {
    Serial.print("ADS initialization failed with reason: ");
    Serial.println(ret_val);
  }

  // Start reading data!
  ads_run(true);
}

void loop() {
  // Check for received hot keys on the com port
  if(Serial.available())
  {
    parse_com_port();
  }
}

/* Function parses received characters from the COM port for commands */
void parse_com_port(void)
{
  char key = Serial.read();

  switch(key)
  {
    case '0':
      ads_calibrate(ADS_CALIBRATE_FIRST, 0);
      break;
    case '9':
      ads_calibrate(ADS_CALIBRATE_SECOND, 90);
      break;
    case 'c':
      if(ads_dev_id == ADS_ONE_AXIS){
        ads_calibrate(ADS_CALIBRATE_CLEAR, 0);
      } else {
        ads_calibrate(ADS_TWO_CALIBRATE_CLEAR, 0);
      }
      break;
    case 'k':
      ads_calibrate((ADS_CALIBRATION_STEP_T)0x55, 0);
      break;
    case 'r':
      ads_run(true);
      break;
    case 's':
      ads_run(false);
      break;
    case 'f':
      ads_calibrate(ADS_TWO_CALIBRATE_FLAT, 90);
      break;
    case 'p':
      ads_calibrate(ADS_TWO_CALIBRATE_PERP, 90);
      break;
    case 'u':
      ads_set_sample_rate(ADS_10_HZ);
      break;
    case 'n':
      ads_set_sample_rate(ADS_100_HZ);
      break;
    default:
      break;
  }
}

// Gets the type of sensor connected over the COM port
void ads_get_device_type(void)
{
  char key = '0';
  
  while(true)
  {
    Serial.println("Select the type of sensor attached to the Bend Maker Kit...");
    Serial.println("Enter '1' for One Axis Sensor");
    Serial.println("Enter '2' for Two Axis Sensor");
    Serial.println("Enter '3' for Two Region Sensor");

    while(!Serial.available());

    key = Serial.read();

    if(key == '1')
    {
      ads_dev_id = ADS_ONE_AXIS;
      ads_xfer_size = 3;
      Serial.println(key);

      Serial.print("One Axis Sensor selected.. Is this correct [y/n]: ");

      while(!Serial.available());

      key = Serial.read();
      Serial.println(key);

      if(key == 'y' || key == 'Y')
      {
        break;
      }
    }
    else if(key == '2')
    {
      ads_dev_id = ADS_TWO_AXIS;
      ads_xfer_size = 5;
      Serial.println(key);

      Serial.print("Two Axis Sensor selected.. Is this correct [y/n]: ");

      while(!Serial.available());

      key = Serial.read();
      Serial.println(key);

      if(key == 'y' || key == 'Y')
      {
        break;
      }
    }
    else if(key == '3')
    {
      ads_dev_id = ADS_TWO_REGION;
      ads_xfer_size = 5;
      Serial.println(key);

      Serial.print("Two Region Sensor selected.. Is this correct [y/n]: ");

      while(!Serial.available());

      key = Serial.read();
      Serial.println(key);

      if(key == 'y' || key == 'Y')
      {
        break;
      }
    }
  }  
}
