/**
 * Created by cottley on 4/13/2018.
 */

#ifndef ADS_H_
#define ADS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ads_combined_hal.h"
#include "ads_err.h"
#include "ads_util.h"

typedef void (*ads_callback)(float*);

typedef enum {
	ADS_CALIBRATE_FIRST = 0,			// One Axis calibration commands
	ADS_CALIBRATE_SECOND,
	ADS_CALIBRATE_CLEAR,
	ADS_TWO_CALIBRATE_FIRST = 0,		// Two Axis and Two Region calibratin commands
	ADS_TWO_CALIBRATE_FLAT,
	ADS_TWO_CALIBRATE_PERP,
	ADS_TWO_CALIBRATE_CLEAR,
	ADS_TWO_CALIBRATE_NONE
} ADS_CALIBRATION_STEP_T;

typedef enum {
	ADS_1_HZ   = 16384,
	ADS_10_HZ  = 1638,
	ADS_20_HZ  = 819,
	ADS_50_HZ  = 327,
	ADS_100_HZ = 163,
	ADS_200_HZ = 81,
	ADS_333_HZ = 49,
	ADS_500_HZ = 32,
} ADS_SPS_T;

typedef struct {
	ADS_SPS_T sps;
	ads_callback ads_sample_callback;
	uint32_t reset_pin;
	uint32_t datardy_pin;
	ADS_DEV_IDS_T ads_dev_id;
	uint8_t xfer_size;
	uint8_t addr;
} ads_init_t;


/**
 * @brief Reads ADS sample data when ADS is in polled mode
 *
 * @param	sample	floating point array returns new sample 
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_read_polled(float * sample);


/**
 * @brief Places ADS in free run or sleep mode
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_run(bool run);

/**
 * @brief Places ADS in poll mode. Each time sensor data is read a new sample is taken
 *
 * @param	poll	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_polled(bool poll);

/**
 * @brief Sets the sample rate of the ADS in free run mode
 *
 * @param	sps ADS_SPS_T sample rate
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_set_sample_rate(ADS_SPS_T sps);

/**
 * @brief Enables the ADS data ready interrupt line
 *
 * @param	run	true if activating ADS, false is putting in suspend mode
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_enable_interrupt(bool enable);

/**
 * @brief Updates the I2C address of the selected ADS. The default address 
 *		  is 0x12. Use this function to program an ADS to allow multiple
 *		  devices on the same I2C bus.
 *
 * @param	device	device number of the device that is being updated
 * @param	address	new address of the ADS
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_ERR_BAD_PARAM if failed
 */
int ads_update_device_address(uint8_t device, uint8_t address);

/**
 * @brief Initializes the hardware abstraction layer and sample rate of the ADS
 *
 * @param	ads_init_t	initialization structure of the ADS
 * @return	ADS_OK if successful ADS_ERR if failed
 */
int ads_init(ads_init_t * ads_init);

/**
 * @brief Calibrates one axis ADS. ADS_CALIBRATE_FIRST should be at 0 degrees on
 *				ADS_CALIBRATE_SECOND can be at 45 - 255 degrees, recommended 90 degrees.
 *
 * @param	ads_calibration_step 	ADS_CALIBRATE_STEP_T to perform
 * @param degrees uint8_t angle at which sensor is bent when performing 
 *				ADS_CALIBRATE_FIRST, and ADS_CALIBRATE_SECOND
 * @return	ADS_OK if successful ADS_ERR_IO or ADS_BAD_PARAM if failed
 */
int ads_calibrate(ADS_CALIBRATION_STEP_T ads_calibration_step, uint8_t degrees);

/**
 * @brief Shutdown ADS. Requires reset to wake up from Shutdown. ~50nA in shutdwon
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_shutdown(void);

/**
 * @brief Wakes up ADS from shutdown. Delay is necessary for ADS to reinitialize 
 *
 * @return	ADS_OK if successful ADS_ERR_IO if failed
 */
int ads_wake(void);

/**
 * @brief Checks that the device id is ADS_ONE_AXIS. ADS should not be in free run
 * 			when this function is called.
 *
 * @return	ADS_OK if dev_id is ADS_ONE_AXIS, ADS_ERR_DEV_ID if not
 */
 int ads_get_dev_id(void);
 
 int ads_get_fw_ver(uint16_t * fw_ver);
 
 void ads_set_xfer_size(uint8_t xfer_size);


#endif /* ADS_H_ */
