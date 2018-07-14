/** 
* @file     bsec_integratoin.h 
*
* @brief    Functions to integrate the Bosch BSEC library
*           
*
*           Written to be used with the STM32CubeMX libraries for STM32 microcontrollers
*           https://www.st.com/en/development-tools/stm32cubemx.html
*           
*           Author:  Andy Josephson, 2018      
*/

#ifndef _BSEC_INTEGRATION_H
#define _BSEC_INTEGRATION_H

#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bme680.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
  * @brief Function to start up the BSEC library
  * @param A bsec_version_t pointer that will receive the library version
  * @retval 1 on success, 0 on failure
  */
int8_t bsec_begin(bsec_version_t *version);

/**
  * @brief Function to retrieve and apply sensor settings
  * @param timestamp - A timestamp provided to the BSEC library in nanosecond resolution
  * @param bsecSettings - A pointer to an object to store the settings that BSEC wants to apply to the sensor
  * @param sensor - A pointer to an object to apply the settings to the sensor
  * @retval
  */
int8_t bsec_sensor_settings( volatile int64_t timestamp, bsec_bme_settings_t *bsecSettings, struct bme680_dev *sensor );

/**
  * @brief Function to process BSEC library data
  * @param timestamp - A timestamp provided to the BSEC library in nanosecond resolution
  * @param inputs - BSEC sensor inputs
  * @param numInputs - Number of inputs
  * @param outputs - BSEC library outputs
  * @param numOutputs - Number of outputs
  * @param sensorData - Raw data from the sensor
  * @retval
  */
int8_t bsec_process_data( volatile int64_t timestamp, 
                          bsec_input_t *inputs, 
                          uint8_t numInputs,
                          bsec_output_t *outputs, 
                          uint8_t numOutputs,
                          struct bme680_field_data *sensorData);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _BSEC_INTEGRATION_H */