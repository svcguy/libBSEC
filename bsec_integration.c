/** 
* @file     bsec_integration.c 
*
* @brief    Wrappers for the Bosch BME series of sensors and the associated
*           library provided by Bosch
*           
*
*           Written to be used with the STM32CubeMX libraries for STM32 microcontrollers
*           https://www.st.com/en/development-tools/stm32cubemx.html
*           
*           Author:  Andy Josephson, 2018      
*/

#include "bsec_integration.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "bme680.h"


/**
  * @brief Function to start up the BSEC library
  * @param version - a pointer to hold the verison of the library
  * @retval 1 on success, 0 on failure
  */
int8_t bsec_begin(bsec_version_t *version)
{
  bsec_library_return_t ret;

  // Get library version
  ret = bsec_get_version( version );
  if( ret != BSEC_OK )
  { 
    return 0;
  }

  // Init
  ret = bsec_init();
  if( ret != BSEC_OK )
  {
    return 0;
  }

  // Sensor subscription
  //  This sets up the number of "virtual sensors" (outputs) in low power (LP) mode for the BME680
  bsec_sensor_configuration_t requestedVirtualSensors[7];
  uint8_t numRequestedVirtualSensors = 7;

  requestedVirtualSensors[0].sensor_id = BSEC_OUTPUT_IAQ_ESTIMATE;
  requestedVirtualSensors[0].sample_rate = BSEC_SAMPLE_RATE_LP;
  requestedVirtualSensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
  requestedVirtualSensors[1].sample_rate = BSEC_SAMPLE_RATE_LP;
  requestedVirtualSensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
  requestedVirtualSensors[2].sample_rate = BSEC_SAMPLE_RATE_LP;
  requestedVirtualSensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
  requestedVirtualSensors[3].sample_rate = BSEC_SAMPLE_RATE_LP;
  requestedVirtualSensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
  requestedVirtualSensors[4].sample_rate = BSEC_SAMPLE_RATE_LP;
  requestedVirtualSensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
  requestedVirtualSensors[5].sample_rate = BSEC_SAMPLE_RATE_LP;
  requestedVirtualSensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
  requestedVirtualSensors[6].sample_rate = BSEC_SAMPLE_RATE_LP;

  bsec_sensor_configuration_t requiredSensorSettings[BSEC_MAX_PHYSICAL_SENSOR];
  uint8_t numRequiredSensorSettings = BSEC_MAX_PHYSICAL_SENSOR;

  ret = bsec_update_subscription( requestedVirtualSensors, numRequestedVirtualSensors, 
                                  requiredSensorSettings, &numRequiredSensorSettings );

  if( ret != BSEC_OK )
  {
    return 0;
  }

  return 1;
}

/**
  * @brief Function to retrieve and apply sensor settings
  * @param timestamp - A timestamp provided to the BSEC library in nanosecond resolution
  * @param bsecSettings - A pointer to an object to store the settings that BSEC wants to apply to the sensor
  * @param sensor - A pointer to an object to apply the settings to the sensor
  * @retval
  */
int8_t bsec_sensor_settings( volatile int64_t timestamp, bsec_bme_settings_t *bsecSettings, struct bme680_dev *sensor )
{
    bsec_library_return_t ret;

    // Retrieve sensor settings 
    ret = bsec_sensor_control( timestamp, bsecSettings );
    if( ret != BSEC_OK )
    {
        return -1;
    }

    // Apply settings
    sensor->gas_sett.heatr_temp = bsecSettings->heater_temperature;
    sensor->gas_sett.heatr_dur = bsecSettings->heating_duration;
    sensor->tph_sett.os_hum = bsecSettings->humidity_oversampling;
    sensor->tph_sett.os_pres = bsecSettings->pressure_oversampling;
    sensor->gas_sett.run_gas = bsecSettings->run_gas;
    sensor->tph_sett.os_temp = bsecSettings->temperature_oversampling;
    sensor->power_mode = BME680_FORCED_MODE;
    bme680_set_sensor_settings( BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL 
        | BME680_GAS_SENSOR_SEL, sensor );
    bme680_set_sensor_mode(sensor);

    return 1;
}

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
                          struct bme680_field_data *sensorData )
{
    bsec_library_return_t ret;

    // Process Data
    inputs[0].sensor_id = BSEC_INPUT_GASRESISTOR;
    inputs[0].signal = sensorData->gas_resistance;
    inputs[0].time_stamp = timestamp;
    inputs[1].sensor_id = BSEC_INPUT_TEMPERATURE;
    inputs[1].signal = sensorData->temperature;
    inputs[1].time_stamp = timestamp;
    inputs[2].sensor_id = BSEC_INPUT_HUMIDITY;
    inputs[2].signal = sensorData->humidity;
    inputs[2].time_stamp = timestamp;
    inputs[3].sensor_id = BSEC_INPUT_PRESSURE;
    inputs[3].signal = sensorData->pressure;
    inputs[3].time_stamp = timestamp;
    inputs[4].sensor_id = BSEC_INPUT_HEATSOURCE;
    inputs[4].signal = 0;
    inputs[4].time_stamp = timestamp;
    
    ret = bsec_do_steps( inputs, numInputs, outputs, &numOutputs );
    if( ret != BSEC_OK )
    {
        return -1;
    }

    return 1;
}