# libBSEC
A basic integration of the Bosch SensorTec BSEC library

The Bosch SensorTec library is provided only as a closed source binary and must be downloaded by the user
Use of these files requires the header files in that download and requires linking in the Bosch binary.
These files also assume the use of the BME680 sensor, but the BSEC library is compatible with the BME280 as well.

* https://www.bosch-sensortec.com/bst/products/all_products/bsec (you must select the download for your specific system)
* BME680 driver - https://github.com/BoschSensortec/BME680_driver
* BME280 driver - https://github.com/BoschSensortec/BME280_driver
* My BME680 STM32CubeMX implementation - https://github.com/svcguy/libBME680 (includes BME680 driver as a submodule)

It's very likely the files in this repo will need to be changed to suit your application

This simple integration sets up the number of library outputs (7) and inputs (5) in low power (LP) mode for the BME680
The BSEC library requires nanosecond resolution timestamps be passed to it.  I've used microsecond timestamps multiplied by 1000 with luck.
It also has pretty strict timing requirements for calling the functions.  See the integration guide in the /doc folder of the BSEC download