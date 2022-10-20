/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi323.h"
#include "common.h"

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    /* Variable to define limit to print temperature data. */
    uint16_t limit = 10;

    /* Structure to define accelerometer configuration. */
    struct bmi323_sens_config config = { 0 };

    /* Structure to define interrupt with its feature */
    struct bmi323_map_int map_int = { 0 };

    /* Variable to store temperature value */
    uint16_t temperature_data;

    /* Initialize the interrupt status of temperature */
    uint16_t int_status;

    /* Variable to store temperature */
    float temperature_value;

    /* Variable to store sensor time. */
    uint32_t sensor_time = 0;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI323_I2C_INTF
     * For SPI : BMI323_SPI_INTF
     */
    rslt = bmi323_interface_init(&dev, BMI323_SPI_INTF);
    bmi323_error_codes_print_result("bmi323_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323. */
        rslt = bmi323_init(&dev);
        bmi323_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            /* Get default configurations for the type of feature selected. */
            rslt = bmi323_get_sensor_config(&config, 1, &dev);
            bmi323_error_codes_print_result("bmi323_get_sensor_config", rslt);

            /* Configure the type of feature. */
            config.type = BMI323_ACCEL;

            /* Enable accel by selecting mode. */
            config.cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

            if (rslt == BMI323_OK)
            {
                rslt = bmi323_set_sensor_config(&config, 1, &dev);
                bmi323_error_codes_print_result("bmi323_set_sensor_config", rslt);

                if (rslt == BMI323_OK)
                {
                    /* Select the feature and map the interrupt to pin BMI323_INT1 or BMI323_INT2. */
                    map_int.temp_drdy_int = BMI323_INT1;

                    /* Map temperature data ready interrupt to interrupt pin. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi323_error_codes_print_result("bmi323_map_interrupt", rslt);

                    while (limit > 0)
                    {
                        /* To get the status of temperature data ready interrupt. */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi323_error_codes_print_result("bmi323_map_interrupt", rslt);

                        if (int_status & BMI323_INT_STATUS_TEMP_DRDY)
                        {
                            /* Get temperature data. */
                            rslt = bmi323_get_temperature_data(&temperature_data, &dev);
                            bmi323_error_codes_print_result("bmi323_get_sensor_data", rslt);

                            temperature_value = (float)((float)temperature_data / 512) + 23;

                            /* \370C is for printing the data in degree celsius format */
                            printf("Temperature data %f\370C\t", temperature_value);

                            rslt = bmi323_get_sensor_time(&sensor_time, &dev);
                            bmi323_error_codes_print_result("bmi323_get_sensor_data", rslt);

                            /* Print sensor time */
                            printf("Sensor time %ld\n", sensor_time);
                            limit--;
                        }
                    }
                }
            }
        }
    }

    bmi323_coines_deinit();

    return rslt;
}
