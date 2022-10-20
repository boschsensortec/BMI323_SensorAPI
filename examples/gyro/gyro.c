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
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for gyro.
 *
 *  @param[in] dev       : Structure instance of bmi323_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_gyro_config(struct bmi323_dev *dev);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print gyro data. */
    uint8_t limit = 100;

    float x = 0, y = 0, z = 0;

    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    /* Create an instance of sensor data structure. */
    struct bmi323_sensor_data sensor_data = { 0 };

    /* Initialize the interrupt status of gyro. */
    uint16_t int_status = 0;

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
            /* Gyro configuration settings. */
            rslt = set_gyro_config(&dev);
            bmi323_error_codes_print_result("Set gyro config", rslt);

            /* Select gyro sensor. */
            sensor_data.type = BMI323_GYRO;

            printf("Gyro data in LSB units and degrees per second\n");

            /* Loop to print gyro data when interrupt occurs. */
            while (limit > 0)
            {
                /* To get the data ready interrupt status of gyro. */
                rslt = bmi323_get_int1_status(&int_status, &dev);
                bmi323_error_codes_print_result("Get interrupt status", rslt);

                /* To check the data ready interrupt status and print the status for 10 samples. */
                if (int_status & BMI323_INT_STATUS_GYR_DRDY)
                {
                    /* Get gyro data for x, y and z axis. */
                    rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev);
                    bmi323_error_codes_print_result("Get sensor data", rslt);

                    printf("Gyro-x = %d\t", sensor_data.sens_data.gyr.x);
                    printf("Gyro-y = %d\t", sensor_data.sens_data.gyr.y);
                    printf("Gyro-z = %d\r\n", sensor_data.sens_data.gyr.z);

                    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                    x = lsb_to_dps(sensor_data.sens_data.gyr.x, 2000, dev.resolution);
                    y = lsb_to_dps(sensor_data.sens_data.gyr.y, 2000, dev.resolution);
                    z = lsb_to_dps(sensor_data.sens_data.gyr.z, 2000, dev.resolution);

                    /* Print the data in dps. */
                    printf("DPS-x = %4.2f, DPS-y = %4.2f, DPS-z = %4.2f\n", x, y, z);
                    limit--;
                }
            }
        }
    }

    return rslt;

    bmi323_coines_deinit();
}

/*!
 *  @brief This internal API is used to set configurations for gyro.
 */
static int8_t set_gyro_config(struct bmi323_dev *dev)
{
    struct bmi323_map_int map_int = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi323_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    bmi323_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.gyr_drdy_int = BMI323_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        bmi323_error_codes_print_result("Map interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
            config.cfg.gyr.odr = BMI323_GYR_ODR_100HZ;

            /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
            config.cfg.gyr.range = BMI323_GYR_RANGE_2000DPS;

            /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
             *  Value   Name      Description
             *    0   odr_half   BW = gyr_odr/2
             *    1  odr_quarter BW = gyr_odr/4
             */
            config.cfg.gyr.bwp = BMI323_GYR_BW_ODR_HALF;

            /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
            config.cfg.gyr.gyr_mode = BMI323_GYR_MODE_NORMAL;

            /* Value    Name    Description
             *  000     avg_1   No averaging; pass sample without filtering
             *  001     avg_2   Averaging of 2 samples
             *  010     avg_4   Averaging of 4 samples
             *  011     avg_8   Averaging of 8 samples
             *  100     avg_16  Averaging of 16 samples
             *  101     avg_32  Averaging of 32 samples
             *  110     avg_64  Averaging of 64 samples
             */
            config.cfg.gyr.avg_num = BMI323_GYR_AVG4;

            /* Set the gyro configurations. */
            rslt = bmi323_set_sensor_config(&config, 1, dev);
            bmi323_error_codes_print_result("Set sensor config", rslt);
        }
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI323_GYR_RANGE_2000DPS)) * (val);
}
