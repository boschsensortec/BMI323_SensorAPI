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
/*!         Macros definition                                                 */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi323_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_config(struct bmi323_dev *dev);

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Gravity.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This internal API is used to read the accel data before and after axis remap.
 *
 *  @param[in] sensor_data       : Structure instance of bmi323_sensor_data.
 *  @param[in] dev               : Structure instance of bmi323_dev.
 *
 *  @return Status of execution.
 */
static int8_t axis_remap_accel_data(struct bmi323_sensor_data *sensor_data, struct bmi323_dev *dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    struct bmi323_axes_remap remap = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Create an instance of sensor data structure. */
    struct bmi323_sensor_data sensor_data = { 0 };

    /* Select accel sensor. */
    sensor_data.type = BMI323_ACCEL;

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
            printf("\nACCEL DATA BEFORE PERFORMING AXIS REMAP\n\n");

            rslt = axis_remap_accel_data(&sensor_data, &dev);
            bmi323_error_codes_print_result("axis_remap_accel_data", rslt);

            if (rslt == BMI323_OK)
            {
                rslt = bmi323_get_remap_axes(&remap, &dev);
                bmi323_error_codes_print_result("get_remap_axes", rslt);

                /* @note: XYZ axis denotes x = x, y = y, z = z
                 * Similarly,
                 *    ZYX, x = z, y = y, z = x
                 */
                remap.axis_map = BMI323_ZYX;

                /* Invert the z axis of accelerometer and gyroscope */
                remap.invert_z = BMI323_INVERT;

                if (rslt == BMI323_OK)
                {
                    rslt = bmi323_set_remap_axes(remap, &dev);
                    bmi323_error_codes_print_result("set_remap_axes", rslt);

                    if (rslt == BMI323_OK)
                    {
                        printf("\nACCEL DATA AFTER PERFORMING AXIS REMAP\n\n");

                        rslt = axis_remap_accel_data(&sensor_data, &dev);
                        bmi323_error_codes_print_result("axis_remap_accel_data", rslt);
                    }
                }
            }
        }
    }

    bmi323_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_config(struct bmi323_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi323_sens_config config;

    /* Structure to map interrupt */
    struct bmi323_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config.type = BMI323_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    bmi323_error_codes_print_result("bmi323_get_sensor_config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.acc_drdy_int = BMI323_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        bmi323_error_codes_print_result("bmi323_map_data_int", rslt);

        if (rslt == BMI323_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config.cfg.acc.odr = BMI323_ACC_ODR_100HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config.cfg.acc.range = BMI323_ACC_RANGE_2G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config.cfg.acc.bwp = BMI323_ACC_BW_ODR_QUARTER;

            /* Set number of average samples for accel. */
            config.cfg.acc.avg_num = BMI323_ACC_AVG64;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config.cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

            /* Set the accel configurations. */
            rslt = bmi323_set_sensor_config(&config, 1, dev);
            bmi323_error_codes_print_result("bmi323_set_sensor_config", rslt);
        }
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    float gravity;

    uint32_t half_scale = ((1 << bit_width) / 2);

    gravity = (float)((GRAVITY_EARTH * val * g_range) / half_scale);

    return gravity;
}

/*!
 *@brief This internal API is used to read the accel data before and after axis remap.
 */
static int8_t axis_remap_accel_data(struct bmi323_sensor_data *sensor_data, struct bmi323_dev *dev)
{
    /* Variable to define error */
    int8_t rslt;

    /* Variable to define the number of iterations */
    uint16_t limit = 20;

    float x, y, z;

    /* Initialize the interrupt status of accel. */
    uint16_t int_status;

    rslt = set_accel_config(dev);
    bmi323_error_codes_print_result("set_accel_config", rslt);

    while ((rslt == BMI323_OK) && (limit > 0))
    {
        /* To get the status of accel data ready interrupt. */
        rslt = bmi323_get_int1_status(&int_status, dev);
        bmi323_error_codes_print_result("bmi323_get_int1_status", rslt);

        /* To check the accel data ready interrupt status and print the status for 10 samples. */
        if (int_status & BMI323_INT_STATUS_ACC_DRDY)
        {
            /* Get accelerometer data for x, y and z axis. */
            rslt = bmi323_get_sensor_data(sensor_data, 1, dev);
            bmi323_error_codes_print_result("Get sensor data", rslt);

            printf("Accel-x = %d\t", sensor_data->sens_data.acc.x);
            printf("Accel-y = %d\t", sensor_data->sens_data.acc.y);
            printf("Accel-z = %d\n", sensor_data->sens_data.acc.z);

            /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
            x = lsb_to_mps2(sensor_data->sens_data.acc.x, 2, dev->resolution);
            y = lsb_to_mps2(sensor_data->sens_data.acc.y, 2, dev->resolution);
            z = lsb_to_mps2(sensor_data->sens_data.acc.z, 2, dev->resolution);

            /* Print the data in m/s2. */
            printf("Gravity-x = %4.2f, Gravity-y = %4.2f, Gravity-z = %4.2f\n", x, y, z);
            limit--;
        }
    }

    return rslt;
}
