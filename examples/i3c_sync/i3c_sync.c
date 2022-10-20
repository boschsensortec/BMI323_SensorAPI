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
/*!         Macros definition                                       */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
#define TEMPERATURE    UINT8_C(0x02)

/* Structure to define accelerometer and gyroscope configuration. */
struct bmi323_sens_config config[2];

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for FIFO, accelerometer and gyroscope.
 *
 *  @param[in] dev       : Structure instance of bmi323_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_sensor_config(struct bmi323_dev *dev);

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

int main(void)
{
    struct bmi323_dev dev;
    int8_t rslt;

    /* Variable to set the data sample rate of i3c sync
     * 0x0032 is set to 50 samples this value can vary  */
    uint16_t sample_rate = 0x0032;

    /* Variable to set the delay time of i3c sync */
    uint8_t delay_time = BMI3XO_I3C_SYNC_ODR_100HZ;

    /* Variable to set the i3c sync ODR */
    uint8_t odr = BMI3XO_I3C_SYNC_ODR_100HZ;

    /* Variable to enable the filer */
    uint8_t filter_en = BMI323_ENABLE;

    uint8_t limit = 20;

    uint16_t int_status;

    struct bmi323_feature_enable feature = { 0 };

    struct bmi323_sensor_data sensor_data[3];

    /* Variable to store temperature */
    float temperature_value;

    float x = 0, y = 0, z = 0;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI323_I2C_INTF
     * For SPI : BMI323_SPI_INTF
     */
    rslt = bmi323_interface_init(&dev, BMI323_SPI_INTF);
    bmi323_error_codes_print_result("bmi323_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize BMI323 */
        rslt = bmi323_init(&dev);
        bmi323_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            rslt = bmi323_configure_enhanced_flexibility(&dev);
            bmi323_error_codes_print_result("bmi323_configure_enhanced_flexibility", rslt);

            if (rslt == BMI323_OK)
            {
                rslt = set_sensor_config(&dev);
                bmi323_error_codes_print_result("set_sensor_config", rslt);

                /* Enable 13c_sync feature */
                feature.i3c_sync_en = BMI323_ENABLE;

                /* Enable the selected sensors */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi323_error_codes_print_result("bmi323_select_sensor", rslt);

                /* Set the data sample rate of i3c sync */
                rslt = bmi323_set_i3c_tc_sync_tph(sample_rate, &dev);
                bmi323_error_codes_print_result("set_i3c_tc_sync_tph", rslt);

                /* Set the delay time of i3c sync */
                rslt = bmi323_set_i3c_tc_sync_tu(delay_time, &dev);
                bmi323_error_codes_print_result("set_i3c_tc_sync_tu", rslt);

                /* Set i3c sync ODR */
                rslt = bmi323_set_i3c_tc_sync_odr(odr, &dev);
                bmi323_error_codes_print_result("set_i3c_tc_sync_odr", rslt);

                /* Enable the i3c sync filter */
                rslt = bmi323_set_i3c_sync_filter_en(filter_en, &dev);
                bmi323_error_codes_print_result("set_i3c_sync_filter_en", rslt);

                /* After any change to the I3C-Sync configuration parameters,
                 * a config changed CMD (0x0201) must be written
                 * to CMD register to update the internal configuration */
                rslt = bmi323_set_command_register(BMI323_CMD_I3C_TCSYNC_UPDATE, &dev);
                bmi323_error_codes_print_result("bmi323_set_command_register", rslt);

                /* Set the type of sensor data */
                sensor_data[ACCEL].type = BMI323_I3C_SYNC_ACCEL;
                sensor_data[GYRO].type = BMI323_I3C_SYNC_GYRO;
                sensor_data[TEMPERATURE].type = BMI323_I3C_SYNC_TEMP;

                printf("I3C accel, gyro and temperature data\n");

                while (limit > 0)
                {
                    /* Delay provided based on i3c sync ODR */
                    dev.delay_us(10000, dev.intf_ptr);

                    /* To get the status of i3c sync interrupt status. */
                    rslt = bmi323_get_int1_status(&int_status, &dev);
                    bmi323_error_codes_print_result("bmi323_get_int1_status", rslt);

                    /* To check the i3c sync accel data */
                    if (int_status & BMI323_INT_STATUS_I3C)
                    {
                        rslt = bmi323_get_sensor_data(sensor_data, 3, &dev);
                        bmi323_error_codes_print_result("bmi323_get_sensor_data", rslt);

                        printf("\ni3c_acc_x = %d\t i3c_acc_y = %d\t i3c_acc_z = %d\t i3c_time = %d\t\n",
                               sensor_data[ACCEL].sens_data.i3c_sync.sync_x,
                               sensor_data[ACCEL].sens_data.i3c_sync.sync_y,
                               sensor_data[ACCEL].sens_data.i3c_sync.sync_z,
                               sensor_data[ACCEL].sens_data.i3c_sync.sync_time);

                        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                        x = lsb_to_mps2(sensor_data[ACCEL].sens_data.i3c_sync.sync_x, 2, dev.resolution);
                        y = lsb_to_mps2(sensor_data[ACCEL].sens_data.i3c_sync.sync_y, 2, dev.resolution);
                        z = lsb_to_mps2(sensor_data[ACCEL].sens_data.i3c_sync.sync_z, 2, dev.resolution);

                        /* Print the data in m/s2. */
                        printf("Gravity-x = %4.2f, Gravity-y = %4.2f, Gravity-z = %4.2f\n", x, y, z);

                        printf("i3c_gyro_x = %d\t i3c_gyro_y = %d\t i3c_gyro_z = %d\t i3c_sync_time = %d\t\n",
                               sensor_data[GYRO].sens_data.i3c_sync.sync_x,
                               sensor_data[GYRO].sens_data.i3c_sync.sync_y,
                               sensor_data[GYRO].sens_data.i3c_sync.sync_z,
                               sensor_data[GYRO].sens_data.i3c_sync.sync_time);

                        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                        x = lsb_to_dps(sensor_data[GYRO].sens_data.i3c_sync.sync_x, 2000, dev.resolution);
                        y = lsb_to_dps(sensor_data[GYRO].sens_data.i3c_sync.sync_y, 2000, dev.resolution);
                        z = lsb_to_dps(sensor_data[GYRO].sens_data.i3c_sync.sync_z, 2000, dev.resolution);

                        /* Print the data in dps. */
                        printf("DPS-x = %4.2f, DPS-y = %4.2f, DPS-z = %4.2f\n", x, y, z);

                        printf("i3c_temperature = %d\t i3c_sync_time = %d\t\n",
                               sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_temp,
                               sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_time);

                        temperature_value =
                            (float)((float)sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_temp / 512) + 23;

                        /* \370C is for printing the data in degree celsius format */
                        printf("Temperature data %f\370C\t\n", temperature_value);
                    }

                    limit--;
                }
            }
        }
    }

    return rslt;

    bmi323_coines_deinit();
}

/*!
 * @brief This internal API is used to set configurations for accelerometer, gyroscope and FIFO.
 */
static int8_t set_sensor_config(struct bmi323_dev *dev)
{
    int8_t rslt;

    /* Structure to define interrupt with its feature */
    struct bmi323_map_int map_int = { 0 };

    /* Configure type of feature */
    config[ACCEL].type = BMI323_ACCEL;
    config[GYRO].type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("bmi323_get_sensor_config", rslt);

    /* Configure the accel and gyro settings */
    config[ACCEL].cfg.acc.bwp = BMI323_ACC_BW_ODR_QUARTER;
    config[ACCEL].cfg.acc.range = BMI323_ACC_RANGE_2G;
    config[ACCEL].cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

    config[GYRO].cfg.gyr.bwp = BMI323_GYR_BW_ODR_HALF;
    config[GYRO].cfg.gyr.range = BMI323_GYR_RANGE_125DPS;
    config[GYRO].cfg.gyr.gyr_mode = BMI323_GYR_MODE_NORMAL;

    rslt = bmi323_set_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("bmi323_set_sensor_config", rslt);

    /* Get configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("bmi323_get_sensor_config", rslt);

    /* Select the feature and map the interrupt to pin BMI323_INT1 or BMI323_INT2 */
    map_int.i3c_out = BMI323_INT1;

    /* Map i3c sync interrupt to interrupt pin. */
    rslt = bmi323_map_interrupt(map_int, dev);
    bmi323_error_codes_print_result("bmi323_map_interrupt", rslt);

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
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale) + BMI323_GYR_RANGE_2000DPS)) * (val);
}
