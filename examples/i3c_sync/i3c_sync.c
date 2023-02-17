/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <math.h>
#include "bmi323.h"
#include "common.h"

/******************************************************************************/
/*!         Macros definition                                       */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)
#define TEMPERATURE    UINT8_C(0x02)

/******************************************************************************/
/*!         Structure Definition                                              */

/*! Structure to define accelerometer and gyroscope configuration. */
struct bmi3_sens_config config[2];

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for FIFO, accelerometer and gyroscope.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_sensor_config(struct bmi3_dev *dev);

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
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

/******************************************************************************/
/*!               Functions                                                   */

/* This function starts the execution of program. */
int main(void)
{
    struct bmi3_dev dev;
    int8_t rslt;

    /* Variable to set the data sample rate of i3c sync
     * 0x0032 is set to 50 samples this value can vary  */
    uint16_t sample_rate = 0x0032;

    /* Variable to set the delay time of i3c sync */
    uint8_t delay_time = BMI3_I3C_SYNC_DIVISION_FACTOR_11;

    /* Variable to set the i3c sync ODR */
    uint8_t odr = BMI3_I3C_SYNC_ODR_50HZ;

    /* Variable to enable the filer */
    uint8_t i3c_tc_res = BMI323_ENABLE;

    uint8_t limit = 20;

    uint16_t int_status;

    struct bmi3_feature_enable feature = { 0 };

    struct bmi3_sensor_data sensor_data[3];

    /* Variable to store temperature */
    float temperature_value;

    float acc_x = 0, acc_y = 0, acc_z = 0;
    float gyr_x = 0, gyr_y = 0, gyr_z = 0;
    uint8_t indx = 0;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize BMI323 */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            rslt = bmi323_configure_enhanced_flexibility(&dev);
            bmi3_error_codes_print_result("bmi323_configure_enhanced_flexibility", rslt);

            if (rslt == BMI323_OK)
            {
                rslt = set_sensor_config(&dev);
                bmi3_error_codes_print_result("set_sensor_config", rslt);

                /* Enable i3c_sync feature */
                feature.i3c_sync_en = BMI323_ENABLE;

                /* Enable the selected sensors */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi3_error_codes_print_result("bmi323_select_sensor", rslt);

                /* Set the data sample rate of i3c sync */
                rslt = bmi323_set_i3c_tc_sync_tph(sample_rate, &dev);
                bmi3_error_codes_print_result("set_i3c_tc_sync_tph", rslt);

                /* Set the delay time of i3c sync */
                rslt = bmi323_set_i3c_tc_sync_tu(delay_time, &dev);
                bmi3_error_codes_print_result("set_i3c_tc_sync_tu", rslt);

                /* Set i3c sync ODR */
                rslt = bmi323_set_i3c_tc_sync_odr(odr, &dev);
                bmi3_error_codes_print_result("set_i3c_tc_sync_odr", rslt);

                /* Enable the i3c sync filter */
                rslt = bmi323_set_i3c_sync_i3c_tc_res(i3c_tc_res, &dev);
                bmi3_error_codes_print_result("set_i3c_sync_i3c_tc_res", rslt);

                /* After any change to the I3C-Sync configuration parameters,
                 * a config changed CMD (0x0201) must be written
                 * to CMD register to update the internal configuration */
                rslt = bmi323_set_command_register(BMI3_CMD_I3C_TCSYNC_UPDATE, &dev);
                bmi3_error_codes_print_result("bmi323_set_command_register", rslt);

                /* Set the type of sensor data */
                sensor_data[ACCEL].type = BMI3_I3C_SYNC_ACCEL;
                sensor_data[GYRO].type = BMI3_I3C_SYNC_GYRO;
                sensor_data[TEMPERATURE].type = BMI3_I3C_SYNC_TEMP;

                printf("I3C accel, gyro and temperature data\n");

                printf(
                    "\nDATA_SET, i3c_acc_x, i3c_acc_y, i3c_acc_z, i3c_time, Gravity-x, Gravity-y, Gravity-z, i3c_gyro_x, i3c_gyro_y, i3c_gyro_z, DPS-x, DPS-y, DPS-z, i3c_temperature, i3c_sync_time, Temperature data(Degree Celcius)\n");

                while (indx <= limit)
                {
                    /* Delay provided based on i3c sync ODR */
                    dev.delay_us(20000, dev.intf_ptr);

                    /* To get the status of i3c sync interrupt status. */
                    rslt = bmi323_get_int1_status(&int_status, &dev);
                    bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

                    /* To check the i3c sync accel data */
                    if (int_status & BMI3_INT_STATUS_I3C)
                    {
                        rslt = bmi323_get_sensor_data(sensor_data, 3, &dev);
                        bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

                        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                        acc_x = lsb_to_mps2((int16_t)sensor_data[ACCEL].sens_data.i3c_sync.sync_x, 2, dev.resolution);
                        acc_y = lsb_to_mps2((int16_t)sensor_data[ACCEL].sens_data.i3c_sync.sync_y, 2, dev.resolution);
                        acc_z = lsb_to_mps2((int16_t)sensor_data[ACCEL].sens_data.i3c_sync.sync_z, 2, dev.resolution);

                        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                        gyr_x = lsb_to_dps((int16_t)sensor_data[GYRO].sens_data.i3c_sync.sync_x,
                                           (float)2000,
                                           dev.resolution);
                        gyr_y = lsb_to_dps((int16_t)sensor_data[GYRO].sens_data.i3c_sync.sync_y,
                                           (float)2000,
                                           dev.resolution);
                        gyr_z = lsb_to_dps((int16_t)sensor_data[GYRO].sens_data.i3c_sync.sync_z,
                                           (float)2000,
                                           dev.resolution);

                        temperature_value =
                            (float)((((float)((int16_t)sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_temp)) /
                                     512.0) +
                                    23.0);

                        printf(
                            "%d, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %f\t\n",
                            indx,
                            sensor_data[ACCEL].sens_data.i3c_sync.sync_x,
                            sensor_data[ACCEL].sens_data.i3c_sync.sync_y,
                            sensor_data[ACCEL].sens_data.i3c_sync.sync_z,
                            sensor_data[ACCEL].sens_data.i3c_sync.sync_time,
                            acc_x,
                            acc_y,
                            acc_z,
                            sensor_data[GYRO].sens_data.i3c_sync.sync_x,
                            sensor_data[GYRO].sens_data.i3c_sync.sync_y,
                            sensor_data[GYRO].sens_data.i3c_sync.sync_z,
                            sensor_data[GYRO].sens_data.i3c_sync.sync_time,
                            gyr_x,
                            gyr_y,
                            gyr_z,
                            sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_temp,
                            sensor_data[TEMPERATURE].sens_data.i3c_sync.sync_time,
                            temperature_value);
                    }

                    indx++;
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accelerometer, gyroscope and FIFO.
 */
static int8_t set_sensor_config(struct bmi3_dev *dev)
{
    int8_t rslt;

    /* Structure to define interrupt with its feature */
    struct bmi3_map_int map_int = { 0 };

    /* Configure type of feature */
    config[ACCEL].type = BMI323_ACCEL;
    config[GYRO].type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    /* Configure the accel and gyro settings */
    config[ACCEL].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
    config[ACCEL].cfg.acc.range = BMI3_ACC_RANGE_2G;
    config[ACCEL].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    config[GYRO].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
    config[GYRO].cfg.gyr.range = BMI3_GYR_RANGE_125DPS;
    config[GYRO].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    rslt = bmi323_set_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);

    /* Get configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    /* Select the feature and map the interrupt to pin BMI3_INT1 or BMI3_INT2 */
    map_int.i3c_out = BMI3_INT1;

    /* Map i3c sync interrupt to interrupt pin. */
    rslt = bmi323_map_interrupt(map_int, dev);
    bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
