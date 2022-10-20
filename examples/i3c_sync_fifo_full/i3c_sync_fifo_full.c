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
#define GRAVITY_EARTH                     (9.80665f)

#define ACCEL                             UINT8_C(0x00)
#define GYRO                              UINT8_C(0x01)
#define TEMPERATURE                       UINT8_C(0x02)

#define BMI323_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)
#define BMI323_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

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
    uint8_t delay_time = 0;

    /* Variable to set the i3c sync ODR */
    uint8_t odr = BMI3XO_I3C_SYNC_ODR_100HZ;

    /* Variable to enable the filer */
    uint8_t filter_en = BMI323_ENABLE;

    uint16_t int_status;

    /* Variable to index bytes. */
    uint8_t idx;

    struct bmi323_feature_enable feature = { 0 };

    /* Variable to store temperature */
    float temperature_value;

    float x = 0, y = 0, z = 0;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMI323_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Array of accelerometer frames */

    /*! Calculation for frame count: Total frame count = FIFO buffer size(2048 bytes)/ Total frames(6 accelerometer) which equals to 341.
     */
    struct bmi323_fifo_sens_axes_data fifo_accel_data[341];

    /* Array of gyroscope frames */

    /*! Calculation for frame count: Total frame count = FIFO buffer size(2048 bytes)/ Total frames(6 gyroscope) which equals to 341.
     */
    struct bmi323_fifo_sens_axes_data fifo_gyro_data[341];

    /* Array of temperature frames */

    /*! Calculation for frame count: Total frame count = Since Temperature runs based on Accel, Accel buffer size is been provided
     */
    struct bmi323_fifo_temperature_data fifo_temp_data[341];

    /* Initialize FIFO frame structure */
    struct bmi323_fifo_frame fifoframe = { 0 };

    uint16_t fifo_length = 0;

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

                printf("I3C accel, gyro and temperature data");

                /* Update FIFO structure */
                /* Mapping the buffer to store the FIFO data. */
                fifoframe.data = fifo_data;

                /* Length of FIFO frame. */
                fifoframe.length = BMI323_FIFO_RAW_DATA_USER_LENGTH;

                while (1)
                {
                    /* Read FIFO data on interrupt. */
                    rslt = bmi323_get_int1_status(&int_status, &dev);
                    bmi323_error_codes_print_result("bmi323_get_int1_status", rslt);

                    /* To check the status of FIFO full interrupt. */
                    if ((rslt == BMI323_OK) && (int_status & BMI323_INT_STATUS_FFULL))
                    {
                        printf("\nFIFO full interrupt occurred");

                        rslt = bmi323_get_fifo_length(&fifoframe.available_fifo_len, &dev);
                        bmi323_error_codes_print_result("bmi323_get_fifo_length", rslt);

                        /* Convert available fifo length from word to byte */
                        fifo_length = (uint16_t)(fifoframe.available_fifo_len * 2);

                        fifoframe.length = fifo_length + dev.dummy_byte;

                        printf("\nFIFO length in words : %d\n", fifoframe.available_fifo_len);
                        printf("\nFIFO data bytes available : %d \n", fifo_length);
                        printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

                        /* Read FIFO data */
                        rslt = bmi323_read_fifo_data(&fifoframe, &dev);
                        bmi323_error_codes_print_result("bmi323_read_fifo_data", rslt);

                        if (rslt == BMI323_OK)
                        {
                            /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                            rslt = bmi323_extract_accel(fifo_accel_data, &fifoframe, &dev);
                            printf("\nParsed accelerometer data frames: %d\n", fifoframe.avail_fifo_accel_frames);
                            bmi323_error_codes_print_result("bmi323_extract_accel", rslt);

                            if (rslt == BMI323_OK)
                            {
                                printf("Accel data in LSB units and Gravity data in m/s^2\n");

                                /* Print the parsed accelerometer data from the FIFO buffer */
                                for (idx = 0; idx < fifoframe.avail_fifo_accel_frames; idx++)
                                {
                                    printf("ACCEL[%d] X : %d raw LSB Y : %d raw LSB Z : %d raw LSB, sens_time: %d\n",
                                           idx,
                                           fifo_accel_data[idx].x,
                                           fifo_accel_data[idx].y,
                                           fifo_accel_data[idx].z,
                                           fifo_accel_data[idx].sensor_time);

                                    /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range.
                                     * */
                                    x = lsb_to_mps2(fifo_accel_data[idx].x, 2, dev.resolution);
                                    y = lsb_to_mps2(fifo_accel_data[idx].y, 2, dev.resolution);
                                    z = lsb_to_mps2(fifo_accel_data[idx].z, 2, dev.resolution);

                                    /* Print the data in m/s2. */
                                    printf("Gravity-x = %4.2f, Gravity-y = %4.2f, Gravity-z = %4.2f\n", x, y, z);
                                }
                            }
                            else
                            {
                                switch (rslt)
                                {
                                    case BMI323_W_PARTIAL_READ:
                                        printf("fifo_partial_read\n");
                                        break;
                                    case BMI323_W_FIFO_ACCEL_DUMMY_FRAME:
                                        printf("fifo_accel_dummy_frame\n");
                                        break;
                                    case BMI323_W_ST_PARTIAL_READ:
                                        printf("sensortime_partial_read\n");
                                        break;
                                    default:
                                        break;
                                }
                            }

                            /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
                            rslt = bmi323_extract_gyro(fifo_gyro_data, &fifoframe, &dev);
                            printf("\nParsed gyroscope data frames: %d\n", fifoframe.avail_fifo_gyro_frames);
                            bmi323_error_codes_print_result("bmi323_extract_gyro", rslt);

                            if (rslt == BMI323_OK)
                            {
                                printf("Gyro data in LSB units and degrees per second\n");

                                /* Print the parsed gyroscope data from the FIFO buffer */
                                for (idx = 0; idx < fifoframe.avail_fifo_gyro_frames; idx++)
                                {
                                    printf("GYRO[%d] X : %d raw LSB Y : %d raw LSB Z : %d raw LSB, sens_time: %d\n",
                                           idx,
                                           fifo_gyro_data[idx].x,
                                           fifo_gyro_data[idx].y,
                                           fifo_gyro_data[idx].z,
                                           fifo_gyro_data[idx].sensor_time);

                                    /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                                    x = lsb_to_dps(fifo_gyro_data[idx].x, 2000, dev.resolution);
                                    y = lsb_to_dps(fifo_gyro_data[idx].y, 2000, dev.resolution);
                                    z = lsb_to_dps(fifo_gyro_data[idx].z, 2000, dev.resolution);

                                    /* Print the data in dps. */
                                    printf("DPS-x = %4.2f, DPS-y = %4.2f, DPS-z = %4.2f\n", x, y, z);
                                }
                            }
                            else
                            {
                                switch (rslt)
                                {
                                    case BMI323_W_PARTIAL_READ:
                                        printf("fifo_partial_read\n");
                                        break;
                                    case BMI323_W_FIFO_GYRO_DUMMY_FRAME:
                                        printf("fifo_gyro_dummy_frame\n");
                                        break;
                                    case BMI323_W_ST_PARTIAL_READ:
                                        printf("sensortime_partial_read\n");
                                        break;
                                    default:
                                        break;
                                }
                            }

                            /* Parse the FIFO data to extract temperature data from the FIFO buffer */
                            rslt = bmi323_extract_temperature(fifo_temp_data, &fifoframe, &dev);
                            printf("\nParsed temperature data frames: %d\n", fifoframe.avail_fifo_temp_frames);
                            bmi323_error_codes_print_result("bmi323_extract_temperature", rslt);

                            if (rslt == BMI323_OK)
                            {
                                /* Print the parsed temperature data from the FIFO buffer */
                                for (idx = 0; idx < fifoframe.avail_fifo_temp_frames; idx++)
                                {
                                    printf("TEMP[%d] fifo_temp_data : %d raw LSB, sens_time: %d\n", idx,
                                           fifo_temp_data[idx].temp_data, fifo_temp_data[idx].sensor_time);

                                    temperature_value = (float)((float)fifo_temp_data[idx].temp_data / 512) + 23;

                                    /* \370C is for printing the data in degree celsius format */
                                    printf("Temperature data %f\370C\t\n", temperature_value);
                                }
                            }
                            else
                            {
                                switch (rslt)
                                {
                                    case BMI323_W_PARTIAL_READ:
                                        printf("fifo_partial_read\n");
                                        break;
                                    case BMI323_W_FIFO_TEMP_DUMMY_FRAME:
                                        printf("fifo_temperature_dummy_frame\n");
                                        break;
                                    case BMI323_W_ST_PARTIAL_READ:
                                        printf("sensortime_partial_read\n");
                                        break;
                                    default:
                                        break;
                                }
                            }
                        }

                        break;
                    }
                }
            }
        }
    }

    bmi323_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accelerometer, gyroscope and FIFO.
 */
static int8_t set_sensor_config(struct bmi323_dev *dev)
{
    int8_t rslt;

    /* Array to define set FIFO flush */
    uint8_t data[2] = { BMI323_ENABLE, 0 };

    /* Structure to define interrupt with its feature */
    struct bmi323_map_int map_int = { 0 };

    /* Configure type of feature */
    config[ACCEL].type = BMI323_ACCEL;
    config[GYRO].type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("get_sensor_fifo_config", rslt);

    /* Configure the accel and gyro settings */
    config[ACCEL].cfg.acc.bwp = BMI323_ACC_BW_ODR_QUARTER;
    config[ACCEL].cfg.acc.range = BMI323_ACC_RANGE_2G;
    config[ACCEL].cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

    config[GYRO].cfg.gyr.bwp = BMI323_GYR_BW_ODR_HALF;
    config[GYRO].cfg.gyr.range = BMI323_GYR_RANGE_125DPS;
    config[GYRO].cfg.gyr.gyr_mode = BMI323_GYR_MODE_NORMAL;

    rslt = bmi323_set_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("set_sensor_fifo_config", rslt);

    /* Get configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("get_sensor_fifo_config", rslt);

    /* To enable the accelerometer, gyroscope, temperature and sensor time in FIFO conf addr */
    rslt = bmi323_set_fifo_config(BMI323_FIFO_ALL_EN, BMI323_ENABLE, dev);
    bmi323_error_codes_print_result("bmi323_set_fifo_config", rslt);

    /* Set the FIFO flush in FIFO control register to clear the FIFO data */
    rslt = bmi323_set_regs(BMI323_REG_FIFO_CTRL, data, 2, dev);
    bmi323_error_codes_print_result("bmi323_set_regs", rslt);

    /* Map the FIFO full interrupt to INT1 */
    /* Note: User can map the interrupt to INT1 or INT2 */
    map_int.ffull_int = BMI323_INT1;

    /* Map the interrupt configuration */
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
