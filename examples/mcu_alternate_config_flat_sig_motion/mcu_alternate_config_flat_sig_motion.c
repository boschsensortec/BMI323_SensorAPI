/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include "coines.h"
#include <stdlib.h>
#include "bmi323.h"
#include "common.h"
#include <stdbool.h>

/*********************************************************************/
/*          Global variable declaration                              */
/*********************************************************************/

volatile bool drdy_int_status = 0;
volatile bool feat_int_status = 0;

struct bmi323_dev dev;

/*********************************************************************/
/*          Function declaration                                     */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initialize the bmi323 sensor
 *
 *  @param[in] void
 *
 *  @return void
 *
 */
static void init_bmi323(void);

/*!
 * @brief This internal API is used to set the data ready interrupt status
 *
 *  @param[in] void
 *
 *  @return void
 */
static void drdy_int_callback(void);

/*!
 * @brief This internal API is used to set the feature interrupt status
 *
 *  @param[in] void
 *
 *  @return void
 */
static void feat_int_callback(void);

/*!
 * @brief This internal API is used to set the configurations for flat, sig-motion and alternate configuration feature.
 *
 *  @param[in] void
 *
 *  @return void
 */
static void set_feature_config(void);

/*********************************************************************/
/*           Functions                                               */
/*********************************************************************/

/*!
 *  @brief This internal API is used to initialize the bmi323 sensor
 */
static void init_bmi323(void)
{
    int8_t rslt = BMI323_OK;

    /* Initialize bmi323 sensor */
    rslt = bmi323_init(&dev);
    bmi323_error_codes_print_result("bmi323_init", rslt);

    if (rslt == BMI323_OK)
    {
        printf("BMI323 initialization success!\n");
        printf("Chip ID - 0x%x\n", dev.chip_id);
    }
    else
    {
        printf("BMI323 initialization failure!\n");
        exit(COINES_E_FAILURE);
    }
}

/*!
 * @brief This internal API is used to set the configurations for flat, sig-motion and alternate configuration feature.
 */
static void set_feature_config(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi323_sens_config config[7] = { 0 };

    /* Configure the type of feature */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_FLAT;
    config[2].type = BMI323_SIG_MOTION;
    config[3].type = BMI323_ALT_AUTO_CONFIG;
    config[4].type = BMI323_ALT_ACCEL;
    config[5].type = BMI323_GYRO;
    config[6].type = BMI323_ALT_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 7, &dev);
    bmi323_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Sensor configuration settings */
        /* Enable accel by selecting the mode */
        config[0].cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

        config[0].cfg.acc.odr = BMI323_ACC_ODR_800HZ;

        /* Set flat configuration settings */
        config[1].cfg.flat.theta = 9;
        config[1].cfg.flat.blocking = 3;
        config[1].cfg.flat.hold_time = 50;

        /* Set sig-motion configuration settings */
        config[2].cfg.sig_motion.block_size = 200;
        config[2].cfg.sig_motion.peak_2_peak_max = 30;
        config[2].cfg.sig_motion.peak_2_peak_min = 30;

        /* Assign the features to user and alternate switch
         * NOTE: Any of one the feature (either flat or sig-motion) can be assigned to alternate configuration.
         * Eg: If sig-motion is assigned to alternate configuration, then flat can be assigned to user configuration and vice versa. */
        config[3].cfg.alt_auto_cfg.alt_switch_src_select = BMI323_ALT_SIG_MOTION;
        config[3].cfg.alt_auto_cfg.user_switch_src_select = BMI323_ALT_FLAT;

        /* Alternate configuration settings for accel */
        config[4].cfg.alt_acc.alt_acc_mode = BMI323_ACC_MODE_NORMAL;
        config[4].cfg.alt_acc.alt_acc_odr = BMI323_ACC_ODR_400HZ;
        config[4].cfg.alt_acc.alt_acc_avg_num = BMI323_ACC_AVG4;

        /* Enable gyro by selecting the mode */
        config[5].cfg.gyr.gyr_mode = BMI323_GYR_MODE_NORMAL;

        config[5].cfg.gyr.odr = BMI323_GYR_ODR_100HZ;

        /* Alternate configuration settings for gyro */
        config[6].cfg.alt_gyr.alt_gyro_mode = BMI323_GYR_MODE_NORMAL;
        config[6].cfg.alt_gyr.alt_gyro_odr = BMI323_GYR_ODR_400HZ;
        config[6].cfg.alt_gyr.alt_gyro_avg_num = BMI323_GYR_AVG4;

        /* Set new configurations. */
        rslt = bmi323_set_sensor_config(config, 7, &dev);
        bmi323_error_codes_print_result("Set sensor config", rslt);

        if (rslt == BMI323_OK)
        {
            rslt = bmi323_alternate_config_ctrl((BMI323_ALT_ACC_ENABLE | BMI323_ALT_GYR_ENABLE),
                                                BMI323_ALT_CONF_RESET_OFF,
                                                &dev);
            bmi323_error_codes_print_result("Enable alternate config control", rslt);
        }
    }
}

/*!
 * @brief This internal API is used to set the data ready interrupt status
 */
static void drdy_int_callback(void)
{
    drdy_int_status = 1;
}

/*!
 * @brief This internal API is used to set the feature interrupt status
 */
static void feat_int_callback(void)
{
    feat_int_status = 1;
}

/*!
 *  @brief Main Function where the execution getting started to test the code.
 *
 *  @param[in] argc
 *  @param[in] argv
 *
 *  @return status
 *
 */
int main(int argc, char *argv[])
{
    /* Variable to define result */
    int8_t rslt;

    /* Create an instance of sensor data structure */
    struct bmi323_sensor_data sensor_data[2] = { 0 };

    /* Interrupt mapping structure */
    struct bmi323_map_int map_int = { 0 };

    /* Structure to store alternate configuration status */
    struct bmi323_alt_status alt_status = { 0 };

    /* Variable to get data ready interrupt status */
    uint16_t drdy_int;

    /* Variable to get feature interrupt status */
    uint16_t feat_int;

    uint8_t limit = 2;
    uint8_t count = 0;

    /* Feature enable initialization. */
    struct bmi323_feature_enable feature = { 0 };

    /* Structure to define interrupt pin type, mode and configurations */
    struct bmi323_int_pin_config int_cfg = { 0 };

    /* Select accel sensor. */
    sensor_data[0].type = BMI323_ACCEL;

    /* Select gyro sensor. */
    sensor_data[1].type = BMI323_GYRO;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI323_I2C_INTF
     * For SPI : BMI323_SPI_INTF
     */
    rslt = bmi323_interface_init(&dev, BMI323_SPI_INTF);
    bmi323_error_codes_print_result("bmi323 interface init", rslt);

    /* After sensor init introduce 200 msec sleep */
    coines_delay_msec(200);

    /* Initialize the sensor */
    init_bmi323();

    /* Set the configurations for flat, sig-motion and alternate configuration feature */
    set_feature_config();

    /* Enable flat and sig-motion feature */
    feature.flat_en = BMI323_ENABLE;
    feature.sig_mot_en = BMI323_ENABLE;

    /* Enable the selected sensors. */
    rslt = bmi323_select_sensor(&feature, &dev);
    bmi323_error_codes_print_result("bmi323_select_sensor", rslt);

    /* Get the pin configurations */
    rslt = bmi323_get_int_pin_config(&int_cfg, &dev);
    bmi323_error_codes_print_result("bmi323_get_int_pin_config", rslt);

    /* Assign pin type */
    int_cfg.pin_type = BMI323_INT1;
    int_cfg.pin_cfg[0].output_en = BMI323_INT_OUTPUT_ENABLE;
    int_cfg.pin_cfg[0].lvl = BMI323_INT_ACTIVE_HIGH;

    rslt = bmi323_set_int_pin_config(&int_cfg, &dev);
    bmi323_error_codes_print_result("bmi323_set_int_pin_config", rslt);

    int_cfg.pin_type = BMI323_INT2;
    int_cfg.pin_cfg[1].output_en = BMI323_INT_OUTPUT_ENABLE;
    int_cfg.pin_cfg[1].lvl = BMI323_INT_ACTIVE_HIGH;

    /* Set the pin configurations */
    rslt = bmi323_set_int_pin_config(&int_cfg, &dev);
    bmi323_error_codes_print_result("bmi323_set_int_pin_config", rslt);

    /* Select the feature and map the interrupt to pin BMI323_INT1 or BMI323_INT2 */
    map_int.flat_out = BMI323_INT2;
    map_int.sig_mot_out = BMI323_INT2;
    map_int.acc_drdy_int = BMI323_INT1;
    map_int.gyr_drdy_int = BMI323_INT1;

    /* Map the feature interrupt. */
    rslt = bmi323_map_interrupt(map_int, &dev);
    bmi323_error_codes_print_result("Map interrupt", rslt);

    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, drdy_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);
    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, feat_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);

    printf("Place the sensor flat to get flat interrupt which runs in user config\n");
    printf("Move the board in the same pattern to get sig-motion interrupt which runs in alternate config\n");

    while (1)
    {
        if (drdy_int_status == 1)
        {
            /* Clear buffer */
            drdy_int_status = 0;

            /* Read the interrupt status from int 1 pin */
            rslt = bmi323_get_int1_status(&drdy_int, &dev);
            bmi323_error_codes_print_result("Read interrupt status", rslt);

            /* To check the accel data ready interrupt status and print the status of x, y and z-axis. */
            if (drdy_int & BMI323_INT_STATUS_ACC_DRDY)
            {
                /* Get accelerometer data for x, y and z-axis. */
                rslt = bmi323_get_sensor_data(&sensor_data[0], 1, &dev);
                bmi323_error_codes_print_result("Get sensor data", rslt);

                printf("Accel-x = %d\tAccel-y = %d\tAccel-z = %d\tSensor time %ld\n",
                       sensor_data[0].sens_data.acc.x,
                       sensor_data[0].sens_data.acc.y,
                       sensor_data[0].sens_data.acc.z,
                       sensor_data[0].sens_data.acc.sens_time);
            }

            /* To check the gyro data ready interrupt status and print the status of x, y and z-axis. */
            if (drdy_int & BMI323_INT_STATUS_GYR_DRDY)
            {
                /* Get gyro data for x, y and z-axis. */
                rslt = bmi323_get_sensor_data(&sensor_data[1], 1, &dev);
                bmi323_error_codes_print_result("Get sensor data", rslt);

                printf("Gyro-x = %d\tGyro-y = %d\tGyro-z = %d\tSensor time %ld\n",
                       sensor_data[1].sens_data.gyr.x,
                       sensor_data[1].sens_data.gyr.y,
                       sensor_data[1].sens_data.gyr.z,
                       sensor_data[1].sens_data.gyr.sens_time);
            }
        }

        if (feat_int_status == 1)
        {
            feat_int_status = 0;
            alt_status.alt_accel_status = 0;
            alt_status.alt_gyro_status = 0;

            rslt = bmi323_get_int2_status(&feat_int, &dev);
            bmi323_error_codes_print_result("Read interrupt status", rslt);

            /* To check the interrupt status of sig-motion. */
            if (feat_int & BMI323_INT_STATUS_SIG_MOTION)
            {
                printf("\nSignificant motion interrupt is generated\n");

                rslt = bmi323_read_alternate_status(&alt_status, &dev);
                printf("Alternate accel status %d\n", alt_status.alt_accel_status);
                printf("Alternate gyro status %d\n", alt_status.alt_gyro_status);

                count++;
            }

            /* Check the interrupt status for flat */
            if (feat_int & BMI323_INT_STATUS_FLAT)
            {
                printf("\nFlat interrupt generated\n");

                rslt = bmi323_read_alternate_status(&alt_status, &dev);
                printf("Alternate accel status %d\n", alt_status.alt_accel_status);
                printf("Alternate gyro status %d\n", alt_status.alt_gyro_status);
            }
        }

        if (count == limit)
        {
            break;
        }
    }

    /* Close the communication */
    coines_close_comm_intf(COINES_COMM_INTF_USB);

    return EXIT_SUCCESS;
}
