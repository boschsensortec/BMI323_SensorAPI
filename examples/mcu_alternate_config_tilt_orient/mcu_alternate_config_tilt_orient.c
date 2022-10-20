/**\
 * Copyright (c) 2022 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

#include <stdio.h>
#include <stdlib.h>
#include "bmi323.h"
#include "common.h"
#include "coines.h"
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
 * @brief This internal API is used to set the configurations for tilt, orient and alternate configuration feature.
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
 * @brief This internal API is used to set the configurations for tilt, orient and alternate configuration feature.
 */
static void set_feature_config(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi323_sens_config config[7] = { 0 };

    /* Configure the type of feature */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_TILT;
    config[2].type = BMI323_ORIENTATION;
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

        config[0].cfg.acc.odr = BMI323_ACC_ODR_100HZ;

        /* Set tilt configuration settings */
        config[1].cfg.tilt.segment_size = 90;
        config[1].cfg.tilt.min_tilt_angle = 200;

        /* Set orientation configuration settings */
        config[2].cfg.orientation.hold_time = 4;
        config[2].cfg.orientation.hysteresis = 5;
        config[2].cfg.orientation.theta = 16;
        config[2].cfg.orientation.mode = 1;
        config[2].cfg.orientation.slope_thres = 30;

        /* Assign the features to user and alternate switch
         * NOTE: Any of one the feature (either tilt or orient) can be assigned to alternate configuration.
         * Eg: If tilt is assigned to alternate configuration, then orient can be assigned to user configuration and vice versa. */
        config[3].cfg.alt_auto_cfg.alt_switch_src_select = BMI323_ALT_TILT;
        config[3].cfg.alt_auto_cfg.user_switch_src_select = BMI323_ALT_ORIENT;

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

    uint8_t limit = 5;
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

    /* Set the configurations for tilt, orient and alternate configuration feature */
    set_feature_config();

    /* Enable tilt and orientation feature */
    feature.tilt_en = BMI323_ENABLE;
    feature.orientation_en = BMI323_ENABLE;

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
    map_int.tilt_out = BMI323_INT2;
    map_int.orientation_out = BMI323_INT2;
    map_int.acc_drdy_int = BMI323_INT1;
    map_int.gyr_drdy_int = BMI323_INT1;

    /* Map the feature interrupt. */
    rslt = bmi323_map_interrupt(map_int, &dev);
    bmi323_error_codes_print_result("Map interrupt", rslt);

    coines_attach_interrupt(COINES_SHUTTLE_PIN_20, drdy_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);
    coines_attach_interrupt(COINES_SHUTTLE_PIN_21, feat_int_callback, COINES_PIN_INTERRUPT_FALLING_EDGE);

    printf("Tilt the board for tilt interrupt which runs in alternate config\n");
    printf("Move the board at different orientations which runs in user config\n");

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

            /* Check the interrupt status of tilt */
            if (feat_int & BMI323_INT_STATUS_TILT)
            {
                printf("\nTilt detected\n");

                rslt = bmi323_read_alternate_status(&alt_status, &dev);
                printf("Alternate accel status %d\n", alt_status.alt_accel_status);
                printf("Alternate gyro status %d\n", alt_status.alt_gyro_status);

                count++;
            }

            /* To check the interrupt status of orientation interrupt. */
            if (feat_int & BMI323_INT_STATUS_ORIENTATION)
            {
                printf("\nOrientation interrupt is generated\n");

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
