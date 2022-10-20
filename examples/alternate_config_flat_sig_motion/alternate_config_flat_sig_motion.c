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
 *  @brief This internal API is used to set configurations for flat, sig-motion and alternate configuration feature.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_feature_config(struct bmi3_dev *dev);

/******************************************************************************/
/*!               Functions                                                   */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable to get interrupt status. */
    uint16_t int_status = 0;

    /* Structure to store alternate status */
    struct bmi3_alt_status alt_status = { 0 };

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Feature enable initialization. */
    struct bmi3_feature_enable feature = { 0 };

    /* Interrupt mapping structure. */
    struct bmi3_map_int map_int = { 0 };

    /* Create an instance of sensor data structure. */
    struct bmi3_sensor_data sensor_data[2] = { 0 };

    uint8_t limit = 2;
    uint8_t count = 0;

    /* Select accel sensor. */
    sensor_data[0].type = BMI323_ACCEL;

    /* Select gyro sensor. */
    sensor_data[1].type = BMI323_GYRO;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323. */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            /* Set feature configurations. */
            rslt = set_feature_config(&dev);
            bmi3_error_codes_print_result("Set feature config", rslt);

            if (rslt == BMI323_OK)
            {
                /* Enable flat and sig-motion feature */
                feature.flat_en = BMI323_ENABLE;
                feature.sig_motion_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi3_error_codes_print_result("Sensor select", rslt);

                if (rslt == BMI323_OK)
                {
                    /* Select the feature and map the interrupt to pin BMI3_INT1 or BMI3_INT2 */
                    map_int.flat_out = BMI3_INT1;
                    map_int.sig_motion_out = BMI3_INT1;
                    map_int.acc_drdy_int = BMI3_INT1;
                    map_int.gyr_drdy_int = BMI3_INT1;

                    /* Map the feature interrupt. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi3_error_codes_print_result("Map interrupt", rslt);

                    printf("\nSIG-MOTION/FLAT ALTERNATE CONFIGURATION TEST\n\n");

                    printf("Place the sensor flat to get flat interrupt which runs in alternate config\n");
                    printf("Move the board in the same pattern to get sig-motion interrupt which runs in user config\n");

                    /* Delay to make sure for the user to see the current test scenario */
                    dev.delay_us(2000000, dev.intf_ptr);

                    for (;;)
                    {
                        /* Clear buffer */
                        int_status = 0;
                        alt_status.alt_accel_status = 0;
                        alt_status.alt_gyro_status = 0;

                        dev.delay_us(5000, dev.intf_ptr);

                        /* Read the interrupt status from int 1 pin */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi3_error_codes_print_result("Read interrupt status", rslt);

                        /* To check the accel data ready interrupt status and print the status of x, y and z-axis.
                         * */
                        if (int_status & BMI3_INT_STATUS_ACC_DRDY)
                        {
                            /* Get accelerometer data for x, y and z-axis. */
                            rslt = bmi323_get_sensor_data(&sensor_data[0], 1, &dev);
                            bmi3_error_codes_print_result("Get sensor data", rslt);

                            printf("Accel-x = %d\tAccel-y = %d\tAccel-z = %d\tSensor time %ld\n",
                                   sensor_data[0].sens_data.acc.x,
                                   sensor_data[0].sens_data.acc.y,
                                   sensor_data[0].sens_data.acc.z,
                                   (long int)sensor_data[0].sens_data.acc.sens_time);
                        }

                        /* To check the gyro data ready interrupt status and print the status of x, y and z-axis. */
                        if (int_status & BMI3_INT_STATUS_GYR_DRDY)
                        {
                            /* Get gyro data for x, y and z-axis. */
                            rslt = bmi323_get_sensor_data(&sensor_data[1], 1, &dev);
                            bmi3_error_codes_print_result("Get sensor data", rslt);

                            printf("Gyro-x = %d\tGyro-y = %d\tGyro-z = %d\tSensor time %ld\n",
                                   sensor_data[1].sens_data.gyr.x,
                                   sensor_data[1].sens_data.gyr.y,
                                   sensor_data[1].sens_data.gyr.z,
                                   (long int)sensor_data[1].sens_data.gyr.sens_time);
                        }

                        /* To check the interrupt status of sig-motion. */
                        if (int_status & BMI3_INT_STATUS_SIG_MOTION)
                        {
                            printf("\nSignificant motion interrupt is generated\n");

                            rslt = bmi323_read_alternate_status(&alt_status, &dev);
                            printf("Alternate accel status %d\n", alt_status.alt_accel_status);
                            printf("Alternate gyro status %d\n", alt_status.alt_gyro_status);

                            count++;
                        }

                        /* Check the interrupt status of the flat */
                        if (int_status & BMI3_INT_STATUS_FLAT)
                        {
                            printf("\nFlat interrupt generated\n");

                            rslt = bmi323_read_alternate_status(&alt_status, &dev);
                            printf("Alternate accel status %d\n", alt_status.alt_accel_status);
                            printf("Alternate gyro status %d\n", alt_status.alt_gyro_status);
                        }

                        if (count == limit)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for flat, sig-motion and alternate configuration feature.
 */
static int8_t set_feature_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi3_sens_config config[7] = { 0 };

    /* Configure type of feature */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_FLAT;
    config[2].type = BMI323_SIG_MOTION;
    config[3].type = BMI323_ALT_AUTO_CONFIG;
    config[4].type = BMI323_ALT_ACCEL;
    config[5].type = BMI323_GYRO;
    config[6].type = BMI323_ALT_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 7, dev);
    bmi3_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Sensor configuration settings */
        /* Enable accel by selecting the mode */
        config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

        config[0].cfg.acc.odr = BMI3_ACC_ODR_100HZ;

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
         * Eg: If flat is assigned to alternate configuration, then sig-motion can be assigned to user configuration and vice versa. */
        config[3].cfg.alt_auto_cfg.alt_conf_alt_switch_src_select = BMI3_ALT_FLAT;
        config[3].cfg.alt_auto_cfg.alt_conf_user_switch_src_select = BMI3_ALT_SIG_MOTION;

        /* Alternate configuration settings for accel */
        config[4].cfg.alt_acc.alt_acc_mode = BMI3_ALT_ACC_MODE_NORMAL;
        config[4].cfg.alt_acc.alt_acc_odr = BMI3_ALT_ACC_ODR_400HZ;
        config[4].cfg.alt_acc.alt_acc_avg_num = BMI3_ALT_ACC_AVG4;

        /* Enable gyro by selecting the mode */
        config[5].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

        config[5].cfg.gyr.odr = BMI3_GYR_ODR_100HZ;

        /* Alternate configuration settings for gyro */
        config[6].cfg.alt_gyr.alt_gyro_mode = BMI3_ALT_GYR_MODE_NORMAL;
        config[6].cfg.alt_gyr.alt_gyro_odr = BMI3_ALT_GYR_ODR_400HZ;
        config[6].cfg.alt_gyr.alt_gyro_avg_num = BMI3_ALT_GYR_AVG4;

        /* Set new configurations. */
        rslt = bmi323_set_sensor_config(config, 7, dev);
        bmi3_error_codes_print_result("Set sensor config", rslt);

        if (rslt == BMI323_OK)
        {
            rslt = bmi323_alternate_config_ctrl((BMI3_ALT_ACC_ENABLE | BMI3_ALT_GYR_ENABLE),
                                                BMI3_ALT_CONF_RESET_OFF,
                                                dev);
            bmi3_error_codes_print_result("Enable alternate config control", rslt);
        }
    }

    return rslt;
}
