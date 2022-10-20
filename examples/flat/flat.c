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
 *  @brief This internal API is used to set configurations for flat interrupt.
 *
 *  @param[in] dev       : Structure instance of bmi323_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_feature_config(struct bmi323_dev *dev);

/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable to get flat interrupt status. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    /* Feature enable initialization. */
    struct bmi323_feature_enable feature = { 0 };

    /* Interrupt mapping structure. */
    struct bmi323_map_int map_int = { 0 };

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
            /* Set feature configurations for flat interrupt. */
            rslt = set_feature_config(&dev);
            bmi323_error_codes_print_result("Set feature config", rslt);

            if (rslt == BMI323_OK)
            {
                feature.flat_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi323_error_codes_print_result("Sensor enable", rslt);

                if (rslt == BMI323_OK)
                {
                    map_int.flat_out = BMI323_INT1;

                    /* Map the feature interrupt for flat feature. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi323_error_codes_print_result("Map interrupt", rslt);
                    printf("Keep the board on a flat surface\n");

                    /* Loop to get flat interrupt. */
                    do
                    {
                        /* Clear buffer. */
                        int_status = 0;

                        /* To get the interrupt status of flat. */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi323_error_codes_print_result("Get interrupt status", rslt);

                        /* To check the interrupt status of flat. */
                        if (int_status & BMI323_INT_STATUS_FLAT)
                        {
                            printf("Flat interrupt is generated\n");
                            break;
                        }
                    } while (rslt == BMI323_OK);
                }
            }
        }
    }

    return rslt;

    bmi323_coines_deinit();
}

/*!
 * @brief This internal API is used to set configurations for flat interrupt.
 */
static int8_t set_feature_config(struct bmi323_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi323_sens_config config[2] = { { 0 } };

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_FLAT;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Enable accel by selecting the mode. */
        config[0].cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

        /* Maximum allowed tilt angle for device to be in flat state. Angle is computed as 64 * (tan(angle)^2). Range =
         * 0 to 63. */
        config[1].cfg.flat.theta = 9;

        /* Blocking mode to prevent change of flat status during large movement of device.
         * Value    Name        Description
         *  00     MODE_0   Blocking is disabled
         *  01     MODE_1   Block if acceleration on any axis is greater than 1.5g
         *  10     MODE_2   Block if acceleration on any axis is greater than 1.5g or slope is greater than half of
         *                  slope_thres
         *  11     MODE_3   Block if acceleration on any axis is greater than 1.5g or slope is greater than slope_thres
         */
        config[1].cfg.flat.blocking = 3;

        /* Minimum duration the device shall be in flat position for status to be asserted. Range = 0 to 255 */
        config[1].cfg.flat.hold_time = 50;

        /* Set new configurations. */
        rslt = bmi323_set_sensor_config(config, 2, dev);
        bmi323_error_codes_print_result("Set sensor config", rslt);
    }

    return rslt;
}
