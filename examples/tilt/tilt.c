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
 *  @brief This internal API is used to set configurations for tilt interrupt.
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

    /* Variable to get tilt interrupt status. */
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
            /* Set feature configurations for tilt interrupt. */
            rslt = set_feature_config(&dev);
            bmi323_error_codes_print_result("Set feature config", rslt);

            if (rslt == BMI323_OK)
            {
                feature.tilt_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi323_error_codes_print_result("Sensor enable", rslt);

                if (rslt == BMI323_OK)
                {
                    map_int.tilt_out = BMI323_INT1;

                    /* Map the feature interrupt for tilt feature. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi323_error_codes_print_result("Map interrupt", rslt);
                    printf("Tilt the board in any direction\n");

                    /* Loop to get tilt interrupt. */
                    do
                    {
                        /* Clear buffer. */
                        int_status = 0;

                        /* To get the interrupt status of tilt. */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi323_error_codes_print_result("Get interrupt status", rslt);

                        /* To check the interrupt status of tilt. */
                        if (int_status & BMI323_INT_STATUS_TILT)
                        {
                            printf("Tilt interrupt is generated\n");
                            break;
                        }
                    } while (rslt == BMI323_OK);
                }
            }
        }
    }

    bmi323_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for tilt interrupt.
 */
static int8_t set_feature_config(struct bmi323_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi323_sens_config config[2] = { { 0 } };

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_TILT;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Enable accel by selecting the mode. */
        config[0].cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

        /* Set tilt configuration settings. */
        /* Duration for which the acceleration vector is averaged to be reference vector. Range = 0 to 255. */
        config[1].cfg.tilt.segment_size = 90;

        /* Minimum angle by which the device shall be tilted for event detection. Angle is computed as 256 *
         * cos(angle). Range = 0 to 255. */
        config[1].cfg.tilt.min_tilt_angle = 200;

        /* Set new configurations. */
        rslt = bmi323_set_sensor_config(config, 2, dev);
        bmi323_error_codes_print_result("Set sensor config", rslt);
    }

    return rslt;
}
