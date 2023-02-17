/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
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
 *  @brief This internal API is used to set configurations for orientation interrupt.
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

    /* Variable to get orientation interrupt status. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Feature enable initialization. */
    struct bmi3_feature_enable feature = { 0 };

    /* Interrupt mapping structure. */
    struct bmi3_map_int map_int = { 0 };

    /* Structure to get orient data. */
    struct bmi3_sensor_data sensor_data = { 0 };

    /* Variables to store the output of orientation. */
    uint8_t orientation_out = 0;
    uint8_t orientation_faceup_down = 0;

    /* Select orient type. */
    sensor_data.type = BMI323_ORIENTATION;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323. */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            /* Set feature configurations for orientation. */
            rslt = set_feature_config(&dev);
            bmi3_error_codes_print_result("Set feature config", rslt);

            if (rslt == BMI323_OK)
            {
                feature.orientation_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi3_error_codes_print_result("Sensor enable", rslt);

                if (rslt == BMI323_OK)
                {
                    map_int.orientation_out = BMI3_INT1;

                    /* Map the feature interrupt for orientation. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi3_error_codes_print_result("Map interrupt", rslt);
                    printf("Move the board in different directions\n");

                    /* Loop to get orientation interrupt. */
                    do
                    {
                        /* Clear buffer. */
                        int_status = 0;

                        /* To get the interrupt status of orientation. */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi3_error_codes_print_result("Get interrupt status", rslt);

                        /* To check the interrupt status of orientation. */
                        if (int_status & BMI3_INT_STATUS_ORIENTATION)
                        {
                            printf("Orientation interrupt is generated\n");

                            rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev);
                            bmi3_error_codes_print_result("Get sensor data", rslt);

                            orientation_out = sensor_data.sens_data.orient_output.orientation_portrait_landscape;
                            orientation_faceup_down = sensor_data.sens_data.orient_output.orientation_faceup_down;

                            switch (orientation_out)
                            {
                                case BMI3_LANDSCAPE_LEFT:
                                    printf("Orientation state is landscape left\n");
                                    break;
                                case BMI3_LANDSCAPE_RIGHT:
                                    printf("Orientation state is landscape right\n");
                                    break;
                                case BMI3_PORTRAIT_UP_DOWN:
                                    printf("Orientation state is portrait upside down\n");
                                    break;
                                case BMI3_PORTRAIT_UP_RIGHT:
                                    printf("Orientation state is portrait upright\n");
                                    break;
                                default:
                                    break;
                            }

                            switch (orientation_faceup_down)
                            {
                                case BMI3_FACE_UP:
                                    printf("Orientation state is face up\n");
                                    break;
                                case BMI3_FACE_DOWN:
                                    printf("Orientation state is face down\n");
                                    break;
                                default:
                                    break;
                            }

                            break;
                        }
                    } while (rslt == BMI323_OK);
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for orientation interrupt.
 */
static int8_t set_feature_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi3_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_ORIENTATION;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Function to select the context feature configurations.
         * For smart phone: BMI323_SMART_PHONE_SEL
         * For wearables  : BMI323_WEARABLE_SEL
         * For hearables  : BMI323_HEARABLE_SEL */
        rslt = bmi323_context_switch_selection(BMI323_WEARABLE_SEL, dev);
        bmi3_error_codes_print_result("context switch selection", rslt);

        if (rslt == BMI323_OK)
        {
            /* Get default configurations for the type of feature selected. */
            rslt = bmi323_get_sensor_config(config, 2, dev);
            bmi3_error_codes_print_result("Get sensor config", rslt);

            printf("Orientation wearable configurations\n");
            printf("Upside down: %d\n", config[1].cfg.orientation.ud_en);
            printf("Mode: %d\n", config[1].cfg.orientation.mode);
            printf("Blocking; %d\n", config[1].cfg.orientation.blocking);
            printf("Theta: %d\n", config[1].cfg.orientation.theta);
            printf("Hold_time: %d\n", config[1].cfg.orientation.hold_time);
            printf("Slope_thres: %d\n", config[1].cfg.orientation.slope_thres);
            printf("Hyst: %d\n", config[1].cfg.orientation.hysteresis);

            if (rslt == BMI323_OK)
            {
                /* Enable accel by selecting the mode. */
                config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

                /* Set new configurations. */
                rslt = bmi323_set_sensor_config(config, 2, dev);
                bmi3_error_codes_print_result("Set sensor config", rslt);
            }
        }
    }

    return rslt;
}
