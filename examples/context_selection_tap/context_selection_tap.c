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
 *  @brief This internal API is used to set configurations for tap interrupt.
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

    uint8_t data[2];
    uint8_t loop = 10;

    /* Variable to get tap interrupt status. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Feature enable initialization. */
    struct bmi3_feature_enable feature = { 0 };

    /* Interrupt mapping structure. */
    struct bmi3_map_int map_int = { 0 };

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
            /* Set feature configurations for tap. */
            rslt = set_feature_config(&dev);
            bmi3_error_codes_print_result("Set feature config", rslt);

            if (rslt == BMI323_OK)
            {
                feature.tap_detector_s_tap_en = BMI323_ENABLE;
                feature.tap_detector_d_tap_en = BMI323_ENABLE;
                feature.tap_detector_t_tap_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi3_error_codes_print_result("Sensor enable", rslt);

                if (rslt == BMI323_OK)
                {
                    map_int.tap_out = BMI3_INT1;

                    /* Map the feature interrupt for tap. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi3_error_codes_print_result("Map interrupt", rslt);
                    printf("Tap the board either single, double or triple tap\n");

                    /* Loop to get tap interrupt. */
                    do
                    {
                        dev.delay_us(5000, dev.intf_ptr);

                        /* To get the interrupt status of tap. */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi3_error_codes_print_result("Get interrupt status", rslt);

                        /* Check the interrupt status of the tap. */
                        if (int_status & BMI3_INT_STATUS_TAP)
                        {
                            printf("Tap interrupt is generated\n");
                            rslt = bmi323_get_regs(BMI3_REG_FEATURE_EVENT_EXT, data, 2, &dev);

                            if (data[0] & BMI3_TAP_DET_STATUS_SINGLE)
                            {
                                printf("Single tap asserted\n");
                                loop--;
                            }

                            if (data[0] & BMI3_TAP_DET_STATUS_DOUBLE)
                            {
                                printf("Double tap asserted\n");
                                loop--;
                            }

                            if (data[0] & BMI3_TAP_DET_STATUS_TRIPLE)
                            {
                                printf("Triple tap asserted\n");
                                loop--;
                            }
                        }
                    } while (loop > 0);
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for tap interrupt.
 */
static int8_t set_feature_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi3_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_TAP;

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

            printf("Tap wearable configurations\n");
            printf("Axis_sel: %d\n", config[1].cfg.tap.axis_sel);
            printf("Wait_for_timeout: %d\n", config[1].cfg.tap.wait_for_timeout);
            printf("Max_peaks_for_tap: %d\n", config[1].cfg.tap.max_peaks_for_tap);
            printf("Mode: %d\n", config[1].cfg.tap.mode);
            printf("Tap_peak_thres: %d\n", config[1].cfg.tap.tap_peak_thres);
            printf("Max_gest_dur: %d\n", config[1].cfg.tap.max_gest_dur);
            printf("Max_dur_between_peaks: %d\n", config[1].cfg.tap.max_dur_between_peaks);
            printf("Tap_shock_settling_dur: %d\n", config[1].cfg.tap.tap_shock_settling_dur);
            printf("Min_quite_dur_between_taps: %d\n", config[1].cfg.tap.min_quite_dur_between_taps);
            printf("Quite_time_after_gest: %d\n", config[1].cfg.tap.quite_time_after_gest);

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
