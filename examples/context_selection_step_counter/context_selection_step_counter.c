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
 *  @brief This internal API is used to set configurations for step counter interrupt.
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

    /* Variable to get step counter interrupt status. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    /* Feature enable initialization. */
    struct bmi323_feature_enable feature = { 0 };

    /* Interrupt mapping structure. */
    struct bmi323_map_int map_int = { 0 };

    /* Structure to store sensor data. */
    struct bmi323_sensor_data sensor_data = { 0 };

    /* Sensor type of sensor to get data. */
    sensor_data.type = BMI323_STEP_COUNTER;

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
            /* Set feature configurations for step counter interrupt. */
            rslt = set_feature_config(&dev);
            bmi323_error_codes_print_result("Set feature config", rslt);

            if (rslt == BMI323_OK)
            {
                feature.step_counter_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi323_error_codes_print_result("Sensor enable", rslt);

                if (rslt == BMI323_OK)
                {
                    map_int.step_counter_out = BMI323_INT1;

                    /* Map the feature interrupt for step counter. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi323_error_codes_print_result("Map interrupt", rslt);
                    printf("Move the board in steps\n");

                    /* Loop to get step counter interrupt. */
                    do
                    {
                        /* Clear buffer. */
                        int_status = 0;

                        /* To get the interrupt status of step counter. */
                        rslt = bmi323_get_int1_status(&int_status, &dev);
                        bmi323_error_codes_print_result("Get interrupt status", rslt);

                        /* To check the interrupt status of step counter. */
                        if (int_status & BMI323_INT_STATUS_STEP_COUNTER)
                        {
                            printf("Step counter interrupt is generated\n");

                            /* Get step counter output */
                            rslt = bmi323_get_sensor_data(&sensor_data, 1, &dev);
                            bmi323_error_codes_print_result("Get interrupt status", rslt);

                            /* Print the step counter output */
                            printf("No of steps counted  = %ld\n", sensor_data.sens_data.step_counter_output);
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
 * @brief This internal API is used to set configurations for step counter interrupt.
 */
static int8_t set_feature_config(struct bmi323_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi323_sens_config config[2];

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_STEP_COUNTER;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi323_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Function to select the context feature configurations.
         * For smart phone: BMI323_SMART_PHONE_SEL
         * For wearables  : BMI323_WEARABLE_SEL
         * For hearables  : BMI323_HEARABLE_SEL */
        rslt = bmi323_context_switch_selection(BMI323_SMART_PHONE_SEL, dev);
        bmi323_error_codes_print_result("context switch selection", rslt);

        if (rslt == BMI323_OK)
        {
            /* Get default configurations for the type of feature selected. */
            rslt = bmi323_get_sensor_config(config, 2, dev);
            bmi323_error_codes_print_result("Get sensor config", rslt);

            printf("Step counter smart phone configurations\n");
            printf("Watermark level = %x\n", config[1].cfg.step_counter.watermark_level);
            printf("Reset counter = %x\n", config[1].cfg.step_counter.reset_counter);
            printf("env_min_dist_up: %d\n", config[1].cfg.step_counter.env_min_dist_up);
            printf("env_coef_up: %d\n", config[1].cfg.step_counter.env_coef_up);
            printf("env_min_dist_down: %d\n", config[1].cfg.step_counter.env_min_dist_down);
            printf("env_coef_down: %d\n", config[1].cfg.step_counter.env_coef_down);
            printf("eean_val_decay: %d\n", config[1].cfg.step_counter.mean_val_decay);
            printf("eean_step_dur: %d\n", config[1].cfg.step_counter.mean_step_dur);
            printf("step_buffer_size: %d\n", config[1].cfg.step_counter.step_buffer_size);
            printf("filter_cascade_enabled: %d\n", config[1].cfg.step_counter.filter_cascade_enabled);
            printf("step_counter_increment: %d\n", config[1].cfg.step_counter.step_counter_increment);
            printf("peak_duration_min_walking: %d\n", config[1].cfg.step_counter.peak_duration_min_walking);
            printf("peak_duration_min_running: %d\n", config[1].cfg.step_counter.peak_duration_min_running);
            printf("activity_detection_factor: %d\n", config[1].cfg.step_counter.activity_detection_factor);
            printf("activity_detection_thres: %d\n", config[1].cfg.step_counter.activity_detection_thres);
            printf("step_duration_max: %d\n", config[1].cfg.step_counter.step_duration_max);
            printf("step_duration_window: %d\n", config[1].cfg.step_counter.step_duration_window);
            printf("step_duration_pp_enabled; %d\n", config[1].cfg.step_counter.step_duration_pp_enabled);
            printf("step_duration_thres: %d\n", config[1].cfg.step_counter.step_duration_thres);
            printf("mean_crossing_pp_enabled: %d\n", config[1].cfg.step_counter.mean_crossing_pp_enabled);
            printf("mcr_threshold: %d\n", config[1].cfg.step_counter.mcr_threshold);
            printf("device_context: %d\n", config[1].cfg.step_counter.device_context);

            if (rslt == BMI323_OK)
            {
                /* Enable accel by selecting the mode. */
                config[0].cfg.acc.acc_mode = BMI323_ACC_MODE_NORMAL;

                /* Enable water-mark level for to get interrupt after 20 step counts. */
                config[1].cfg.step_counter.watermark_level = 1;

                /* Set new configurations. */
                rslt = bmi323_set_sensor_config(config, 2, dev);
                bmi323_error_codes_print_result("Set sensor config", rslt);

                if (rslt == BMI323_OK)
                {
                    /* Get default configurations for the type of feature selected. */
                    rslt = bmi323_get_sensor_config(config, 2, dev);
                    bmi323_error_codes_print_result("Get sensor config", rslt);

                    printf("Step counter watermark level is %d\n", config[1].cfg.step_counter.watermark_level);
                }
            }
        }
    }

    return rslt;
}
