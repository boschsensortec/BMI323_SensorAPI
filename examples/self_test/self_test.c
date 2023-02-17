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
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Variable to define error. */
    int8_t rslt;

    uint8_t idx;

    uint8_t limit = 3;

    /* Array to define self-test modes. */
    uint8_t self_test_selection[3] = { BMI3_ST_ACCEL_ONLY, BMI3_ST_GYRO_ONLY, BMI3_ST_BOTH_ACC_GYR };

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    struct bmi3_st_result st_result_status = { 0 };

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323 */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            for (idx = 0; idx < limit; idx++)
            {
                if ((idx + 1) == BMI3_ST_ACCEL_ONLY)
                {
                    printf("Self-test for accel only\n");
                }
                else if ((idx + 1) == BMI3_ST_GYRO_ONLY)
                {
                    printf("\n\nSelf-test for gyro only\n");
                }
                else
                {
                    printf("\n\nSelf-test for both accel and gyro\n");
                }

                /* Performs self-test for either accel, gyro or both */
                rslt = bmi323_perform_self_test(self_test_selection[idx], &st_result_status, &dev);
                bmi3_error_codes_print_result("Perform_self_test", rslt);

                if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
                {
                    printf("Self-test is successfully completed \n");
                }

                if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
                {
                    printf("Self-test is not successfully completed\n");

                    switch (st_result_status.self_test_err_rslt)
                    {
                        case BMI3_SC_ST_ABORTED_MASK:
                            printf("SC_ST_ABORTED\n");
                            break;
                        case BMI3_ST_IGNORED_MASK:
                            printf("BMI323_ST_IGNORED\n");
                            break;
                        case BMI3_SC_ST_PRECON_ERR_MASK:
                            printf("BMI323_SC_ST_PRECON_ERR\n");
                            break;
                        case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
                            printf("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
                            break;
                        default:
                            break;
                    }
                }

                printf("Result of acc_x_axis is %d\n", st_result_status.acc_sens_x_ok);
                printf("Result of acc_y_axis is %d\n", st_result_status.acc_sens_y_ok);
                printf("Result of acc_z_axis is %d\n", st_result_status.acc_sens_z_ok);
                printf("Result of gyr_x_axis is %d\n", st_result_status.gyr_sens_x_ok);
                printf("Result of gyr_y_axis is %d\n", st_result_status.gyr_sens_y_ok);
                printf("Result of gyr_z_axis is %d\n", st_result_status.gyr_sens_z_ok);
                printf("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
                printf("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
                printf("Result of ST_result is %d\n", st_result_status.self_test_rslt);
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}
