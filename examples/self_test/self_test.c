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
/*!            Functions                                        */

/* This function starts the execution of program. */
int main(void)
{
    /* Variable to define error. */
    int8_t rslt;

    uint8_t idx;

    uint8_t limit = 3;

    /* Array to define self-test modes. */
    uint8_t self_test_selection[3] = { BMI323_ST_ACCEL_ONLY, BMI323_ST_GYRO_ONLY, BMI323_ST_BOTH_ACC_GYR };

    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI323_I2C_INTF
     * For SPI : BMI323_SPI_INTF
     */
    rslt = bmi323_interface_init(&dev, BMI323_SPI_INTF);
    bmi323_error_codes_print_result("Bmi3x0_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323 */
        rslt = bmi323_init(&dev);
        bmi323_error_codes_print_result("Bmi3x0_init", rslt);

        if (rslt == BMI323_OK)
        {
            for (idx = 0; idx < limit; idx++)
            {
                struct bmi323_acc_gyr_self_test_status st_result_status = { 0 };

                if ((idx + 1) == BMI323_ST_ACCEL_ONLY)
                {
                    printf("Self-test for accel only\n");
                }
                else if ((idx + 1) == BMI323_ST_GYRO_ONLY)
                {
                    printf("Self-test for gyro only\n");
                }
                else
                {
                    printf("Self-test for both accel and gyro\n");
                }

                /* Performs self-test for either accel, gyro or both */
                rslt = bmi323_perform_self_test(self_test_selection[idx], &st_result_status, &dev);
                bmi323_error_codes_print_result("Perform_self_test", rslt);

                if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
                {
                    switch (st_result_status.self_test_err_rslt)
                    {
                        case BMI323_SC_ST_ABORTED_MASK:
                            printf("SC_ST_ABORTED\n");
                            break;
                        case BMI323_ST_IGNORED_MASK:
                            printf("BMI323_ST_IGNORED\n");
                            break;
                        case BMI323_SC_ST_PRECON_ERR_MASK:
                            printf("BMI323_SC_ST_PRECON_ERR\n");
                            break;
                        case BMI323_MODE_CHANGE_WHILE_SC_ST_MASK:
                            printf("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
                            break;
                        default:
                            break;
                    }

                    printf("Result of acc_x_axis is %d\n", st_result_status.acc_x_ok);
                    printf("Result of acc_y_axis is %d\n", st_result_status.acc_y_ok);
                    printf("Result of acc_z_axis is %d\n", st_result_status.acc_z_ok);
                    printf("Result of gyr_x_axis is %d\n", st_result_status.gyr_x_ok);
                    printf("Result of gyr_y_axis is %d\n", st_result_status.gyr_y_ok);
                    printf("Result of gyr_z_axis is %d\n", st_result_status.gyr_z_ok);
                    printf("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
                    printf("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
                    printf("Result of ST_result is %d\n", st_result_status.self_test_rslt);
                }
            }
        }
    }

    return rslt;

    bmi323_coines_deinit();
}
