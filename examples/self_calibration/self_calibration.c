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
    /* Sensor initialization configuration. */
    struct bmi323_dev dev = { 0 };

    /* Structure to define the self-calibration result and error result */
    struct bmi323_self_calib_rslt sc_rslt;

    /* Variable to choose apply correction */
    uint8_t apply_corr = BMI323_SC_APPLY_CORR_EN;

    /* Variable to define error. */
    int8_t rslt;

    /* Variable to define index. */
    uint8_t idx;

    /* Variable to define limit. */
    uint8_t limit = 2;

    /* Array to define self-calibration modes. */
    uint8_t sc_selection[2] = { BMI323_SC_SENSITIVITY_EN, BMI323_SC_OFFSET_EN };

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
                if ((idx + 1) == BMI323_SC_SENSITIVITY_EN)
                {
                    printf("Self-calibration for sensitivity mode\n");
                }
                else
                {
                    printf("Self-calibration for offset mode\n");
                }

                /* Performs self-calibration for either sensitivity, offset or both */
                rslt = bmi323_perform_gyro_sc(sc_selection[idx], apply_corr, &sc_rslt, &dev);
                bmi323_error_codes_print_result("Perform self-calibration", rslt);

                if ((rslt == BMI323_OK) && (sc_rslt.gyro_sc_rslt == BMI323_TRUE))
                {
                    switch (sc_rslt.sc_error_rslt)
                    {
                        case BMI323_SC_ST_ABORTED_MASK:
                            printf("SC_ST_ABORTED\n");
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

                    printf("Result of self-test error is %d\n", sc_rslt.sc_error_rslt);
                    printf("Result of self-calibration is %d\n", sc_rslt.gyro_sc_rslt);
                }
            }
        }
    }

    return rslt;

    bmi323_coines_deinit();
}
