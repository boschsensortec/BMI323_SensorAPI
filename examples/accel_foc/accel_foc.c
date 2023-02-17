/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "bmi323.h"
#include "common.h"
#include "coines.h"

/******************************************************************************/
/*!                   Macro Definitions                                       */

#define ACCEL_SAMPLE_COUNT  UINT8_C(100)

/******************************************************************************/
/*!         Global Variable Declaration                                       */

/*! Structure to store temporary axes data values */
struct temp_axes_val
{
    /* X data */
    int32_t x;

    /* Y data */
    int32_t y;

    /* Z data */
    int32_t z;
};

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used perform accel foc and determine limits based on range
 *
 *  @param[in] range              : Range of Accel
 *  @param[in] g_value_foc        : Structure instance of bmi3_accel_foc_g_value.
 *  @param[in,out] bmi3_dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t perform_foc_range_test(uint8_t range, struct bmi3_accel_foc_g_value g_value_foc,
                                     struct bmi3_dev *bmi3_dev);

/*!
 *  @brief This internal API is to determine if average accel FOC data is within limits
 *
 *  @param[in] avg_accel_foc_data      : Average Accel FOC value
 *  @param[in] reference               : Reference LSB based on Accel Range
 *  @param[in] foc_sign                : Input sign of performed Accel FOC
 *  @param[in] min_val                 : Minimum acceptable LSB limit
 *  @param[in] max_val                 : Maximum acceptable LSB limit
 *
 *  @return Status of execution.
 */
static int8_t accel_foc_report(int16_t avg_accel_foc_data,
                               int16_t reference,
                               uint8_t foc_sign,
                               int16_t min_val,
                               int16_t max_val);

/*!
 *  @brief This internal API is to collect and verify accel sensor data
 *
 *  @param[in] range                   : Value of Accel range
 *  @param[in] reference               : Reference LSB based on Accel Range
 *  @param[in] matched_axis            : Input Axis to perform Accel FOC
 *  @param[in] foc_sign                : Input sign to perform Accel FOC
 *  @param[in,out] bmi3_dev            : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t verify_accel_foc_data(uint8_t range,
                                    int16_t reference,
                                    int8_t matched_axis,
                                    uint8_t foc_sign,
                                    struct bmi3_dev *bmi3_dev);

/*!
 *  @brief This internal API is to calculate noise level for Accel FOC data
 *
 *  @param[in] matched_axis            : Input Axis to perform accel FOC
 *  @param[in] accel_foc_data          : Array of Accel FOC data
 *  @param[in] avg_accel_foc_data      : Average Accel FOC data
 *
 *  @return Status of execution.
 */
static void calculate_noise(int8_t matched_axis,
                            const struct bmi3_sens_axes_data *accel_foc_data,
                            struct bmi3_sens_axes_data avg_accel_foc_data);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi3_dev dev;

    uint8_t try = 0, j;
    int8_t rslt;
    struct bmi3_sens_config sens_cfg = { 0 };
    uint8_t range, input_axis = 0;

    /* Set accel foc axis and it's sign (x, y, z, sign) */
    struct bmi3_accel_foc_g_value g_value_foc = { 0, 0, 0, 0 };

    /* Structure instance of accel dp gain offset */
    struct bmi3_acc_dp_gain_offset acc_dp_gain_offset = { 0 };

    /* Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    printf("Functional test for accel foc start..\n\n");

    printf("Choose the axis for accel FOC to be done\n");
    printf("Press '1' to choose X axis\n");
    printf("Press '2' to choose Y axis\n");
    printf("Press '3' to choose Z axis\n");

    printf("Press '4' to choose -X axis\n");
    printf("Press '5' to choose -Y axis\n");
    printf("Press '6' to choose -Z axis\n");

    for (;;)
    {
        scanf("%hu", (short unsigned int *)&input_axis);
        if (input_axis > 0 && input_axis < 7)
        {
            break;
        }
    }

    if (input_axis == 1)
    {
        printf("The chosen input axis for FOC is : X\n");

        g_value_foc.x = 1;
        g_value_foc.y = 0;
        g_value_foc.z = 0;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 2)
    {
        printf("The chosen input axis for FOC is : Y\n");

        g_value_foc.x = 0;
        g_value_foc.y = 1;
        g_value_foc.z = 0;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 3)
    {
        printf("The chosen input axis for FOC is : Z\n");

        g_value_foc.x = 0;
        g_value_foc.y = 0;
        g_value_foc.z = 1;
        g_value_foc.sign = 0;
    }
    else if (input_axis == 4)
    {
        printf("The chosen input axis for FOC is : -X\n");

        g_value_foc.x = 1;
        g_value_foc.y = 0;
        g_value_foc.z = 0;
        g_value_foc.sign = 1;
    }
    else if (input_axis == 5)
    {
        printf("The chosen input axis for FOC is : -Y\n");

        g_value_foc.x = 0;
        g_value_foc.y = 1;
        g_value_foc.z = 0;
        g_value_foc.sign = 1;
    }
    else if (input_axis == 6)
    {
        printf("The chosen input axis for FOC is : -Z\n");

        g_value_foc.x = 0;
        g_value_foc.y = 0;
        g_value_foc.z = 1;
        g_value_foc.sign = 1;
    }

    printf("Confirm your chosen axis and the sensor keeping position are same before doing FOC\n");

    for (j = 0; j < 2; j++)
    {
        try = 0;

        if (j == 1)
        {
            printf("Keep sensor in wrong position and press 5\n");
        }
        else if (j == 0)
        {
            printf("Keep sensor in right position and press 5\n");
        }

        for (;;)
        {
            scanf("%hu", (short unsigned int *)&try);
            if (try == 5)
            {
                break;
            }
        }

        for (range = BMI3_ACC_RANGE_2G; range <= BMI3_ACC_RANGE_16G; range++)
        {
            /****************************************************************/
            /* Initialize by enabling configuration load */
            printf("#########################################################\n\n");

            rslt = bmi323_init(&dev);
            bmi3_error_codes_print_result("bmi323_init", rslt);

            sens_cfg.type = BMI3_ACCEL;

            /* Testing with different settings other than the default configurations
             *  Default is : 50Hz ODR and 2g RANGE
             *  Note - Change accel_conf.range for testing in different range values
             */
            sens_cfg.cfg.acc.odr = BMI3_ACC_ODR_50HZ;
            sens_cfg.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
            sens_cfg.cfg.acc.avg_num = BMI3_ACC_AVG4;

            /****************************************************************/
            sens_cfg.cfg.acc.range = range;

            /* Set the configuration details */
            rslt = bmi323_set_sensor_config(&sens_cfg, 1, &dev);
            bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);

            sens_cfg.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Set the configuration details */
            rslt = bmi323_set_sensor_config(&sens_cfg, 1, &dev);
            bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);

            /* Get the configuration details to verify whether the configured values are set */
            rslt = bmi323_get_sensor_config(&sens_cfg, 1, &dev);
            bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

            printf("ODR = %d, RANGE = %d, BANDWIDTH = %d\n",
                   sens_cfg.cfg.acc.odr,
                   sens_cfg.cfg.acc.range,
                   sens_cfg.cfg.acc.bwp);

            /* Perform FOC for different ranges */
            rslt = perform_foc_range_test(range, g_value_foc, &dev);

            if ((j == 1) && (rslt == BMI3_E_OUT_OF_RANGE))
            {
                printf("\n#########   Valid input - Wrong position   #########\n\n");
                bmi3_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 0) && (rslt == BMI3_OK))
            {
                printf("\n#########   Valid input - Right position   #########\n\n");
                bmi3_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 1) && (rslt == BMI3_OK))
            {
                printf("\n#########   Invalid input - Right position   #########\n\n");
                bmi3_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 0) && (rslt == BMI3_E_OUT_OF_RANGE))
            {
                printf("\n#########   Invalid input - Wrong position   #########\n\n");
                bmi3_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 0) && (rslt == BMI3_E_OUT_OF_RANGE))
            {
                printf("\n#########   Valid input - Right position   #########\n\n");
                printf("\n#########   Before FOC is better than after FOC   #########\n\n");
                bmi3_error_codes_print_result("perform_foc_range_test", rslt);
            }
            else if ((j == 1) && (rslt == BMI3_E_OUT_OF_RANGE))
            {
                printf("\n#########   Invalid input - Right position   #########\n\n");
                printf("\n#########   Before FOC is better than after FOC   #########\n\n");
                bmi3_error_codes_print_result("perform_foc_range_test", rslt);
            }

            if (j == 0)
            {
                /* Get accel dp offset */
                rslt = bmi323_get_acc_dp_off_dgain(&acc_dp_gain_offset, &dev);
                bmi3_error_codes_print_result("bmi323_get_acc_dp_off_dgain", rslt);

                printf("\n******Accel dp offest gain values******\n");

                printf("Value of acc_dp_off_x is %d\n", acc_dp_gain_offset.acc_dp_off_x);
                printf("Value of acc_dp_off_y is %d\n", acc_dp_gain_offset.acc_dp_off_y);
                printf("Value of acc_dp_off_z is %d\n", acc_dp_gain_offset.acc_dp_off_z);

                printf("Value of acc_dp_gain_x is %d\n", acc_dp_gain_offset.acc_dp_dgain_x);
                printf("Value of acc_dp_gain_y is %d\n", acc_dp_gain_offset.acc_dp_dgain_y);
                printf("Value of acc_dp_gain_z is %d\n", acc_dp_gain_offset.acc_dp_dgain_z);
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

static int8_t accel_foc_report(int16_t avg_accel_foc_data,
                               int16_t reference,
                               uint8_t foc_sign,
                               int16_t min_val,
                               int16_t max_val)
{
    int8_t rslt = BMI3_OK;
    int16_t diff_after = 0;

    if (foc_sign == 0)
    {
        if ((avg_accel_foc_data >= (min_val)) && (avg_accel_foc_data <= (max_val)))
        {
            if (avg_accel_foc_data >= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** PASS | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d in range\n", avg_accel_foc_data);
            rslt = BMI3_OK;
        }
        else
        {
            if (avg_accel_foc_data >= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** FAIL | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d not in range\n", avg_accel_foc_data);
            rslt = BMI3_E_OUT_OF_RANGE;
        }
    }

    if (foc_sign == 1)
    {
        if ((avg_accel_foc_data <= (min_val)) && (avg_accel_foc_data >= (max_val)))
        {
            if (avg_accel_foc_data <= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** PASS | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d in range\n", avg_accel_foc_data);
            rslt = BMI3_OK;
        }
        else
        {
            if (avg_accel_foc_data <= reference)
            {
                diff_after = avg_accel_foc_data - reference;
            }
            else
            {
                diff_after = reference - avg_accel_foc_data;
            }

            printf("\n# ********** FAIL | Difference = %d **********\n", diff_after);
            printf("\n# Avg_FOC %d not in range\n", avg_accel_foc_data);
            rslt = BMI3_E_OUT_OF_RANGE;
        }
    }

    return rslt;
}

static void calculate_noise(int8_t matched_axis,
                            const struct bmi3_sens_axes_data *accel_foc_data,
                            struct bmi3_sens_axes_data avg_accel_foc_data)
{
    uint32_t variance = 0;
    double noise_level;
    uint16_t indx = 0;

    if (matched_axis == 'X')
    {
        for (indx = 0; indx < ACCEL_SAMPLE_COUNT; indx++)
        {
            variance +=
                (uint32_t)((accel_foc_data[indx].x - avg_accel_foc_data.x) *
                           (accel_foc_data[indx].x - avg_accel_foc_data.x));
        }
    }
    else if (matched_axis == 'Y')
    {
        for (indx = 0; indx < ACCEL_SAMPLE_COUNT; indx++)
        {
            variance +=
                (uint32_t)((accel_foc_data[indx].y - avg_accel_foc_data.y) *
                           (accel_foc_data[indx].y - avg_accel_foc_data.y));
        }
    }
    else if (matched_axis == 'Z')
    {
        for (indx = 0; indx < ACCEL_SAMPLE_COUNT; indx++)
        {
            variance +=
                (uint32_t)((accel_foc_data[indx].z - avg_accel_foc_data.z) *
                           (accel_foc_data[indx].z - avg_accel_foc_data.z));
        }
    }

    noise_level = sqrt((double)variance);

    printf("\n# ********** NOISE LEVEL = %lf **********\n", noise_level);
}

static int8_t verify_accel_foc_data(uint8_t range,
                                    int16_t reference,
                                    int8_t matched_axis,
                                    uint8_t foc_sign,
                                    struct bmi3_dev *bmi3_dev)
{
    int8_t rslt;
    uint8_t i;
    int16_t xl, yl, zl;
    int16_t xh, yh, zh;
    int16_t min_val = 0;
    int16_t max_val = 0;
    struct bmi3_sens_axes_data accel_foc_data[ACCEL_SAMPLE_COUNT] = { { 0 } };
    struct temp_axes_val temp_foc_data = { 0 };
    struct bmi3_sens_axes_data avg_accel_foc_data = { 0 };
    struct bmi3_sensor_data sensor_data = { 0 };
    struct bmi3_map_int map_int = { 0 };

    /* Initialize the interrupt status of accel. */
    uint16_t int_status = 0;

    /* Setting initial values */
    xl = yl = zl = 32767;
    xh = yh = zh = -32768;

    /* Map data ready interrupt to interrupt pin. */
    map_int.acc_drdy_int = BMI3_INT1;

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi323_map_interrupt(map_int, bmi3_dev);
    bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

    /* Read accelerometer values before/after FOC */
    for (i = 0; i < ACCEL_SAMPLE_COUNT; i++)
    {
        for (;;)
        {
            /* To get the status of accel data ready interrupt. */
            rslt = bmi323_get_int1_status(&int_status, bmi3_dev);
            bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

            /* To check the accel data ready interrupt status and print the status for 100 samples. */
            if (int_status & BMI3_INT_STATUS_ACC_DRDY)
            {
                rslt = bmi323_get_sensor_data(&sensor_data, 1, bmi3_dev);
                bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

                memcpy(&accel_foc_data[i], &sensor_data.sens_data.acc, sizeof(struct bmi3_sens_axes_data));

                printf("X[%d] = %5d,  Y[%d] = %5d,  Z[%d] = %5d\n",
                       i,
                       accel_foc_data[i].x,
                       i,
                       accel_foc_data[i].y,
                       i,
                       accel_foc_data[i].z);

                if (xl > accel_foc_data[i].x)
                {
                    xl = accel_foc_data[i].x;
                }

                if (xh < accel_foc_data[i].x)
                {
                    xh = accel_foc_data[i].x;
                }

                if (yl > accel_foc_data[i].y)
                {
                    yl = accel_foc_data[i].y;
                }

                if (yh < accel_foc_data[i].y)
                {
                    yh = accel_foc_data[i].y;
                }

                if (zl > accel_foc_data[i].z)
                {
                    zl = accel_foc_data[i].z;
                }

                if (zh < accel_foc_data[i].z)
                {
                    zh = accel_foc_data[i].z;
                }

                temp_foc_data.x += accel_foc_data[i].x;
                temp_foc_data.y += accel_foc_data[i].y;
                temp_foc_data.z += accel_foc_data[i].z;

                break;
            }
        }
    }

    /* Taking average values to calculate percentage deviation */
    avg_accel_foc_data.x = (int16_t)(temp_foc_data.x / ACCEL_SAMPLE_COUNT);
    avg_accel_foc_data.y = (int16_t)(temp_foc_data.y / ACCEL_SAMPLE_COUNT);
    avg_accel_foc_data.z = (int16_t)(temp_foc_data.z / ACCEL_SAMPLE_COUNT);

    printf("********* MIN & MAX VALUES ********\n");

    printf("XL = %5d  YL = %5d  ZL = %5d\n", xl, yl, zl);
    printf("XH = %5d  YH = %5d  ZH = %5d\n", xh, yh, zh);

    printf("***** AVERAGE AFTER FOC *****\n");
    printf("Avg-X = %d        Avg-Y = %d        Avg-Z = %d\n",
           avg_accel_foc_data.x,
           avg_accel_foc_data.y,
           avg_accel_foc_data.z);

    /* Calculate noise level */
    calculate_noise(matched_axis, accel_foc_data, avg_accel_foc_data);

    /* "zero-g offset" of accel is +/- 20 mg for all ranges as per datasheet */
    if (range == 0)
    {
        /* Min and Max limits for Range 2G */
        min_val = BMI3_ACC_2G_MIN_NOISE_LIMIT;
        max_val = BMI3_ACC_2G_MAX_NOISE_LIMIT;
    }
    else if (range == 1)
    {
        /* Min and Max limits for Range 4G */
        min_val = BMI3_ACC_4G_MIN_NOISE_LIMIT;
        max_val = BMI3_ACC_4G_MAX_NOISE_LIMIT;
    }
    else if (range == 2)
    {
        /* Min and Max limits for Range 8G */
        min_val = BMI3_ACC_8G_MIN_NOISE_LIMIT;
        max_val = BMI3_ACC_8G_MAX_NOISE_LIMIT;
    }
    else if (range == 3)
    {
        /* Min and Max limits for Range 16G */
        min_val = BMI3_ACC_16G_MIN_NOISE_LIMIT;
        max_val = BMI3_ACC_16G_MAX_NOISE_LIMIT;
    }

    if ((matched_axis == 'X') && (foc_sign == 0))
    {
        rslt = accel_foc_report(avg_accel_foc_data.x, reference, foc_sign, min_val, max_val);

        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.x,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'Y') && (foc_sign == 0))
    {
        rslt = accel_foc_report(avg_accel_foc_data.y, reference, foc_sign, min_val, max_val);

        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.y,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'Z') && (foc_sign == 0))
    {
        rslt = accel_foc_report(avg_accel_foc_data.z, reference, foc_sign, min_val, max_val);

        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.z,
               reference,
               min_val,
               max_val);
    }
    else if ((matched_axis == 'X') && (foc_sign == 1))
    {
        rslt = accel_foc_report(avg_accel_foc_data.x,
                                (int16_t)(reference * (-1)),
                                foc_sign,
                                (int16_t)(min_val * (-1)),
                                (int16_t)(max_val * (-1)));

        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.x,
               (reference * (-1)),
               (min_val * (-1)),
               (max_val * (-1)));
    }
    else if ((matched_axis == 'Y') && (foc_sign == 1))
    {
        rslt = accel_foc_report(avg_accel_foc_data.y,
                                (int16_t)(reference * (-1)),
                                foc_sign,
                                (int16_t)(min_val * (-1)),
                                (int16_t)(max_val * (-1)));

        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.y,
               (reference * (-1)),
               (min_val * (-1)),
               (max_val * (-1)));
    }
    else if ((matched_axis == 'Z') && (foc_sign == 1))
    {
        rslt = accel_foc_report(avg_accel_foc_data.z,
                                (int16_t)(reference * (-1)),
                                foc_sign,
                                (int16_t)(min_val * (-1)),
                                (int16_t)(max_val * (-1)));

        printf("Range : %d  Avg_FOC-X : %d   Reference : %d   Min_Value : %d  Max_Value : %d\n",
               range,
               avg_accel_foc_data.z,
               (reference * (-1)),
               (min_val * (-1)),
               (max_val * (-1)));
    }

    return rslt;
}

/* Perform FOC for different range and resolutions */
static int8_t perform_foc_range_test(uint8_t range, struct bmi3_accel_foc_g_value g_value_foc,
                                     struct bmi3_dev *bmi3_dev)
{
    int8_t rslt;
    int8_t matched_axis = 0;
    int16_t reference = 0;

    switch (range)
    {
        /* Reference LSB value of 2G */
        case 0:
            reference = BMI3_ACC_FOC_2G_REF;
            break;

        /* Reference LSB value of 4G */
        case 1:
            reference = BMI3_ACC_FOC_4G_REF;
            break;

        /* Reference LSB value of 8G */
        case 2:
            reference = BMI3_ACC_FOC_8G_REF;
            break;

        /* Reference LSB value of 16G */
        case 3:
            reference = BMI3_ACC_FOC_16G_REF;
            break;

        default:
            break;
    }

    if (g_value_foc.x == 1)
    {
        matched_axis = 'X';
    }
    else if (g_value_foc.y == 1)
    {
        matched_axis = 'Y';
    }
    else if (g_value_foc.z == 1)
    {
        matched_axis = 'Z';
    }

    if (g_value_foc.sign == 1)
    {
        printf("MATCHED AXIS is = -%c\n", matched_axis);
    }
    else
    {
        printf("MATCHED AXIS is = %c\n", matched_axis);
    }

    printf("\n\n# Before FOC\n");
    (void)verify_accel_foc_data(range, reference, matched_axis, g_value_foc.sign, bmi3_dev);

    printf("\n\n######### Perform Accel FOC #########\n\n");

    /* Perform accelerometer FOC */
    rslt = bmi323_perform_accel_foc(&g_value_foc, bmi3_dev);
    bmi3_error_codes_print_result("bmi323_perform_accel_foc", rslt);

    /* Provide delay after performing FOC */
    bmi3_dev->delay_us(40000, bmi3_dev->intf_ptr);

    printf("\n\n# After FOC\n");
    rslt = verify_accel_foc_data(range, reference, matched_axis, g_value_foc.sign, bmi3_dev);

    return rslt;
}
