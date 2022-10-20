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
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_config(struct bmi3_dev *dev);

/*!
 * @brief This internal API is to copy contents from sensor data to FOC data.
 *
 * @param[in] acc_foc_data        : Pointer variable to store accel FOC data
 * @param[in] acc_sensor_data     : Pointer variable to store accel sensor data
 * @param[in] size                : Variable to store size of structure
 *
 * @return None
 *
 * @retval None
 */
static void memcopy(void *acc_foc_data, void *acc_sensor_data, int8_t size);

/*!
 * @brief This internal API is to clear the FOC data
 *
 * @param[in] generic_void_pntr   : Void pointer to store FOC data
 * @param[in] number              : Number to fill the array
 * @param[in] limit               : Fills the elements/blocks for given limit
 *
 * @return None
 *
 * @retval None
 */
static void memset_function(void* generic_void_pntr, uint8_t number, size_t limit);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Structure to store temporary accel FOC values */
    struct bmi3_foc_temp_value temp_foc_data = { 0 };

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Set accel FOC axis and it's sign (x, y, z, sign) */
    struct bmi3_accel_foc_g_value g_value_foc = { 0, 0, 1, 0 };

    /* Create an instance of sensor data structure. */
    struct bmi3_sensor_data sens_data = { 0 };

    /* Structure instance of accel foc */
    struct bmi3_sens_axes_data accel_foc_data[20] = { { 0 } };

    /* Structure instance of accel user gain offset */
    struct bmi3_acc_usr_gain_offset usr_gain_offset = { 0 };

    /* Structure instance of accel dp gain offset */
    struct bmi3_acc_dp_gain_offset acc_dp_gain_offset = { 0 };

    /* Variable to store result of API */
    int8_t rslt;

    /* Variable to loop */
    uint8_t indx;

    /* Limit to print accel FOC data */
    uint16_t limit = 20;

    /* Assign accel sensor. */
    sens_data.type = BMI323_ACCEL;

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
            rslt = bmi323_configure_enhanced_flexibility(&dev);
            bmi3_error_codes_print_result("bmi323_configure_enhanced_flexibility", rslt);

            /* Accel configuration settings. */
            rslt = set_accel_config(&dev);
            bmi3_error_codes_print_result("set_accel_config", rslt);

            printf("Do not move the board while performing foc\n");

            printf("\n***** BEFORE ACCEL FOC *****\n");

            printf("\nData set, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z\n\n");

            /* Read 20 samples of accelerometer values before FOC */
            for (indx = 0; indx < limit; indx++)
            {
                /* Giving a delay of 20ms before reading accelerometer data */
                dev.delay_us(20000, dev.intf_ptr);

                /* Get accelerometer data for x, y and z axis. */
                rslt = bmi323_get_sensor_data(&sens_data, 1, &dev);
                bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

                memcopy(&accel_foc_data[indx], &sens_data.sens_data.acc, sizeof(struct bmi3_sens_axes_data));
                printf("%d, %5d, %5d, %5d\n",
                       indx,
                       accel_foc_data[indx].x,
                       accel_foc_data[indx].y,
                       accel_foc_data[indx].z);

                temp_foc_data.x += accel_foc_data[indx].x;
                temp_foc_data.y += accel_foc_data[indx].y;
                temp_foc_data.z += accel_foc_data[indx].z;
            }

            /* Perform accel FOC. */
            rslt = bmi323_perform_accel_foc(&g_value_foc, &dev);
            bmi3_error_codes_print_result("bmi323_perform_accel_foc", rslt);

            if (rslt == BMI323_OK)
            {
                memset_function(accel_foc_data, 0, (20 * sizeof(struct bmi3_sens_axes_data)));
                memset_function(&temp_foc_data, 0, sizeof(struct bmi3_foc_temp_value));

                printf("\n***** AFTER ACCEL FOC *****\n");

                printf("\nData set, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z\n\n");

                /* Read accelerometer values after FOC */
                for (indx = 0; indx < 20; indx++)
                {
                    /* Giving a delay of 20ms before reading accelerometer data */
                    dev.delay_us(20000, dev.intf_ptr);

                    /* Get accelerometer data for x, y and z axis. */
                    rslt = bmi323_get_sensor_data(&sens_data, 1, &dev);
                    bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

                    memcopy(&accel_foc_data[indx], &sens_data.sens_data.acc, sizeof(struct bmi3_sens_axes_data));

                    printf("%d, %5d, %5d, %5d\n", indx, accel_foc_data[indx].x, accel_foc_data[indx].y,
                           accel_foc_data[indx].z);

                    temp_foc_data.x += accel_foc_data[indx].x;
                    temp_foc_data.y += accel_foc_data[indx].y;
                    temp_foc_data.z += accel_foc_data[indx].z;
                }
            }
        }
    }

    if (rslt == BMI323_OK)
    {
        /* Get user offset gain */
        rslt = bmi323_get_user_acc_off_dgain(&usr_gain_offset, &dev);
        bmi3_error_codes_print_result("bmi323_get_user_acc_off_dgain", rslt);

        if (rslt == BMI323_OK)
        {
            printf("\n******Accel user offest gain values******\n");
            printf("Value of acc_usr_off_x is %d\n", usr_gain_offset.acc_usr_off_x);
            printf("Value of acc_usr_off_y is %d\n", usr_gain_offset.acc_usr_off_y);
            printf("Value of acc_usr_off_z is %d\n", usr_gain_offset.acc_usr_off_z);

            printf("Value of acc_usr_gain_x is %d\n", usr_gain_offset.acc_usr_gain_x);
            printf("Value of acc_usr_gain_y is %d\n", usr_gain_offset.acc_usr_gain_y);
            printf("Value of acc_usr_gain_z is %d\n", usr_gain_offset.acc_usr_gain_z);

            /* Get accel dp offset */
            rslt = bmi323_get_acc_dp_off_dgain(&acc_dp_gain_offset, &dev);
            bmi3_error_codes_print_result("bmi3_get_acc_off_dgain", rslt);

            printf("\n******Accel dp offest gain values******\n");

            printf("Value of acc_dp_off_x is %d\n", acc_dp_gain_offset.acc_dp_off_x);
            printf("Value of acc_dp_off_y is %d\n", acc_dp_gain_offset.acc_dp_off_y);
            printf("Value of acc_dp_off_z is %d\n", acc_dp_gain_offset.acc_dp_off_z);

            printf("Value of acc_dp_gain_x is %d\n", acc_dp_gain_offset.acc_dp_gain_x);
            printf("Value of acc_dp_gain_y is %d\n", acc_dp_gain_offset.acc_dp_gain_y);
            printf("Value of acc_dp_gain_z is %d\n", acc_dp_gain_offset.acc_dp_gain_z);
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI323_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    if (rslt == BMI323_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Output Data Rate. By default ODR is set as 100Hz for accel. */
        config.cfg.acc.odr = BMI3_ACC_ODR_100HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config.cfg.acc.range = BMI3_ACC_RANGE_2G;

        /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
        config.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

        /* Set number of average samples for accel. */
        config.cfg.acc.avg_num = BMI3_ACC_AVG64;

        /* Enable the accel mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * Note : By default accel is disabled. The accel will get enable by selecting the mode.
         */
        config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

        /* Set the accel configurations. */
        rslt = bmi323_set_sensor_config(&config, 1, dev);
        bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
    }

    return rslt;
}

/*!
 *  @brief This internal API is to copy contents from sensor data to FOC data.
 */
static void memcopy(void *acc_foc_data, void *acc_sensor_data, int8_t size)
{
    /* Typecast gyr_sensor_data and acc_foc_data to (int8_t *). */
    int8_t *sensor_data = (int8_t *)acc_sensor_data;
    int8_t *foc_data = (int8_t *)acc_foc_data;

    /* Loop to copy the contents from sensor_data to foc_data. */
    for (int8_t indx = 0; indx < size; indx++)
    {
        foc_data[indx] = sensor_data[indx];
    }
}

/*!
 *  @brief This internal API is to clear the FOC data.
 */
static void memset_function(void* generic_void_pntr, uint8_t number, size_t limit)
{
    uint16_t indx;

    /* Type cast the string from void* to uint8_t* */
    uint8_t *ptr_data_u8 = (uint8_t*) generic_void_pntr;

    /* Fill elements/blocks within the limits with number in second argument */
    for (indx = 0; indx < limit; indx++)
    {
        ptr_data_u8[indx] = number;
    }
}
