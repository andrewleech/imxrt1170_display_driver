/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_lpi2c.h"
#include "fsl_video_common.h"
#include "rpi_support.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY (CLOCK_GetFreq(kCLOCK_OscRc48MDiv2))
#define RPI_I2C_BASEADDR  LPI2C1

/*******************************************************************************
 * Code
 ******************************************************************************/

void talk_to_raspi(void)
{
    uint8_t tmp;
    uint8_t brightness = 0x80;

    BOARD_LPI2C_Init(RPI_I2C_BASEADDR, LPI2C_CLOCK_FREQUENCY);

    VIDEO_DelayMs(2000);

    //init_cmd_check
    //read_reg(RPI_ADDR, 0x80, &tmp);
    BOARD_LPI2C_Receive(RPI_I2C_BASEADDR, RPI_ADDR, 0x80, 1, &tmp, 1);
    // Optional: Add debug logging if needed
    // PRINTF("reg 0x80 is 0x%x\r\n", tmp);

    //rpi_display_screen_power_up
    //write_reg(RPI_ADDR, 0x85, 0x00);
    tmp = 0;
    BOARD_LPI2C_Send(RPI_I2C_BASEADDR, RPI_ADDR, 0x85, 1, &tmp, 1);
    VIDEO_DelayMs(800);
    //write_reg(RPI_ADDR, 0x85, 0x01);
    tmp = 0x01;
    BOARD_LPI2C_Send(RPI_I2C_BASEADDR, RPI_ADDR, 0x85, 1, &tmp, 1);
    VIDEO_DelayMs(800);
    //write_reg(RPI_ADDR, 0x81, 0x04);
    tmp = 0x04;
    BOARD_LPI2C_Send(RPI_I2C_BASEADDR, RPI_ADDR, 0x81, 1, &tmp, 1);

    //rpi_display_set_bright
    //write_reg(RPI_ADDR, 0x86, brightness);
    BOARD_LPI2C_Send(RPI_I2C_BASEADDR, RPI_ADDR, 0x86, 1, &brightness, 1);
}
