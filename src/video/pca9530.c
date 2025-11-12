/*
 * Obtained from Embedded Artists who provided this under BSD license.
 * The Clear BSD License
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 */
#include "fsl_common.h"
#include "fsl_lpi2c.h"
#include "fsl_iomuxc.h"
#include "pca9530.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY (CLOCK_GetFreq(kCLOCK_OscRc48MDiv2))


#define PCA9530_I2C  LPI2C5

#define PCA9530_I2C_SLAVE_ADDR_7BIT 0x61

#define PCA9530_INPUT_REG    0x00
#define PCA9530_PSC0_REG     0x01
#define PCA9530_PWM0_REG     0x02
#define PCA9530_PSC1_REG     0x03
#define PCA9530_PWM1_REG     0x04
#define PCA9530_LED_SEL_REG  0x05

#define LED0_LS0_OUT_HIGH    (0<<0)
#define LED0_LS0_OUT_LOW     (1<<0)
#define LED0_LS0_SRC_PWM0    (2<<0)
#define LED0_LS0_SRC_PWM1    (3<<0)
#define LED0_LS0_MASK        (3<<0)

#define LED1_LS0_OUT_HIGH    (0<<2)
#define LED1_LS0_OUT_LOW     (1<<2)
#define LED1_LS0_SRC_PWM0    (2<<2)
#define LED1_LS0_SRC_PWM1    (3<<2)
#define LED1_LS0_MASK        (3<<2)

/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint8_t shadow = 0x00;
static bool already_initialized = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void init_pins(void)
{
  CLOCK_EnableClock(kCLOCK_Iomuxc_Lpsr);      /* LPCG on: LPCG is ON. */

  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,         /* GPIO_LPSR_04 is configured as LPI2C5_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_04 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_05 is configured as LPI2C5_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_05 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_04_LPI2C5_SDA,         /* GPIO_LPSR_04 PAD functional properties : */
      0x20U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Enabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_05_LPI2C5_SCL,         /* GPIO_LPSR_05 PAD functional properties : */
      0x20U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Enabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
}

static void i2c_init()
{
    lpi2c_master_config_t masterConfig = {0};

    /*
    * masterConfig.debugEnable = false;
    * masterConfig.ignoreAck = false;
    * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
    * masterConfig.baudRate_Hz = 100000U;
    * masterConfig.busIdleTimeout_ns = 0;
    * masterConfig.pinLowTimeout_ns = 0;
    * masterConfig.sdaGlitchFilterWidth_ns = 0;
    * masterConfig.sclGlitchFilterWidth_ns = 0;
    */
    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = 100000U;

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(PCA9530_I2C, &masterConfig, LPI2C_CLOCK_FREQUENCY);
}

/* Write to register. */
static status_t i2c_write(uint8_t reg, uint8_t val)
{
    lpi2c_master_transfer_t xfer = {0};
    uint8_t data[] = { val };

    xfer.slaveAddress = PCA9530_I2C_SLAVE_ADDR_7BIT;
    xfer.direction = kLPI2C_Write;
    xfer.subaddress = reg;
    xfer.subaddressSize = 1;
    xfer.data = data;
    xfer.dataSize = 1;
    xfer.flags = kLPI2C_TransferDefaultFlag;

    return LPI2C_MasterTransferBlocking(PCA9530_I2C, &xfer);
}

status_t PCA9530_Init(void)
{
    status_t status;

    if (already_initialized) {
        return kStatus_Success;
    }

    init_pins();
    i2c_init();

    // LEDs are HIGH-impedance (i.e. LEDs are OFF)
    status = i2c_write(PCA9530_LED_SEL_REG, 0x00);
    if (status != kStatus_Success) {
        return status;
    }
    shadow = 0x00;
    already_initialized = true;
    return status;
}

status_t PCA9530_ClearPins(uint8_t mask)
{
    status_t res;
    uint8_t val;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA9530_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    val = shadow;
    if (mask & PCA9530_LED0) {
        val = (val & ~LED0_LS0_MASK) | LED0_LS0_OUT_LOW;
    }
    if (mask & PCA9530_LED1) {
        val = (val & ~LED1_LS0_MASK) | LED1_LS0_OUT_LOW;
    }
    res = i2c_write(PCA9530_LED_SEL_REG, val);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow = val;
    }
    return res;
}
status_t PCA9530_SetPins(uint8_t mask)
{
    status_t res;
    uint8_t val;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA9530_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    val = shadow;
    if (mask & PCA9530_LED0) {
        val = (val & ~LED0_LS0_MASK) | LED0_LS0_OUT_HIGH;
    }
    if (mask & PCA9530_LED1) {
        val = (val & ~LED1_LS0_MASK) | LED1_LS0_OUT_HIGH;
    }
    res = i2c_write(PCA9530_LED_SEL_REG, val);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow = val;
    }
    return res;
}

status_t PCA9530_TogglePins(uint8_t mask)
{
    status_t res;
    uint8_t val;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA9530_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    val = shadow;
    if (mask & PCA9530_LED0) {
         switch (val & LED0_LS0_MASK) {
         case LED0_LS0_OUT_HIGH:
            val = (val & ~LED0_LS0_MASK) | LED0_LS0_OUT_LOW;
            break;
         case LED0_LS0_OUT_LOW:
            val = (val & ~LED0_LS0_MASK) | LED0_LS0_OUT_HIGH;
            break;
         case LED0_LS0_SRC_PWM0:
         case LED0_LS0_SRC_PWM1:
              // Cannot toggle when in PWM mode
              return kStatus_Fail;
         }
    }
    if (mask & PCA9530_LED1) {
         switch (val & LED1_LS0_MASK) {
         case LED1_LS0_OUT_HIGH:
            val = (val & ~LED1_LS0_MASK) | LED1_LS0_OUT_LOW;
            break;
         case LED1_LS0_OUT_LOW:
            val = (val & ~LED1_LS0_MASK) | LED1_LS0_OUT_HIGH;
            break;
         case LED1_LS0_SRC_PWM0:
         case LED1_LS0_SRC_PWM1:
              // Cannot toggle when in PWM mode
              return kStatus_Fail;
         }
    }
    res = i2c_write(PCA9530_LED_SEL_REG, val);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow = val;
    }
    return res;
}
