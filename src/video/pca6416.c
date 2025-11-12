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
#include "pca6416.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY (CLOCK_GetFreq(kCLOCK_OscRc48MDiv2))


#define PCA6416_I2C  LPI2C6

#define PCA6416_I2C_SLAVE_ADDR_7BIT 0x21

#define PCA6416_INPUT_PORT0_REG   0x00
#define PCA6416_INPUT_PORT1_REG   0x01
#define PCA6416_OUTPUT_PORT0_REG  0x02
#define PCA6416_OUTPUT_PORT1_REG  0x03
#define PCA6416_POLINV_PORT0_REG  0x04
#define PCA6416_POLINV_PORT1_REG  0x05
#define PCA6416_CONFIG_PORT0_REG  0x06
#define PCA6416_CONFIG_PORT1_REG  0x07


/*******************************************************************************
 * Variables
 ******************************************************************************/

static uint16_t shadow = 0x0000;
static bool already_initialized = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

static void init_pins(void)
{
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_06_LPI2C6_SDA,         /* GPIO_LPSR_06 is configured as LPI2C6_SDA */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_06 */
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_LPSR_07_LPI2C6_SCL,         /* GPIO_LPSR_07 is configured as LPI2C6_SCL */
      1U);                                    /* Software Input On Field: Force input path of pad GPIO_LPSR_07 */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_06_LPI2C6_SDA,         /* GPIO_LPSR_06 PAD functional properties : */
      0x20U);                                 /* Slew Rate Field: Slow Slew Rate
                                                 Drive Strength Field: normal driver
                                                 Pull / Keep Select Field: Pull Disable
                                                 Pull Up / Down Config. Field: Weak pull down
                                                 Open Drain LPSR Field: Enabled
                                                 Domain write protection: Both cores are allowed
                                                 Domain write protection lock: Neither of DWP bits is locked */
  IOMUXC_SetPinConfig(
      IOMUXC_GPIO_LPSR_07_LPI2C6_SCL,         /* GPIO_LPSR_07 PAD functional properties : */
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
    LPI2C_MasterInit(PCA6416_I2C, &masterConfig, LPI2C_CLOCK_FREQUENCY);
}

/* Write to both port registers. */
static status_t i2c_write(uint8_t port0reg, uint16_t val)
{
    lpi2c_master_transfer_t xfer = {0};
    uint8_t data[] = { val&0xff, (val>>8) };

    xfer.slaveAddress = PCA6416_I2C_SLAVE_ADDR_7BIT;
    xfer.direction = kLPI2C_Write;
    xfer.subaddress = port0reg;
    xfer.subaddressSize = 1;
    xfer.data = data;
    xfer.dataSize = 2;
    xfer.flags = kLPI2C_TransferDefaultFlag;

    return LPI2C_MasterTransferBlocking(PCA6416_I2C, &xfer);
}

/* Read from both port registers. */
static status_t i2c_read(uint8_t port0reg, uint16_t* val)
{
    lpi2c_master_transfer_t xfer = {0};
    uint8_t data[] = { 0,0 };
    status_t res;

    xfer.slaveAddress = PCA6416_I2C_SLAVE_ADDR_7BIT;
    xfer.direction = kLPI2C_Read;
    xfer.subaddress = port0reg;
    xfer.subaddressSize = 1;
    xfer.data = data;
    xfer.dataSize = 2;
    xfer.flags = kLPI2C_TransferDefaultFlag;

    res = LPI2C_MasterTransferBlocking(PCA6416_I2C, &xfer);
    if (res == kStatus_Success && val != NULL) {
        *val = data[0] | (data[1] << 8);
    }
    return res;
}

status_t PCA6416_Init(void)
{
    status_t status;

    if (already_initialized) {
        return kStatus_Success;
    }

    init_pins();
    i2c_init();

    // LEDs are HIGH outputs, i.e. LEDs are OFF
    // BUTTON is an input
    // All other pins are LOW outputs
    status = i2c_write(PCA6416_OUTPUT_PORT0_REG, PCA_ALL_LEDS);
    if (status != kStatus_Success) {
        return status;
    }
    status = i2c_write(PCA6416_CONFIG_PORT0_REG, PCA_USER_BUTTON);
    if (status != kStatus_Success) {
        return status;
    }
    shadow = PCA_ALL_LEDS;
    already_initialized = true;
    return status;
}

status_t PCA6416_SetAllPins(uint16_t values)
{
    status_t res;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA6416_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    res = i2c_write(PCA6416_OUTPUT_PORT0_REG, values);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow = values;
    }
    return res;
}
status_t PCA6416_ClearPins(uint16_t mask)
{
    status_t res;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA6416_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    res = i2c_write(PCA6416_OUTPUT_PORT0_REG, shadow & ~mask);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow &= ~mask;
    }
    return res;
}
status_t PCA6416_SetPins(uint16_t mask)
{
    status_t res;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA6416_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    res = i2c_write(PCA6416_OUTPUT_PORT0_REG, shadow | mask);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow |= mask;
    }
    return res;
}

status_t PCA6416_TogglePins(uint16_t mask)
{
    status_t res;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA6416_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
    res = i2c_write(PCA6416_OUTPUT_PORT0_REG, shadow ^ mask);
    if (res == kStatus_Success) {
        // Update the shadow value only now that we know the write was a success
        shadow ^= mask;
    }
    return res;
}

status_t PCA6416_ReadPins(uint16_t* pValue)
{
    status_t res;
    if (!already_initialized) {
        // Do lazy initialization the first time the GPIO expander is needed.
        res = PCA6416_Init();
        if (res != kStatus_Success) {
            return res;
        }
    }
	return i2c_read(PCA6416_INPUT_PORT0_REG, pValue);
}

bool PCA6416_IsButtonPressed(void) {
	uint16_t val = 0;
	if (PCA6416_ReadPins(&val) == kStatus_Success) {
		return !(val & PCA_USER_BUTTON);
	}
	return false;
}
