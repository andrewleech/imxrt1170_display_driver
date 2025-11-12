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
#ifndef _PCA6416_H_
#define _PCA6416_H_

#include "fsl_common.h"

/*!
 * @addtogroup pca6416
 * @{
 */


#define PCA_BT_REG_ON_3V3     (1<<0)
#define PCA_WL_REG_ON_3V3     (1<<1)
#define PCA_LCD_DISP_MIPI_RST (1<<2)
#define PCA_BT_DEV_WAKE_3V3   (1<<3)
#define PCA_RGB_LED_RED       (1<<4)
#define PCA_RGB_LED_BLUE      (1<<5)
#define PCA_RGB_LED_GREEN     (1<<6)
#define PCA_PCIE_PERST_L_3V3  (1<<7)
#define PCA_M2_B_DISABLE_N    (1<<8)
#define PCA_M2_B_PWR_OFF_N    (1<<9)
#define PCA_LCD_BL_PWR        (1<<10)

#define PCA_USER_BUTTON       (1<<13)
#define PCA_LED_RED           (1<<14)
#define PCA_LED_GREEN         (1<<15)

#define PCA_ALL_LEDS      (PCA_RGB_LED_RED|PCA_RGB_LED_BLUE|PCA_RGB_LED_GREEN|PCA_LED_RED|PCA_LED_GREEN)
#define PCA_DISPLAY_PINS  (PCA_LCD_DISP_MIPI_RST|PCA_LCD_BL_PWR)

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes with all pins LOW outputs
 *
 */
status_t PCA6416_Init(void);

/*!
 * @brief Configure pins
 *
 * @param values pins marked with a 1 in the mask will be HIGH - all other pins LOW
 */
status_t PCA6416_SetAllPins(uint16_t values);

/*!
 * @brief Sets each masked pin to LOW
 *
 * @param mask with 1 for each pin that should be LOW
 */
status_t PCA6416_ClearPins(uint16_t mask);

/*!
 * @brief Sets each masked pin to HIGH
 *
 * @param mask with 1 for each pin that should be HIGH
 */
status_t PCA6416_SetPins(uint16_t mask);

/*!
 * @brief Toggles each masked pin to HIGH
 *
 * @param mask with 1 for each pin that should be toggled
 */
status_t PCA6416_TogglePins(uint16_t mask);

/*!
 * @brief Reads the state of all pins
 *
 * @param pValue updated with the state of all pins only if the read was a success
 */
status_t PCA6416_ReadPins(uint16_t* pValue);

/*!
 * @brief Checks if the USER BUTTON is pressed
 */
bool PCA6416_IsButtonPressed(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PCA6416_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
