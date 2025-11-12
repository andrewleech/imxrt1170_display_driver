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
#ifndef _PCA9530_H_
#define _PCA9530_H_

#include "fsl_common.h"

/*!
 * @addtogroup pca9530
 * @{
 */


#define PCA9530_LED0  (1<<0)
#define PCA9530_LED1  (1<<2)

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
status_t PCA9530_Init(void);

/*!
 * @brief Sets each masked pin to LOW
 *
 * @param mask with 1 for each pin that should be LOW
 */
status_t PCA9530_ClearPins(uint8_t mask);

/*!
 * @brief Sets each masked pin to HIGH
 *
 * @param mask with 1 for each pin that should be HIGH
 */
status_t PCA9530_SetPins(uint8_t mask);

/*!
 * @brief Toggles the value of each masked pin
 *
 * @param mask with 1 for each pin that should be toggled
 */
status_t PCA9530_TogglePins(uint8_t mask);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PCA9530_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
