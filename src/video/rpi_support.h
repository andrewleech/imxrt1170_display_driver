/*
 * Copyright  2019 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RPI_SUPPORT_H_
#define _RPI_SUPPORT_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define RPI_ADDR  0x45

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initialize communication with Raspberry Pi display controller
 */
void talk_to_raspi(void);

#if defined(__cplusplus)
}
#endif

#endif /* _RPI_SUPPORT_H_ */