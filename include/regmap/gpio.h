/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/
#ifndef PMIC_REGMAP_GPIO_H
#define PMIC_REGMAP_GPIO_H

/**
 * @file gpio.h
 *
 * @brief PMIC LLD GPIO module register map definitions.
 */

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                       LP8774x GPIO Module Register Map                     */
/* ========================================================================== */

// GPIO module register addresses
#define GPO_CONF_REG            (0x1CU)
#define VMON1_GPO1_SEQUENCE_REG (0x26U)
#define GPO2_SEQUENCE_REG       (0x27U)
#define NRSTOUT_SEQUENCE_REG    (0x28U)

// GPO_CONF
#define GPO2_FAULT2_POL_SHIFT (7U)
#define GPO1_FAULT1_POL_SHIFT (6U)
#define GPO2_FAULT2_OD_SHIFT  (5U)
#define GPO1_FAULT1_OD_SHIFT  (4U)
#define GPO2_SEL_SHIFT        (2U)
#define GPO1_SEL_SHIFT        (0U)
#define GPO2_FAULT2_POL_MASK  (0x01U << GPO2_FAULT2_POL_SHIFT)
#define GPO1_FAULT1_POL_MASK  (0x01U << GPO1_FAULT1_POL_SHIFT)
#define GPO2_FAULT2_OD_MASK   (0x01U << GPO2_FAULT2_OD_SHIFT)
#define GPO1_FAULT1_OD_MASK   (0x01U << GPO1_FAULT1_OD_SHIFT)
#define GPO2_SEL_MASK         (0x03U << GPO2_SEL_SHIFT)
#define GPO1_SEL_MASK         (0x03U << GPO1_SEL_SHIFT)

// VMON1_GPO1_SEQUENCE, GPO2_SEQUENCE, NRSTOUT_SEQUENCE
#define SHUTDOWN_DELAY_SHIFT (4U)
#define STARTUP_DELAY_SHIFT  (0U)
#define SHUTDOWN_DELAY_MASK  (0x0FU << SHUTDOWN_DELAY_SHIFT)
#define STARTUP_DELAY_MASK   (0x0FU << STARTUP_DELAY_SHIFT)

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_REGMAP_GPIO_H */
