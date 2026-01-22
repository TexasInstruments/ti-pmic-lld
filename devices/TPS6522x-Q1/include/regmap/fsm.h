/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
/**
 * @file fsm.h
 *
 * @brief PMIC LLD FSM module register addresses and bit fields.
 */
#ifndef REGMAP_FSM_H
#define REGMAP_FSM_H

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Register Addresses                            */
/* ========================================================================== */

#define FSM_TRIG_SEL_1_REG ((uint16_t)0x44U)
#define FSM_TRIG_SEL_2_REG ((uint16_t)0x45U)
#define FSM_TRIG_MASK_1_REG ((uint16_t)0x46U)
#define FSM_TRIG_MASK_2_REG ((uint16_t)0x47U)
#define RECOV_CNT_REG_1_REG ((uint16_t)0x83U)
#define RECOV_CNT_REG_2_REG ((uint16_t)0x84U)
#define SOFT_REBOOT_REG_REG ((uint16_t)0xABU)
#define STARTUP_CTRL_REG    ((uint16_t)0xC3U)

/* ========================================================================== */
/*                              Register Bit Fields                           */
/* ========================================================================== */

// FSM_TRIG_SEL_1
#define SEVERE_ERR_TRIG_SHIFT (6U)
#define OTHER_RAIL_TRIG_SHIFT (4U)
#define SOC_RAIL_TRIG_SHIFT   (2U)
#define MCU_RAIL_TRIG_SHIFT   (0U)
#define SEVERE_ERR_TRIG_MASK  (0x03U << SEVERE_ERR_TRIG_SHIFT)
#define OTHER_RAIL_TRIG_MASK  (0x03U << OTHER_RAIL_TRIG_SHIFT)
#define SOC_RAIL_TRIG_MASK    (0x03U << SOC_RAIL_TRIG_SHIFT)
#define MCU_RAIL_TRIG_MASK    (0x03U << MCU_RAIL_TRIG_SHIFT)

// FSM_TRIG_SEL_2
#define MODERATE_ERR_TRIG_SHIFT (0U)
#define MODERATE_ERR_TRIG_MASK  (0x03U << MODERATE_ERR_TRIG_SHIFT)

// FSM_TRIG_MASK_1
#define GPIO4_FSM_MASK_POL_SHIFT ((uint8_t)7U)
#define GPIO4_FSM_MASK_SHIFT     ((uint8_t)6U)
#define GPIO3_FSM_MASK_POL_SHIFT ((uint8_t)5U)
#define GPIO3_FSM_MASK_SHIFT     ((uint8_t)4U)
#define GPIO2_FSM_MASK_POL_SHIFT ((uint8_t)3U)
#define GPIO2_FSM_MASK_SHIFT     ((uint8_t)2U)
#define GPIO1_FSM_MASK_POL_SHIFT ((uint8_t)1U)
#define GPIO1_FSM_MASK_SHIFT     ((uint8_t)0U)
#define GPIO4_FSM_MASK_POL_MASK  (0x01U << GPIO4_FSM_MASK_POL_SHIFT)
#define GPIO4_FSM_MASK_MASK      (0x01U << GPIO4_FSM_MASK_SHIFT)
#define GPIO3_FSM_MASK_POL_MASK  (0x01U << GPIO3_FSM_MASK_POL_SHIFT)
#define GPIO3_FSM_MASK_MASK      (0x01U << GPIO3_FSM_MASK_SHIFT)
#define GPIO2_FSM_MASK_POL_MASK  (0x01U << GPIO2_FSM_MASK_POL_SHIFT)
#define GPIO2_FSM_MASK_MASK      (0x01U << GPIO2_FSM_MASK_SHIFT)
#define GPIO1_FSM_MASK_POL_MASK  (0x01U << GPIO1_FSM_MASK_POL_SHIFT)
#define GPIO1_FSM_MASK_MASK      (0x01U << GPIO1_FSM_MASK_SHIFT)

// FSM_TRIG_MASK_2
#define GPIO6_FSM_MASK_POL_SHIFT ((uint8_t)3U)
#define GPIO6_FSM_MASK_SHIFT     ((uint8_t)2U)
#define GPIO5_FSM_MASK_POL_SHIFT ((uint8_t)1U)
#define GPIO5_FSM_MASK_SHIFT     ((uint8_t)0U)
#define GPIO6_FSM_MASK_POL_MASK  (0x01U << GPIO6_FSM_MASK_POL_SHIFT)
#define GPIO6_FSM_MASK_MASK      (0x01U << GPIO6_FSM_MASK_SHIFT)
#define GPIO5_FSM_MASK_POL_MASK  (0x01U << GPIO5_FSM_MASK_POL_SHIFT)
#define GPIO5_FSM_MASK_MASK      (0x01U << GPIO5_FSM_MASK_SHIFT)

// RECOV_CNT_REG_1
#define RECOV_CNT_SHIFT (0U)
#define RECOV_CNT_MASK  (0x0FU << RECOV_CNT_SHIFT)

// RECOV_CNT_REG_2
#define RECOV_CNT_CLR_SHIFT (4U)
#define RECOV_CNT_THR_SHIFT (0U)
#define RECOV_CNT_CLR_MASK  (0x01U << RECOV_CNT_CLR_SHIFT)
#define RECOV_CNT_THR_MASK  (0x0FU << RECOV_CNT_THR_SHIFT)

// SOFT_REBOOT_REG
#define SOFT_REBOOT_SHIFT (0U)
#define SOFT_REBOOT_MASK  (1U << SOFT_REBOOT_SHIFT)

// STARTUP_CTRL
#define FIRST_STARTUP_DONE_SHIFT (7U)
#define STARTUP_DEST_SHIFT       (5U)
#define FIRST_STARTUP_DONE_MASK  (0x01U << FIRST_STARTUP_DONE_SHIFT)
#define STARTUP_DEST_MASK        (0x03U << STARTUP_DEST_SHIFT)

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* REGMAP_FSM_H */
