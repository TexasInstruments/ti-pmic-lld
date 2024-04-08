/****************************************************************************
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 ****************************************************************************/

/**
 *  @file  pmic_irq_test.h
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC IRQ
 *          and basic register lock/unlock and other miscellaneous tests.
 */
#ifndef PMIC_IRQ_TEST_H_
#define PMIC_IRQ_TEST_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_test_common.h"
#include "pmic_core_priv.h"
#include "pmic_irq.h"
#include "pmic_irq_priv.h"
#include "pmic_irq_tps65386x.h"

#include "kernel/dpl/DebugP.h"
#include "kernel/dpl/ClockP.h"
#include "kernel/dpl/AddrTranslateP.h"
#include "kernel/dpl/HwiP.h"
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#include <stdlib.h>
#include <drivers/hw_include/cslr_soc.h>
#include <drivers/gpio.h>
#include "ti_drivers_config.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */
/*
 * Board info
 */
/* This is based on DMSC board config and core */
#define BOARD_BUTTON_GPIO_INTR_NUM      (CSLR_R5FSS0_CORE0_INTR_GPIO_INTRXBAR_OUT_14)
#define BOARD_BUTTON_GPIO_SWITCH_NUM    (11)
/* ========================================================================== */
/*                                  Variables                                 */
/* ========================================================================== */
uint32_t            gGpioBaseAddr = CONFIG_GPIO0_BASE_ADDR;
HwiP_Object         gGpioHwiObject;
volatile uint32_t   gGpioIntrDone = 0;
/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
int32_t Pmic_maskGpioIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const uint8_t irqGpioNum,
                          const bool mask,
                          const uint8_t gpioIntrType);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_IRQ_TEST_H_ */