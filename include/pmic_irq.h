/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
 * @file   pmic_irq.h
 *
 * @brief  PMIC IRQ Driver API/interface file.
 */

#ifndef PMIC_IRQ_H_
#define PMIC_IRQ_H_

/* ==========================================================================*/
/*                             Include Files                                 */
/* ==========================================================================*/
#include "pmic_core.h"

#ifdef __cplusplus
extern "C" {
#endif
/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

#define PMIC_IRQ_GPO_ALL_INT_MASK_NUM (5U)

#define PMIC_IRQ_ALL (0xFFU)

#define PMIC_IRQ_CLEAR_NONE (0U)
#define PMIC_IRQ_CLEAR (1U)

#define PMIC_IRQ_UNMASK ((bool)-1)
#define PMIC_IRQ_MASK ((bool)0)

#define PMIC_IRQ_GPIO_INT_TYPE (0x0U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * @brief Structure for storing PMIC interrupt status.
 */
typedef struct Pmic_IrqStatus_s {
  uint32_t intStatus[4]; /**< Array to store interrupt status. */
} Pmic_IrqStatus_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

int32_t Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_IrqStatus_t *pErrStat, const bool clearIRQ);

int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqNum);

int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t irqNum, const bool mask);

int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_IrqStatus_t *pErrStat, uint8_t *pIrqNum);

int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqGpioNum, const bool mask,
                             const uint8_t gpioIntrType);

int32_t Pmic_irqGetMaskIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t irqNum, bool *pMaskStatus);

int32_t Pmic_irqGetGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const uint8_t gpioIntrType,
                                bool *pIntrMaskStat);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_H_ */
