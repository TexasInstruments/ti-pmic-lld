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
 *  \ingroup  DRV_PMIC_MODULE
 *  \defgroup DRV_PMIC_IRQ_MODULE PMIC Interrupt Driver API.
 *      This module explains about PMIC Interrupt driver parameters and API
 *      usage. PMIC Interrupt Driver module covers all Interrupt feature APIs,
 *      which includes Get/clear Interrupt status, extract the Interrupt status
 *      as per Interrupt hierarchy, masking/unmasking of all Interrupts and
 *      a separate API for GPIO Interrupt masking/unmasking.
 *
 *      Supported PMIC devices for Interrupt Module:
 *      1. TPS65386x (BB PMIC Device).
 *
 *  @{
 */

/**
 * \file   pmic_irq.h
 *
 * \brief  PMIC IRQ Driver API/interface file.
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
/**
 *  \anchor Pmic_IrqGpioNum
 *  \name PMIC GPIO Interrupt Mask values
 *
 *  @{
 */
#define PMIC_IRQ_GPO_ALL_INT_MASK_NUM (5U)
/* @} */

/**
 *  \anchor Pmic_IrqNum
 *  \name PMIC IRQ flag for all Interrupts
 *
 *  @{
 */
#define PMIC_IRQ_ALL (0xFFU)
/* @} */

/**
 *  \anchor Pmic_IrqClearFlag
 *  \name PMIC IRQ Clear flags
 *
 *  @{
 */
#define PMIC_IRQ_CLEAR_NONE (0U)
#define PMIC_IRQ_CLEAR (1U)
/* @} */

/**
 *  \anchor Pmic_IrqMaskFlag
 *  \name PMIC IRQ Mask flag
 *
 *  @{
 */
#define PMIC_IRQ_UNMASK (bool)false
#define PMIC_IRQ_MASK (bool)true
/* @} */

/**
 *  \anchor Pmic_IrqGpioIntrType
 *  \name PMIC IRQ GPIO Interrupt type
 *
 *  @{
 */
#define PMIC_IRQ_GPIO_INT_TYPE (0x0U)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
 * \brief   PMIC Interrupt status object structure.
 *
 * \param   intStatus    To store all available interrupts using bit fields.
 *
 */
typedef struct Pmic_IrqStatus_s {
  uint32_t intStatus[4];
} Pmic_IrqStatus_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief   API to read Error status.
 *
 * Requirement:
 * Design: did_pmic_irq_cfg_readback
 * Architecture:
 *
 *          This function does the following:
 *             1. This function gets the interrupt status by reading pmic
 *                IRQ register as per IRQ hierarchy defined in device TRM.
 *             2. Decipher error from top register to actual error code.
 *             3. Store the status of all Interrupts.
 *             4. Support clearing interrupts depends on clearIRQ flag.
 *          Note: Application has to ensure to clear the interrupts after the
 *                interrupt has been serviced. If the interrupts are not cleared
 *                after the interrupt had been serviced then Application will
 *                not get any further interrupts which results in event miss
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [OUT]   Pointer to store Error status.
 * \param   clearIRQ          [IN]    Flag to clear Interrupt status after
 *                                    deciphering the interrupt status.
 *                                    For valid values: \ref Pmic_IrqClearFlag.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_IrqStatus_t *pErrStat, const bool clearIRQ);

/*!
 * \brief   API to clear Error status.
 *
 * Requirement:
 * Design: did_pmic_irq_cfg_readback
 * Architecture:
 *
 *          This function does the following:
 *          1. This function clears the IRQ status in PMIC register for a given
 *             IRQ Number.
 *          2. Validates given IRQ Number and find the IRQ register that is
 *             to be cleared.
 *          3. Expected to be called after an IRQ status is deciphered by
 *             Pmic_irqGetErrStatus().
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqNum            [IN]    Interrupt number to clear the status.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqNum
 *                                    for TPS65386x BB PMIC.
 *                                    \ref Pmic_IrqNum
 *                                    for all interrupts except gpio.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqNum);

/*!
 * \brief   API to mask/unmask interrupts.
 *
 * Requirement:
 * Design: did_pmic_irq_cfg_readback
 * Architecture:
 *
 *          This function does the following:
 *          1. This function mask/unmask the given IRQ Number.
 *          2. Validates given IRQ Number and find the IRQ register that
 *             is to be masked/unmasked.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqNum            [IN]    Interrupt number to be masked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqNum
 *                                    for TPS65386x BB PMIC.
 *                                    \ref Pmic_IrqNum
 *                                    for all interrupts except gpio.
 * \param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    For valid values: \ref Pmic_IrqMaskFlag.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t irqNum, const bool mask);

/*!
 * \brief   API to extract each Error status.
 *
 * Requirement:
 * Design: did_pmic_irq_cfg_readback
 * Architecture:
 *
 *          This function is used to extract each Error status from pErrStat
 *          as per the hierarchy given in the TRM. This function clears the
 *          Error status after the status is extracted. This API is expected to
 *          be called after Pmic_irqGetErrStatus.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [IN]    Pointer containing Error Status.
 * \param   pIrqNum           [OUT]   Pointer to store the IRQ Number extracted
 *                                    For TPS65386x BB:
 *                                    \ref Pmic_tps65386x_IrqNum.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_IrqStatus_t *pErrStat, uint8_t *pIrqNum);

/*!
 * \brief   API to mask/unmask GPIO interrupts.
 *
 * Requirement:
 * Design: did_pmic_irq_cfg_readback
 * Architecture:
 *
 *          This function is used to Mask or Unmask GPIO Rise and Fall
 *          Interrupts based on the GPIO IRQ Number.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqGpioNum        [IN]    GPIO Interrupt to be masked/unmasked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqGpioNum
 *                                    for TPS65386x BB PMIC.
 *                                    \ref Pmic_IrqGpioNum
 *                                    for all gpio interrupts.
 * \param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    Valid values: \ref Pmic_IrqMaskFlag.
 * \param   gpioIntrType      [IN]    Parameter to mask GPIO RISE and FALL
 *                                    Interrupt.
 *                                    Valid values: \ref Pmic_IrqGpioIntrType.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqGpioNum, const bool mask,
                             const uint8_t gpioIntrType);

/*!
 * \brief   API to read the status of PMIC interrupts is masked or not
 *
 * Requirement:
 * Design: did_pmic_irq_mask_status
 * Architecture:
 *
 *          This function does the following:
 *          1. This function reads the status of interrupt is masked or not for
 *             the given IRQ Number.
 *          2. Validates given IRQ Number and find the IRQ register to check
 *             the status of interrupt is masked or not
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqNum            [IN]    Interrupt number to be masked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqNum
 *                                    for TPS65386x BB PMIC.
 * \param   pMaskStatus       [OUT]   Pointer to hold the status of interrupt is
 *                                    masked or not
 *                                    For Valid values:
 *                                    \ref Pmic_IrqMaskFlag
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGetMaskIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t irqNum, bool *pMaskStatus);

/*!
 * \brief   API to read the status of PMIC GPIO interrupts is masked or not
 *
 * Requirement:
 * Design: did_pmic_irq_mask_status
 * Architecture:
 *
 *          This function reads the status of GPIO Rise and Fall interrupt is
 *          masked or not for the given GPIO IRQ Number
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqGpioNum        [IN]    GPIO Interrupt to be masked/unmasked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqGpioNum
 *                                    for TPS65386x BB PMIC.
 * \param   gpioIntrType      [IN]    Parameter to mask GPIO RISE and FALL
 *                                    Interrupt.
 *                                    Valid values: \ref Pmic_IrqGpioIntrType.
 * \param   pRiseIntrMaskStat [OUT]   Pointer to hold status of GPIO Rise
 *                                    Interrupt is masked or not
 *                                    For Valid values:
 *                                    \ref Pmic_IrqMaskFlag
 * \param   pFallIntrMaskStat [OUT]   Pointer to hold status of GPIO Fall
 *                                    Interrupt is masked or not
 *                                    For Valid values:
 *                                    \ref Pmic_IrqMaskFlag
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGetGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const uint8_t gpioIntrType,
                                bool *pIntrMaskStat);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_H_ */

/* @} */
