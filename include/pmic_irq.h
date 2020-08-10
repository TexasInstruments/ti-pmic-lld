/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *      1. TPS6594x (Leo PMIC Device).
 *      2. LP8764x  (Hera PMIC Device).
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
#include <pmic_core.h>
#include <cfg/tps6594x/pmic_irq_tps6594x.h>
#include <cfg/lp8764x/pmic_irq_lp8764x.h>

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
#define PMIC_IRQ_GPIO_ALL_INT_MASK_NUM   (12U)
/* @} */

/**
 *  \anchor Pmic_IrqNum
 *  \name PMIC IRQ flag for all Interrupts
 *
 *  @{
 */
#define PMIC_IRQ_ALL        (0xFF)
/* @} */

/**
 *  \anchor Pmic_IrqClearFlag
 *  \name PMIC IRQ Clear flags
 *
 *  @{
 */
#define PMIC_IRQ_CLEAR_NONE             (0U)
#define PMIC_IRQ_CLEAR                  (1U)
/* @} */

/**
 *  \anchor Pmic_IrqMaskFlag
 *  \name PMIC IRQ Mask flag
 *
 *  @{
 */
#define PMIC_IRQ_UNMASK           (bool)false
#define PMIC_IRQ_MASK             (bool)true
/* @} */

/**
 *  \anchor Pmic_IrqGpioIntrType
 *  \name PMIC IRQ GPIO Interrupt type
 *
 *  @{
 */
#define PMIC_IRQ_GPIO_FALL_INT_TYPE             (0x0U)
#define PMIC_IRQ_GPIO_RISE_INT_TYPE             (0x1U)
#define PMIC_IRQ_GPIO_RISE_FALL_INT_TYPE        (0x2U)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/*!
* \brief   PMIC Interrupt status object structure.
*
* \param   intStatus    To store all available interrupts using bit fields.
*                       Supported Interrupt Bit fields: \ref Pmic_IrqNum.
*/
typedef struct Pmic_IrqStatus_s
{
    uint32_t intStatus[4];
} Pmic_IrqStatus_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/*!
 * \brief   API to read Error status.
 *          This function does the following:
 *             1. This function gets the interrupt status by reading pmic
 *                IRQ register as per IRQ hierarchy defined in device TRM.
 *             2. Decipher error from top register to actual error code.
 *             3. Store the status of all Interrupts.
 *             4. Support clearing interrupts depends on clearIRQ flag.
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
int32_t  Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_IrqStatus_t  *pErrStat,
                              const bool         clearIRQ);

/*!
 * \brief   API to clear Error status.
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
 *                                    \ref Pmic_tps6594x_IrqNum
 *                                    for TPS6594x LEO PMIC,
 *                                    \ref Pmic_lp8764x_IrqNum
 *                                    for LP8764x HERA PMIC,
 *                                    \ref Pmic_IrqNum
 *                                    for all interrupts except gpio.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t  Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                              const uint8_t      irqNum);

 /*!
 * \brief   API to mask/unmask interrupts.
 *          This function does the following:
 *          1. This function mask/unmask the given IRQ Number.
 *          2. Validates given IRQ Number and find the IRQ register that
 *             is to be masked/unmasked.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqNum            [IN]    Interrupt status to be cleared.
 *                                    For Valid values:
 *                                    \ref Pmic_tps6594x_IrqNum
 *                                    for TPS6594x LEO PMIC,
 *                                    \ref Pmic_lp8764x_IrqNum
 *                                    for LP8764x HERA PMIC,
 *                                    \ref Pmic_IrqNum
 *                                    for all interrupts except gpio.
 * \param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    For valid values: \ref Pmic_IrqMaskFlag.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t      irqNum,
                         const bool         mask);

 /*!
 * \brief   API to extract each Error status.
 *          This function is used to extract each Error status from pErrStat 
 *          as per the hierarchy given in the TRM. This function clears the 
 *          Error status after the status is extracted. This API is expected to
 *          be called after Pmic_irqGetErrStatus.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   pErrStat          [IN]    Pointer containing Error Status.
 * \param   pIrqNum           [OUT]   Pointer to store the IRQ Number extracted
 *                                    For TPS6594x LEO:
 *                                    \ref Pmic_tps6594x_IrqNum.
 *                                    For LP8764x HERA:
 *                                    \ref Pmic_lp8764x_IrqNum.
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_IrqStatus_t        *pErrStat,
                                uint8_t                 *pIrqNum);

/*!
 * \brief   API to mask/unmask GPIO interrupts.
 *          This function is used to Mask or Unmask GPIO Rise and Fall
 *          Interrupts based on the GPIO IRQ Number.
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   irqGpioNum        [IN]    GPIO Interrupt to be masked/unmasked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps6594x_IrqGpioNum
 *                                    for TPS6594x LEO PMIC,
 *                                    \ref Pmic_lp8764x_IrqGpioNum
 *                                    for LP8764x HERA PMIC,
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
                             const uint8_t      irqGpioNum,
                             const bool         mask,
                             const uint8_t      gpioIntrType);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_IRQ_H_ */

/* @} */
