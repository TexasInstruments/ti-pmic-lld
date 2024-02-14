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
 * \file   pmic_irq_status.c
 *
 * @brief  This file contains APIs definitions for PMIC Interrupt IRQ Status
 *         Handler.
 *
 */

#include "pmic_core_priv.h"
#include "pmic_irq.h"
#include "pmic_irq_priv.h"
#include "pmic_irq_tps65386x_priv.h"
#include <pmic_irq_tps65386x.h>

/**
 * @brief  Function to Set the intStatus bit position.
 *
 *  [IN]:Pmic_IrqStatus_t  *pErrStat - structure contains an array -
 * intStatus[4] [IN]:uint32_t pos will tell the position of bit in which
 * operation will happpen
 *
 */
void Pmic_intrBitSet(Pmic_IrqStatus_t *pErrStat, uint32_t pos) {
  uint32_t intStatSize = 0U;

  /* Size of intStatus in bits */
  intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;

  pErrStat->intStatus[pos / intStatSize] |=
      (((uint32_t)1U) << (pos % intStatSize));
}

/**
 * @brief  Function to Clear the intStatus bit position.
 */
static void Pmic_intrBitClear(Pmic_IrqStatus_t *pErrStat,
                              const uint8_t *pIrqNum) {
  uint32_t intStatSize = 0U;

  /* Size of intStatus in bits */
  intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;

  pErrStat->intStatus[(*pIrqNum) / intStatSize] &=
      ~(1U << ((*pIrqNum) % intStatSize));
}

/**
 * @brief  Function to Extract the intStatus bit position.
 */
static uint8_t Pmic_intrBitExtract(const Pmic_IrqStatus_t *pErrStat,
                                   uint8_t maxVal) {
  uint8_t irqNum = 0U;
  uint32_t intStatSize = 0U;

  /* Size of intStatus in bits */
  intStatSize = sizeof(pErrStat->intStatus[0U]) << 3U;
  for (irqNum = 0U; irqNum < maxVal; irqNum++) {
    if (((pErrStat->intStatus[irqNum / intStatSize]) &
         (((uint32_t)1U) << (irqNum % intStatSize))) != 0U) {
      break;
    }
  }

  return irqNum;
}

/**
 * @brief  Function to Check the device specific Max IrqNum.
 */
static int32_t Pmic_irqValidateIrqNum(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                      const uint8_t irqNum) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t maxVal;

  if ((pPmicCoreHandle->pmicDeviceType) == PMIC_DEV_BB_TPS65386X) {
    maxVal = PMIC_TPS65386X_IRQ_MAX_NUM;
    if ((irqNum > maxVal) && (irqNum != PMIC_IRQ_ALL)) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }
  }

  return pmicStatus;
}

/**
 * @brief  Function to Check the device specific Max IrqNum
 */
static int32_t Pmic_irqValidateIrqNumGetMaskIntrStatus(
    const Pmic_CoreHandle_t *pPmicCoreHandle, const uint8_t irqNum) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t maxVal;

  if ((pPmicCoreHandle->pmicDeviceType) == PMIC_DEV_BB_TPS65386X) {
    maxVal = PMIC_TPS65386X_IRQ_MAX_NUM;
    if (irqNum > maxVal) {
      pmicStatus = PMIC_ST_ERR_INV_PARAM;
    }
  }

  return pmicStatus;
}

/**
 * @brief  Function to get the device specific Max IrqNum.
 */
static void Pmic_getMaxVal(const Pmic_CoreHandle_t *pPmicCoreHandle,
                           uint8_t *maxVal) {
  (*maxVal) = PMIC_TPS65386X_IRQ_MAX_NUM;
}

/**
 * @brief  Function to get the Device specific Interrupt Configuration
 *         registers.
 */
static void Pmic_get_intrCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_IntrCfg_t **pIntrCfg) {
  if ((pPmicCoreHandle->pmicDeviceType) == PMIC_DEV_BB_TPS65386X) {
    pmic_get_tps65386x_intrCfg(pIntrCfg);
  }
}

/**
 * @brief  Function to get the Device specific GPIO Interrupt configuration
 *         registers.
 */
static void Pmic_get_gpioIntrCfg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                 Pmic_GpioIntrTypeCfg_t **pGpioIntrCfg) {
  pmic_get_tps65386x_intrGpioCfg(pGpioIntrCfg);
}

/**
 * @brief  Function to Mask/Unmask GPIO Interrupts.
 */
static int32_t Pmic_irqGpioMask(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t irqGpioNum, const bool mask,
                                const uint8_t gpioIntrType) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;
  Pmic_GpioIntrTypeCfg_t *pGpioIntrCfg = NULL;
  uint8_t bitMask = 0U;
  uint8_t maskVal = 0U;

  Pmic_get_gpioIntrCfg(pPmicCoreHandle, &pGpioIntrCfg);

  /* Start Critical Section */
  Pmic_criticalSectionStart(pPmicCoreHandle);

  if (PMIC_IRQ_GPIO_INT_TYPE == gpioIntrType) {
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioIntrMaskRegAddr,
        &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      if (((bool)true) == mask) {
        maskVal = 1U;
      }
      bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD
                 << pGpioIntrCfg[irqGpioNum].gpioMaskBitPos);
      Pmic_setBitField(&regData, pGpioIntrCfg[irqGpioNum].gpioMaskBitPos,
                       bitMask, maskVal);
      pmicStatus = Pmic_commIntf_sendByte(
          pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioIntrMaskRegAddr,
          regData);
    }
  }

  /* Stop Critical Section */
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}

/**
 * @brief  Function to Mask/Unmask GPIO Interrupts.
 */
static int32_t Pmic_maskGpioIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const uint8_t irqGpioNum, const bool mask,
                                 const uint8_t gpioIntrType) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t irqGpioId = 0U;

  if (PMIC_IRQ_GPO_ALL_INT_MASK_NUM != irqGpioNum) {
    pmicStatus =
        Pmic_irqGpioMask(pPmicCoreHandle, irqGpioNum, mask, gpioIntrType);
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      (PMIC_IRQ_GPO_ALL_INT_MASK_NUM == irqGpioNum)) {
    for (irqGpioId = 0U; irqGpioId < (PMIC_IRQ_GPO_ALL_INT_MASK_NUM - 1U);
         irqGpioId++) {
      pmicStatus =
          Pmic_irqGpioMask(pPmicCoreHandle, irqGpioId, mask, gpioIntrType);
    }
  }

  return pmicStatus;
}

/**
 * @brief  Function to clear IRQ status.
 */
static int32_t Pmic_irqClear(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqNum) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;
  Pmic_IntrCfg_t *pIntrCfg = NULL;
  uint8_t bitMask = 0U;

  Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

  /* Start Critical Section */
  Pmic_criticalSectionStart(pPmicCoreHandle);

  pmicStatus = Pmic_commIntf_recvByte(
      pPmicCoreHandle, pIntrCfg[irqNum].intrClrRegAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pIntrCfg[irqNum].intrClrBitPos);
    Pmic_setBitField(&regData, pIntrCfg[irqNum].intrClrBitPos, bitMask,
                     PMIC_IRQ_CLEAR);
    pmicStatus = Pmic_commIntf_sendByte(
        pPmicCoreHandle, pIntrCfg[irqNum].intrClrRegAddr, regData);
  }

  /* Stop Critical Section */
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}

/**
 * @brief  Function to Clear Interrupt Status register.
 */
static int32_t Pmic_irqClearStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   const uint8_t irqNum) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t irqId = 0U;
  uint8_t maxVal = 0U;

  if (PMIC_IRQ_ALL != irqNum) {
    pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqNum);
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum)) {
    Pmic_getMaxVal(pPmicCoreHandle, &maxVal);

    for (irqId = 0U; irqId < maxVal; irqId++) {
      pmicStatus = Pmic_irqClear(pPmicCoreHandle, irqId);
    }
  }

  return pmicStatus;
}

/**
 * @brief  Function to Mask/Unmask Interrupts.
 */
static int32_t Pmic_irqMask(Pmic_CoreHandle_t *pPmicCoreHandle,
                            const uint8_t irqNum, const bool mask,
                            const Pmic_IntrCfg_t *pIntrCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;
  uint8_t bitMask = 0U;
  uint8_t maskVal = 0U;

  /* Start Critical Section */
  Pmic_criticalSectionStart(pPmicCoreHandle);

  pmicStatus = Pmic_commIntf_recvByte(
      pPmicCoreHandle, pIntrCfg[irqNum].intrMaskRegAddr, &regData);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    if (((bool)true) == mask) {
      maskVal = 1U;
    }
    bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pIntrCfg[irqNum].intrMaskBitPos);
    Pmic_setBitField(&regData, pIntrCfg[irqNum].intrMaskBitPos, bitMask,
                     maskVal);
    pmicStatus = Pmic_commIntf_sendByte(
        pPmicCoreHandle, pIntrCfg[irqNum].intrMaskRegAddr, regData);
  }

  /* Stop Critical Section */
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}

/**
 * @brief  Function to Mask/Unmask PMIC Interrupts except GPIO.
 */
static int32_t Pmic_maskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqNum, const bool mask) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t irqId = 0U;
  uint8_t maxVal = 0U;
  Pmic_IntrCfg_t *pIntrCfg = NULL;

  Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

  if (PMIC_IRQ_ALL != irqNum) {
    if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqNum].intrMaskRegAddr) {
      pmicStatus = PMIC_ST_ERR_FAIL;
    }

    if (PMIC_ST_SUCCESS == pmicStatus) {
      pmicStatus = Pmic_irqMask(pPmicCoreHandle, irqNum, mask, pIntrCfg);
    }
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (PMIC_IRQ_ALL == irqNum)) {
    Pmic_getMaxVal(pPmicCoreHandle, &maxVal);

    for (irqId = 0U; irqId < maxVal; irqId++) {
      if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqId].intrMaskRegAddr) {
        continue;
      }

      pmicStatus = Pmic_irqMask(pPmicCoreHandle, irqId, mask, pIntrCfg);
    }
  }

  return pmicStatus;
}

/**
 * @brief  Function to get the L1 error registers.
 */
static void Pmic_irqGetL1Reg(const Pmic_CoreHandle_t *pPmicCoreHandle,
                             uint16_t *l1RegAddr, uint8_t count) {
  (*l1RegAddr) = PMIC_INT_UNUSED_REGADDR;

  switch (1U << count) {
  case PMIC_SAFETY_OFF_STATE_CFG_MASK:
    (*l1RegAddr) = PMIC_SAFETY_CFG_REGADDR;
    break;

  case PMIC_RDBK_INT_CFG_MASK:
    (*l1RegAddr) = PMIC_RDBK_INT_CFG1_REGADDR;
    break;

  case PMIC_OV_INT_CFG_MASK:
    (*l1RegAddr) = PMIC_OV_INT_CFG1_REGADDR;
    break;

  case PMIC_UV_INT_CFG_MASK:
    (*l1RegAddr) = PMIC_UV_INT_CFG1_REGADDR;
    break;

  case PMIC_WD_ERR_STAT_MASK:
    (*l1RegAddr) = PMIC_WDG_INT_CFG_REGADDR;
    break;

  case PMIC_ESM_INT_CFG_MASK:
    (*l1RegAddr) = PMIC_ESM_INT_CFG_REGADDR;
    break;

  case PMIC_CM_COMP_INT_MASK_CFG_MASK:
    (*l1RegAddr) = PMIC_CM_COMP_INT_MASK_CFG_REGADDR;
    break;

  case PMIC_CM_VMON_INT_CFG_MASK:
    (*l1RegAddr) = PMIC_CM_VMON_INT_CFG_REGADDR;
    break;

  default:
    break;
  }

  if (PMIC_INT_UNUSED_REGADDR == (*l1RegAddr)) {
    (*l1RegAddr) = 0U;
  }
}

/**
 * @brief  Function to decipher L2 Error.
 *
 *          Note: In this API, the default PMIC device is assumed as TPS65386x
 *                BB PMIC. While adding support for New PMIC device, developer
 *                need to update the API functionality for New PMIC device
 *                accordingly.
 */
static int32_t Pmic_irqGetL2Error(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  uint16_t l1RegAddr,
                                  Pmic_IrqStatus_t *pErrStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  switch (pPmicCoreHandle->pmicDeviceType) {

    /* Default case is valid only for TPS65386x BB PMIC */
    pmicStatus =
        Pmic_tps65386x_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
  }

  return pmicStatus;
}

/**
 * @brief   Function to Extract Interrupts as per Hierarchy given in TRM and
 *          clear the bit in pErrStat.
 */
static void Pmic_extractErrStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                  Pmic_IrqStatus_t *pErrStat,
                                  uint8_t *pIrqNum) {
  uint8_t maxVal = 0U;

  Pmic_getMaxVal(pPmicCoreHandle, &maxVal);

  *pIrqNum = Pmic_intrBitExtract(pErrStat, maxVal);
  /* To clear the Error Bit position after extracting */
  Pmic_intrBitClear(pErrStat, pIrqNum);
}

/**
 * @brief   API to read Error status.
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
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   pErrStat          [OUT]   Pointer to store Error status.
 * @param   clearIRQ          [IN]    Flag to clear Interrupt status after
 *                                    deciphering the interrupt status.
 *                                    For valid values: \ref Pmic_IrqClearFlag.
 *
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGetErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             Pmic_IrqStatus_t *pErrStat, const bool clearIRQ) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint16_t l1RegAddr = 0U;
  uint8_t count = 0U;
  uint8_t clearIRQStat = 0U;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    /* Clearing all Error Status structure members */
    pErrStat->intStatus[0U] = 0U;
    pErrStat->intStatus[1U] = 0U;
    pErrStat->intStatus[2U] = 0U;
    pErrStat->intStatus[3U] = 0U;

    if (PMIC_ST_SUCCESS == pmicStatus) {
      for (count = 7U;; count--) {
        l1RegAddr = 0U;
        Pmic_irqGetL1Reg(pPmicCoreHandle, &l1RegAddr, count);
        if (0U != l1RegAddr) {
          pmicStatus = Pmic_irqGetL2Error(pPmicCoreHandle, l1RegAddr, pErrStat);
        }
        if ((PMIC_ST_SUCCESS != pmicStatus) || (count == 0U)) {
          break;
        }
      }
      if (((bool)true) == clearIRQ) {
        clearIRQStat = 1U;
      }

      if (PMIC_IRQ_CLEAR == clearIRQStat) {
        pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle, PMIC_IRQ_ALL);
      }
    }
  }

  return pmicStatus;
}

/**
 * @brief   API to clear Error status.
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
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   irqNum            [IN]    Interrupt number to clear the status.
 *                                    Valid values are:
 *                                    For TPS659386x BB:
 *                                    \ref Pmic_tps65386x_IrqNum.
 *                                    For all: \ref Pmic_IrqNum.
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqClrErrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqNum) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_irqClearStatus(pPmicCoreHandle, irqNum);
  }

  return pmicStatus;
}

/**
 * @brief   API to mask/unmask interrupts.
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
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   irqNum            [IN]    Interrupt status to be cleared.
 *                                    Valid values are:
 *                                    For TPS65386x BB:
 *                                    \ref Pmic_tps65386x_IrqNum.
 *                                    For all: \ref Pmic_IrqNum.
 * @param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    For valid values: \ref Pmic_IrqMaskFlag.
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                         const uint8_t irqNum, const bool mask) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_irqValidateIrqNum(pPmicCoreHandle, irqNum);
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_maskIntr(pPmicCoreHandle, irqNum, mask);
  }

  return pmicStatus;
}

/**
 * @brief   API to extract each Error status.
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
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   pErrStat          [IN]    Pointer containing Error Status.
 * @param   pIrqNum           [OUT]   Pointer to store the IRQ Number extracted
 *                                    For TPS65386x BB:
 *                                    \ref Pmic_tps65386x_IrqNum.
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref: Pmic_ErrorCodes.
 */
int32_t Pmic_getNextErrorStatus(const Pmic_CoreHandle_t *pPmicCoreHandle,
                                Pmic_IrqStatus_t *pErrStat, uint8_t *pIrqNum) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pErrStat)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pIrqNum)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((pErrStat->intStatus[0U] == 0U) && (pErrStat->intStatus[1U] == 0U) &&
       (pErrStat->intStatus[2U] == 0U) && (pErrStat->intStatus[3U] == 0U))) {
    pmicStatus = PMIC_ST_ERR_INV_INT;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    Pmic_extractErrStatus(pPmicCoreHandle, pErrStat, pIrqNum);
  }

  return pmicStatus;
}

/**
 * @brief   API to mask/unmask GPIO interrupts.
 *
 * Requirement:
 * Design: did_pmic_irq_cfg_readback
 * Architecture:
 *
 *          This function is used to Mask or Unmask GPIO Rise and Fall
 *          Interrupts based on the GPIO IRQ Number.
 *
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   irqGpioNum        [IN]    GPIO Interrupt to be masked/unmasked.
 *                                    Valid values:
 *                                    For TPS65386x BB PMIC:
 *                                    \ref Pmic_tps65386x_IrqGpioNum.
 *                                    For all: \ref Pmic_IrqGpioNum.
 * @param   mask              [IN]    Parameter to mask/unmask INTR.
 *                                    Valid values: \ref Pmic_IrqMaskFlag.
 * @param   gpioIntrType      [IN]    Parameter to mask GPIO Interrupt.
 *                                    Valid values: \ref Pmic_IrqGpioIntrType.
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                             const uint8_t irqGpioNum, const bool mask,
                             const uint8_t gpioIntrType) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) &&
      ((irqGpioNum == PMIC_TPS65386X_IRQ_GPO_3_INT_MASK_NUM) &&
       (PMIC_DEV_BB_TPS65386X == pPmicCoreHandle->pmicDeviceType))) {
    pmicStatus = PMIC_ST_ERR_INV_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus =
        Pmic_maskGpioIntr(pPmicCoreHandle, irqGpioNum, mask, gpioIntrType);
  }

  return pmicStatus;
}

/**
 * @brief  Function to get status of the Interrupts is masked or not.
 */
static int32_t Pmic_getIrqMaskStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                     const uint8_t irqNum, bool *pMaskStatus,
                                     const Pmic_IntrCfg_t *pIntrCfg) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;
  uint8_t bitMask = 0U;

  /* Start Critical Section */
  Pmic_criticalSectionStart(pPmicCoreHandle);

  pmicStatus = Pmic_commIntf_recvByte(
      pPmicCoreHandle, pIntrCfg[irqNum].intrMaskRegAddr, &regData);

  /* Stop Critical Section */
  Pmic_criticalSectionStop(pPmicCoreHandle);

  if (PMIC_ST_SUCCESS == pmicStatus) {
    bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD << pIntrCfg[irqNum].intrMaskBitPos);
    *pMaskStatus = PMIC_IRQ_UNMASK;

    if ((Pmic_getBitField(regData, pIntrCfg[irqNum].intrMaskBitPos, bitMask)) ==
        PMIC_IRQ_MASK_VAL_1) {
      *pMaskStatus = PMIC_IRQ_MASK;
    }
  }
  return pmicStatus;
}

/**
 * @brief  Function to get status of the Interrupts is masked or not except
 *         GPIO Interrupts.
 */
static int32_t Pmic_getMaskIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                      const uint8_t irqNum, bool *pMaskStatus) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  Pmic_IntrCfg_t *pIntrCfg = NULL;

  Pmic_get_intrCfg(pPmicCoreHandle, &pIntrCfg);

  if (PMIC_IRQ_INVALID_REGADDR == pIntrCfg[irqNum].intrMaskRegAddr) {
    pmicStatus = PMIC_ST_ERR_FAIL;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus =
        Pmic_getIrqMaskStatus(pPmicCoreHandle, irqNum, pMaskStatus, pIntrCfg);
  }

  return pmicStatus;
}

/**
 * @brief   API to read the status of PMIC interrupts is masked or not
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
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   irqNum            [IN]    Interrupt number to be masked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqNum
 *                                    for TPS65386x BB PMIC
 * @param   pMaskStatus       [OUT]   Pointer to hold the status of interrupt is
 *                                    masked or not
 *                                    For Valid values:
 *                                    \ref Pmic_IrqMaskFlag
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values: \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGetMaskIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                  const uint8_t irqNum, bool *pMaskStatus) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pMaskStatus)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus =
        Pmic_irqValidateIrqNumGetMaskIntrStatus(pPmicCoreHandle, irqNum);
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_getMaskIntrStatus(pPmicCoreHandle, irqNum, pMaskStatus);
  }

  return pmicStatus;
}

/**
 * @brief  Function to read the status of GPIO Interrupts is masked or not
 */
static int32_t Pmic_getIrqGpioMaskStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                         const uint8_t irqGpioNum,
                                         const uint8_t gpioIntrType,
                                         bool *pIntrMaskStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;
  uint8_t regData = 0U;
  Pmic_GpioIntrTypeCfg_t *pGpioIntrCfg = NULL;
  uint8_t bitMask = 0U;

  Pmic_get_gpioIntrCfg(pPmicCoreHandle, &pGpioIntrCfg);

  /* Start Critical Section */
  Pmic_criticalSectionStart(pPmicCoreHandle);

  if (PMIC_IRQ_GPIO_INT_TYPE == gpioIntrType) {
    pmicStatus = Pmic_commIntf_recvByte(
        pPmicCoreHandle, pGpioIntrCfg[irqGpioNum].gpioIntrMaskRegAddr,
        &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
      bitMask = (PMIC_IRQ_MASK_CLR_BITFIELD
                 << pGpioIntrCfg[irqGpioNum].gpioMaskBitPos);
      *pIntrMaskStat = PMIC_IRQ_UNMASK;

      if ((Pmic_getBitField(regData, pGpioIntrCfg[irqGpioNum].gpioMaskBitPos,
                            bitMask)) == PMIC_IRQ_MASK_VAL_1) {
        *pIntrMaskStat = PMIC_IRQ_MASK;
      }
    }
  }

  /* Stop Critical Section */
  Pmic_criticalSectionStop(pPmicCoreHandle);

  return pmicStatus;
}

/**
 * @brief  Function to read the status of GPIO Interrupts is masked or not
 */
static int32_t Pmic_getMaskGpioIntrStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                                          const uint8_t irqGpioNum,
                                          const uint8_t gpioIntrType,
                                          bool *pIntrMaskStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  pmicStatus = Pmic_getIrqGpioMaskStatus(pPmicCoreHandle, irqGpioNum,
                                         gpioIntrType, pIntrMaskStat);

  return pmicStatus;
}

/**
 * @brief   API to read the status of PMIC GPIO interrupts is masked or not
 *          This function reads the status of GPIO Rise and Fall interrupt is
 *          masked or not for the given GPIO IRQ Number
 *
 * @param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * @param   irqGpioNum        [IN]    GPIO Interrupt to be masked/unmasked.
 *                                    For Valid values:
 *                                    \ref Pmic_tps65386x_IrqGpioNum
 *                                    for TPS65386x BB PMIC,
 * @param   gpioIntrType      [IN]    Parameter to mask GPO Interrupt.
 *                                    Valid values: \ref Pmic_IrqGpioIntrType.
 * @param   pRiseIntrMaskStat [OUT]   Pointer to hold status of GPIO
 *                                    Interrupt is masked or not
 *                                    For Valid values:
 *                                    \ref Pmic_IrqMaskFlag
 * @param   pFallIntrMaskStat [OUT]   Pointer to hold status of GPIO
 *                                    Interrupt is masked or not
 *                                    For Valid values:
 *                                    \ref Pmic_IrqMaskFlag
 * @return  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes.
 */
int32_t Pmic_irqGetGpioMaskIntr(Pmic_CoreHandle_t *pPmicCoreHandle,
                                const uint8_t irqGpioNum,
                                const uint8_t gpioIntrType,
                                bool *pIntrMaskStat) {
  int32_t pmicStatus = PMIC_ST_SUCCESS;

  if (NULL == pPmicCoreHandle) {
    pmicStatus = PMIC_ST_ERR_INV_HANDLE;
  }

  if ((PMIC_ST_SUCCESS == pmicStatus) && (NULL == pIntrMaskStat)) {
    pmicStatus = PMIC_ST_ERR_NULL_PARAM;
  }

  if (PMIC_ST_SUCCESS == pmicStatus) {
    pmicStatus = Pmic_getMaskGpioIntrStatus(pPmicCoreHandle, irqGpioNum,
                                            gpioIntrType, pIntrMaskStat);
  }

  return pmicStatus;
}
