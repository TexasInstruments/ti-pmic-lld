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
 *  \file pmic_core_priv.h
 *
 *  \brief This file contains PMIC Driver specific common API
 */

#ifndef PMIC_CORE_PRIV_H_
#define PMIC_CORE_PRIV_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <pmic.h>
#include <pmic_io_priv.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*!
 * \brief: PMIC Module Device Revision Infos
 */
#define PMIC_DEV_REV_REGADDR                (0x01U)
#define PMIC_WDG_LONGWIN_CFG_REGADDR        (0x405U)
/* On J7 1.0 EVM, PMIC_LEO_DEV_REV_ID is 0x08 */
#define PMIC_LEO_DEV_REV_ID                 (0x08U)
/* On J7 2.0 EVM, PMIC_LEO_DEV_REV_ID will be 0x03 */
/* #define PMIC_LEO_DEV_REV_ID              (0x03U) */

/*
 * Need to be verfied on J7VCL
 */
#define PMIC_HERA_DEV_REV_ID                (0x07U)

/*!
 * \brief  PMIC power Configuration Register Address
 */
#define PMIC_CONFIG_1_REGADDR               (0x7DU)

/*!
 * \brief: PMIC Recovery Counter Control and Status Registers
 */
#define PMIC_RECOV_CNT_REG_1_REGADDR        (0x83U)
#define PMIC_RECOV_CNT_REG_2_REGADDR        (0x84U)

/*!
 * \brief: HERA PMIC StartUP Control Register
 */
#define PMIC_STARTUP_CTRL_REGADDR           (0xC3U)

/*!
 * \brief  PMIC invalid register address
 */
#define PMIC_INVALID_REGADDR                (0xFFU)

/*!
 * \brief  PMIC invalid BIT SHIFT value
 */
#define PMIC_INVALID_BIT_SHIFT              (0xFFU)

/*!
 * \brief  PMIC Scratchpad register Addresses
 */
#define PMIC_SCRATCH_PAD_REG_1_REGADDR             (0xC9U)
#define PMIC_SCRATCH_PAD_REG_2_REGADDR             (0xCAU)
#define PMIC_SCRATCH_PAD_REG_3_REGADDR             (0xCBU)
#define PMIC_SCRATCH_PAD_REG_4_REGADDR             (0xCCU)

/*!
 * \brief  PMIC CONFIG_1 register Shift Values
 */
#define PMIC_CONFIG_1_TWARN_LEVEL_SHIFT                     (0U)
#define PMIC_CONFIG_1_TSD_ORD_LEVEL_SHIFT                   (1U)
#define PMIC_CONFIG_1_I2C1_HS_SHIFT                         (3U)
#define PMIC_CONFIG_1_I2C2_HS_SHIFT                         (4U)
#define PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_SHIFT                (5U)
#define PMIC_CONFIG_1_NSLEEP1_MASK_SHIFT                    (6U)
#define PMIC_CONFIG_1_NSLEEP2_MASK_SHIFT                    (7U)

/*!
 * \brief: PMIC Recovery Counter Register 1 Shift Values
 */
#define PMIC_RECOV_CNT_REG_1_RECOV_CNT_SHIFT        (0x00U)

/*!
 * \brief: PMIC Recovery Counter Register 2 Shift Values
 */
#define PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_SHIFT        (0x04U)
#define PMIC_RECOV_CNT_REG_2_RECOV_CNT_THR_SHIFT        (0x00U)

/*!
 * \brief: HERA PMIC StartUP Shift Values
 */
#define PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT        (0x03U)
#define PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT          (0x05U)

/*!
 * \brief   PMIC StartUp NSLEEP Shift Values
 */
#define PMIC_STARTUP_DEST_NSLEEP2B_SHIFT              (0x1U)
#define PMIC_STARTUP_DEST_NSLEEP1B_SHIFT              (0x0U)

/*!
 * \brief: PMIC Recovery Counter Register 1 Mask Values
 */
#define PMIC_RECOV_CNT_REG_1_RECOV_CNT_MASK                   \
                       ((uint8_t)(0x0FU << PMIC_RECOV_CNT_REG_1_RECOV_CNT_SHIFT))

/*!
 * \brief: PMIC Recovery Counter Register 2 Mask Values
 */
#define PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_MASK                   \
                   ((uint8_t)(0x01U << PMIC_RECOV_CNT_REG_2_RECOV_CNT_CLR_SHIFT))
#define PMIC_RECOV_CNT_REG_2_RECOV_CNT_THR_MASK                   \
                   ((uint8_t)(0x0FU << PMIC_RECOV_CNT_REG_2_RECOV_CNT_THR_SHIFT))

/*!
 * \brief: HERA PMIC StartUP Mask Values
 */
#define PMIC_STARTUP_CTRL_LP_STANDBY_SEL_MASK                   \
                     ((uint8_t)(0x01U << PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT))
#define PMIC_STARTUP_CTRL_STARTUP_DEST_MASK                     \
                     ((uint8_t)(0x03U << PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT))

/*!
 * \brief  PMIC CONFIG_1 register bit masks
 */
#define PMIC_CONFIG_1_TWARN_LEVEL_MASK                              \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_TWARN_LEVEL_SHIFT))
#define PMIC_CONFIG_1_TSD_ORD_LEVEL_MASK                            \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_TSD_ORD_LEVEL_SHIFT))
#define PMIC_CONFIG_1_I2C1_HS_MASK                                  \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_I2C1_HS_SHIFT))
#define PMIC_CONFIG_1_I2C2_HS_MASK                                  \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_I2C2_HS_SHIFT))
#define PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_MASK                         \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_EN_ILIM_FSM_CTRL_SHIFT))
#define PMIC_CONFIG_1_NSLEEP1_MASK_MASK                             \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_NSLEEP1_MASK_SHIFT))
#define PMIC_CONFIG_1_NSLEEP2_MASK_MASK                             \
                         ((uint8_t)(0x01U << PMIC_CONFIG_1_NSLEEP2_MASK_SHIFT))

/*!
 * \brief   PMIC StartUp NSLEEP Mask Values
 */
#define PMIC_STARTUP_DEST_NSLEEP2B_MASK                            \
                         ((uint8_t)(0x01U << PMIC_STARTUP_DEST_NSLEEP2B_SHIFT))
#define PMIC_STARTUP_DEST_NSLEEP1B_MASK                            \
                         ((uint8_t)(0x01U << PMIC_STARTUP_DEST_NSLEEP1B_SHIFT))

/*!
 * \brief: PMIC Recovery Counter Threshold Max Value
 */
#define PMIC_RECOV_CNT_THR_MAX                          (0x0FU)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/*!
 *  \brief   This function is used to write a specific bit field value
 */
static inline void Pmic_setBitField(uint8_t *pRegVal,
                                    uint8_t  regFieldShift,
                                    uint8_t  regFieldMask,
                                    uint8_t  fieldVal)
{
    *pRegVal = (((*pRegVal) & (uint8_t) (~(uint8_t) regFieldMask)) |  \
               ((((uint8_t) fieldVal) << (uint8_t) regFieldShift)  &  \
                  (uint8_t) regFieldMask));
}

/*!
 * \brief   This function is used to read a specific bit field value
 */
static inline uint8_t Pmic_getBitField(uint8_t regData,
                                       uint8_t regFieldShift,
                                       uint8_t regFieldMask)
{
   uint8_t fieldVal;

    fieldVal = (((regData) & (uint8_t) regFieldMask) >>  \
                 (uint8_t) regFieldShift);

   return fieldVal;
}

/*!
 * \brief: Checks the validParam bit position is set or not in validParamVal
 *         This function checks the given bit position is being set or not in
 *         the validParamVal argument
 *
 *  \param   validParamVal [IN]   Valid param value
 *  \param   bitPos        [IN]   bit position value
 *
 *  \retval  Return true if the given bit pos is set, else return false
 */
bool pmic_validParamCheck(uint32_t validParamVal, uint8_t bitPos);

/*!
 * \brief: Function call wrapper to lock PMIC LLD critical section
 *         This function locks to critical area by calling registred locking
 *         mechanism using pmic core handle.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pPmicCoreHandle);

/*!
 * \brief: Function call wrapper to unlock PMIC LLD critical section
 *         This function unlocks to critical area by calling registred locking
 *         mechanism using pmic core handle.
 *
 *  \param   pPmicCoreHandle  [IN]  PMIC Interface Handle
 */
void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pPmicCoreHandle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_PRIV_H_ */
