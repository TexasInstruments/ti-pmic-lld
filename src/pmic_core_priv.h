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
/*!  PMIC TI Device ID register Address */
#define PMIC_DEV_REV_REGADDR                (0x01U)
/*!  PMIC TI NVM ID register Address */
#define PMIC_NVM_CODE_1_REGADDR             (0x02U)
/*!  PMIC TI NVM Revision register Address */
#define PMIC_NVM_CODE_2_REGADDR             (0x03U)
/*!  PMIC TI Silicon Revision register Address */
#define PMIC_MANUFACTURING_VER_REGADDR      (0xA6U)
/*!  PMIC CUSTOMER NVM ID register Address */
#define PMIC_CUSTOMER_NVM_ID_REG_REGADDR    (0xA7U)

#define PMIC_WDG_LONGWIN_CFG_REGADDR        (0x405U)

/*!
 * \brief  PMIC STAT_STARTUP register Addresses
 */
#define PMIC_STAT_STARTUP_REGADDR           (0x73U)

/*!
 * \brief  PMIC STAT_READBACK_ERR register Addresses
 */
#define PMIC_STAT_READBACK_ERR_REGADDR      (0x77U)

/*!
 * \brief  PMIC PLL_CTRL register Addresses
 */
#define PMIC_PLL_CTRL_REGADDR               (0x7CU)

/*!
 * \brief  PMIC power Configuration Register Address
 */
#define PMIC_CONFIG_1_REGADDR               (0x7DU)

/*!
 * \brief  PMIC CONFIG_2 register Addresses
 */
#define PMIC_CONFIG_2_REGADDR               (0x7EU)

/*!
 * \brief  PMIC ENABLE_DRV register Addresses
 */
#define PMIC_ENABLE_DRV_REG_REGADDR         (0x80U)

/*!
 * \brief   PMIC MISC Control Register Address
 */
#define PMIC_MISC_CTRL_REGADDR              (0x81U)

/*!
 * \brief  PMIC ENABLE_DRV_STAT register Addresses
 */
#define PMIC_ENABLE_DRV_STAT_REGADDR        (0x82U)

/*!
 * \brief: PMIC Recovery Counter Control and Status Registers
 */
#define PMIC_RECOV_CNT_REG_1_REGADDR        (0x83U)
#define PMIC_RECOV_CNT_REG_2_REGADDR        (0x84U)

/*!
 * \brief  PMIC Spread Spectrum 1 register Addresses
 */
#define PMIC_SPREAD_SPECTRUM_1_REGADDR      (0x88U)

/*!
 * \brief  PMIC User Spare register Addresses
 */
#define PMIC_USER_SPARE_REGS_REGADDR        (0x8EU)

/*!
 * \brief  PMIC Register Lock register Addresses
 */
#define PMIC_REGISTER_LOCK_REGADDR          (0xA1U)

/*!
 * \brief  PMIC Soft Reboot register Addresses
 */
#define PMIC_SOFT_REBOOT_REG_REGADDR        (0xABU)

/*!
 * \brief: HERA PMIC StartUP Control Register
 */
#define PMIC_STARTUP_CTRL_REGADDR           (0xC3U)

/*!
 * \brief  PMIC Scratchpad register Addresses
 */
#define PMIC_SCRATCH_PAD_REG_1_REGADDR      (0xC9U)
#define PMIC_SCRATCH_PAD_REG_2_REGADDR      (0xCAU)
#define PMIC_SCRATCH_PAD_REG_3_REGADDR      (0xCBU)
#define PMIC_SCRATCH_PAD_REG_4_REGADDR      (0xCCU)

/*!
 * \brief  PMIC invalid register address
 */
#define PMIC_INVALID_REGADDR                (0xFFU)

/*!
 * \brief: PMIC SERIAL_IF_CONFIG register address (Bank/Page 1 Register address)
 *         Application can only read this register to check I2C1SPI/I2C2 CRC
 *         is enabled or not
 */
#define PMIC_SERIAL_IF_CONFIG_REGADDR           (0x11AU)

/*!
 * \brief  PMIC invalid BIT SHIFT value
 */
#define PMIC_INVALID_BIT_SHIFT              (0xFFU)

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
#define PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_SHIFT     (0x02U)
#define PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT              (0x03U)
#define PMIC_STARTUP_CTRL_FAST_BIST_SHIFT                   (0x04U)
#define PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT                (0x05U)
#define PMIC_STARTUP_CTRL_FIRST_STARTUP_DONE_SHIFT          (0x07U)

/*!
 * \brief   PMIC StartUp NSLEEP Shift Values
 */
#define PMIC_STARTUP_DEST_NSLEEP2B_SHIFT              (0x1U)
#define PMIC_STARTUP_DEST_NSLEEP1B_SHIFT              (0x0U)

/*!
 * \brief   PMIC MANUFACTURING_VER Register Shift Values
 */
#define PMIC_MANUFACTURING_VER_SILICON_REV_SHIFT      (0x0U)

/*!
 * \brief   PMIC User Spare Register Shift Values
 */
#define PMIC_USER_SPARE_REGS_USER_SPARE_1_SHIFT       (0x0U)
#define PMIC_USER_SPARE_REGS_USER_SPARE_2_SHIFT       (0x1U)
#define PMIC_USER_SPARE_REGS_USER_SPARE_3_SHIFT       (0x2U)
#define PMIC_USER_SPARE_REGS_USER_SPARE_4_SHIFT       (0x3U)

/*!
 * \brief   PMIC ENABLE_DRV Register Shift Values
 */
#define PMIC_ENABLE_DRV_REG_ENABLE_DRV_SHIFT         (0x0U)

/*!
 * \brief   PMIC ENABLE_DRV_STAT Register Shift Values
 */
#define PMIC_ENABLE_DRV_STAT_EN_DRV_IN_SHIFT            (0x0U)
#define PMIC_ENABLE_DRV_STAT_NRSTOUT_IN_SHIFT           (0x1U)
#define PMIC_ENABLE_DRV_STAT_NRSTOUT_SOC_IN_SHIFT       (0x2U)
#define PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_SHIFT     (0x3U)
#define PMIC_ENABLE_DRV_STAT_SPMI_LPM_EN_SHIFT          (0x4U)

/*!
 * \brief   PMIC Spread Spectrum 1 Register Shift Values
 */
#define PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_SHIFT           (0x0U)
#define PMIC_SPREAD_SPECTRUM_1_SS_EN_SHIFT              (0x2U)

/*!
 * \brief   PMIC Register Lock Register Shift Values
 */
#define PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_SHIFT    (0x0U)

/*!
 * \brief   PMIC MISC Control Register Shift Values
 */
#define PMIC_MISC_CTRL_NRSTOUT_SHIFT                (0x0U)
#define PMIC_MISC_CTRL_NRSTOUT_SOC_SHIFT            (0x1U)
#define PMIC_MISC_CTRL_LPM_EN_SHIFT                 (0x2U)
#define PMIC_MISC_CTRL_CLKMON_EN_SHIFT              (0x3U)
#define PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT      (0x4U)
#define PMIC_MISC_CTRL_SEL_EXT_CLK_SHIFT            (0x5U)
#define PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_SHIFT    (0x6U)

/*!
 * \brief   PMIC PLL Control Register Shift Values
 */
#define PMIC_PLL_CTRL_EXT_CLK_FREQ_SHIFT            (0x0U)

/*!
 * \brief   PMIC CONFIG_2 Register Shift Values
 */
#define PMIC_CONFIG_2_BB_CHARGER_EN_SHIFT           (0x0U)
#define PMIC_CONFIG_2_BB_ICHR_SHIFT                 (0x1U)
#define PMIC_CONFIG_2_BB_VEOC_SHIFT                 (0x2U)
#define PMIC_CONFIG_2_BB_EOC_RDY_SHIFT              (0x7U)

/*!
 * \brief  PMIC External Clock Validity status register bit field Shift Values
 */
#define PMIC_STAT_MISC_EXT_CLK_STAT_SHIFT           (0x1U)

/*!
 * \brief  PMIC ENABLE_STAT register bit field Shift Values
 */
#define PMIC_STAT_STARTUP_ENABLE_STAT_SHIFT           (0x1U)

/*!
 * \brief  PMIC STAT_READBACK_ERR register Shift Values
 */
#define PMIC_STAT_READBACK_ERR_EN_DRV_READBACK_STAT_SHIFT        (0x0U)
#define PMIC_STAT_READBACK_ERR_NINT_READBACK_STAT_SHIFT          (0x1U)
#define PMIC_STAT_READBACK_ERR_NRSTOUT_READBACK_STAT_SHIFT       (0x2U)
#define PMIC_STAT_READBACK_ERR_NRSTOUT_SOC_READBACK_STAT_SHIFT   (0x3U)

/*!
 * \brief  PMIC SERIAL_IF_CONFIG register Shift Values
 */
#define PMIC_SERIAL_IF_CONFIG_I2C1_SPI_CRC_EN_SHIFT              (0x1U)
#define PMIC_SERIAL_IF_CONFIG_I2C2_CRC_EN_SHIFT                  (0x2U)

/*!
 * \brief   PMIC DEV_REV Register Shift Values
 */
/* Valid only PG 2.0 */
#define PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_SHIFT        (0x1U)
/* Valid only PG 1.0 */
#define PMIC_DEV_REV_TI_DEVICE_ID_SILICON_REV_SHIFT               (0x0U)

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
#define PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_MASK                   \
           ((uint8_t)(0x01U << PMIC_STARTUP_CTRL_SKIP_LP_STANDBY_EE_READ_SHIFT))
#define PMIC_STARTUP_CTRL_LP_STANDBY_SEL_MASK                   \
           ((uint8_t)(0x01U << PMIC_STARTUP_CTRL_LP_STANDBY_SEL_SHIFT))
#define PMIC_STARTUP_CTRL_FAST_BIST_MASK                   \
           ((uint8_t)(0x01U << PMIC_STARTUP_CTRL_FAST_BIST_SHIFT))
#define PMIC_STARTUP_CTRL_STARTUP_DEST_MASK                     \
           ((uint8_t)(0x03U << PMIC_STARTUP_CTRL_STARTUP_DEST_SHIFT))
#define PMIC_STARTUP_CTRL_FIRST_STARTUP_DONE_MASK                     \
           ((uint8_t)(0x01U << PMIC_STARTUP_CTRL_FIRST_STARTUP_DONE_SHIFT))

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
 * \brief   PMIC MANUFACTURING_VER Register Mask Values
 */
#define PMIC_MANUFACTURING_VER_SILICON_REV_MASK                            \
                  ((uint8_t)(0x3FU << PMIC_MANUFACTURING_VER_SILICON_REV_SHIFT))

/*!
 * \brief   PMIC User Spare Register Mask Values
 */
#define PMIC_USER_SPARE_REGS_USER_SPARE_1_MASK                            \
                  ((uint8_t)(0x01U << PMIC_USER_SPARE_REGS_USER_SPARE_1_SHIFT))
#define PMIC_USER_SPARE_REGS_USER_SPARE_2_MASK                            \
                  ((uint8_t)(0x01U << PMIC_USER_SPARE_REGS_USER_SPARE_2_SHIFT))
#define PMIC_USER_SPARE_REGS_USER_SPARE_3_MASK                            \
                  ((uint8_t)(0x01U << PMIC_USER_SPARE_REGS_USER_SPARE_3_SHIFT))
#define PMIC_USER_SPARE_REGS_USER_SPARE_4_MASK                            \
                  ((uint8_t)(0x01U << PMIC_USER_SPARE_REGS_USER_SPARE_4_SHIFT))

/*!
 * \brief   PMIC Spread Spectrum 1 Register Mask Values
 */
#define PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_MASK                            \
                  ((uint8_t)(0x03U << PMIC_SPREAD_SPECTRUM_1_SS_DEPTH_SHIFT))
#define PMIC_SPREAD_SPECTRUM_1_SS_EN_MASK                            \
                  ((uint8_t)(0x01U << PMIC_SPREAD_SPECTRUM_1_SS_EN_SHIFT))

/*!
 * \brief   PMIC ENABLE_DRV Register Mask Values
 */
#define PMIC_ENABLE_DRV_REG_ENABLE_DRV_MASK                            \
                  ((uint8_t)(0x01U << PMIC_ENABLE_DRV_REG_ENABLE_DRV_SHIFT))

/*!
 * \brief   PMIC ENABLE_DRV_STAT Register Mask Values
 */
#define PMIC_ENABLE_DRV_STAT_EN_DRV_IN_MASK                            \
               ((uint8_t)(0x01U << PMIC_ENABLE_DRV_STAT_EN_DRV_IN_SHIFT))
#define PMIC_ENABLE_DRV_STAT_NRSTOUT_IN_MASK                            \
               ((uint8_t)(0x01U << PMIC_ENABLE_DRV_STAT_NRSTOUT_IN_SHIFT))
#define PMIC_ENABLE_DRV_STAT_NRSTOUT_SOC_IN_MASK                            \
               ((uint8_t)(0x01U << PMIC_ENABLE_DRV_STAT_NRSTOUT_SOC_IN_SHIFT))
#define PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_MASK                            \
               ((uint8_t)(0x01U << PMIC_ENABLE_DRV_STAT_FORCE_EN_DRV_LOW_SHIFT))
#define PMIC_ENABLE_DRV_STAT_SPMI_LPM_EN_MASK                            \
               ((uint8_t)(0x01U << PMIC_ENABLE_DRV_STAT_SPMI_LPM_EN_SHIFT))

/*!
 * \brief   PMIC Register Lock Register Mask Values
 */
/*!  PMIC Register Lock Register Mask Values to read the register lock status */
#define PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_READ_MASK              \
             ((uint8_t)(0x01U << PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_SHIFT))

/*!  PMIC Register Lock Register Mask Values to write lock/unlock value to
 *   register lock register */
#define PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_WRITE_MASK             \
             ((uint8_t)(0xFFU << PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_SHIFT))



/*!
 * \brief   PMIC MISC Control Register Mask Values
 */
#define PMIC_MISC_CTRL_NRSTOUT_MASK              \
                  ((uint8_t)(0x01U << PMIC_MISC_CTRL_NRSTOUT_SHIFT))
#define PMIC_MISC_CTRL_NRSTOUT_SOC_MASK              \
                  ((uint8_t)(0x01U << PMIC_MISC_CTRL_NRSTOUT_SOC_SHIFT))
#define PMIC_MISC_CTRL_LPM_EN_MASK              \
                  ((uint8_t)(0x01U << PMIC_MISC_CTRL_LPM_EN_SHIFT))
#define PMIC_MISC_CTRL_CLKMON_EN_MASK              \
                  ((uint8_t)(0x01U << PMIC_MISC_CTRL_CLKMON_EN_SHIFT))
#define PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_MASK              \
                  ((uint8_t)(0x01U << PMIC_MISC_CTRL_AMUXOUT_REFOUT_EN_SHIFT))
#define PMIC_MISC_CTRL_SEL_EXT_CLK_MASK              \
                  ((uint8_t)(0x01U << PMIC_MISC_CTRL_SEL_EXT_CLK_SHIFT))
#define PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_MASK              \
                  ((uint8_t)(0x03U << PMIC_MISC_CTRL_SYNCCLKOUT_FREQ_SEL_SHIFT))

/*!
 * \brief   PMIC PLL Control Register Mask Values
 */
#define PMIC_PLL_CTRL_EXT_CLK_FREQ_MASK              \
                  ((uint8_t)(0x03U << PMIC_PLL_CTRL_EXT_CLK_FREQ_SHIFT))


/*!
 * \brief   PMIC CONFIG_2 Register Mask Values
 */
#define PMIC_CONFIG_2_BB_CHARGER_EN_MASK              \
                  ((uint8_t)(0x01U << PMIC_CONFIG_2_BB_CHARGER_EN_SHIFT))
#define PMIC_CONFIG_2_BB_ICHR_MASK              \
                  ((uint8_t)(0x01U << PMIC_CONFIG_2_BB_ICHR_SHIFT))
#define PMIC_CONFIG_2_BB_VEOC_MASK              \
                  ((uint8_t)(0x03U << PMIC_CONFIG_2_BB_VEOC_SHIFT))
#define PMIC_CONFIG_2_BB_EOC_RDY_MASK              \
                  ((uint8_t)(0x01U << PMIC_CONFIG_2_BB_EOC_RDY_SHIFT))

/*!
 * \brief  PMIC Extrnal Clock Validity status Mask Values
 */
#define PMIC_STAT_MISC_EXT_CLK_STAT_MASK              \
                  ((uint8_t)(0x01U << PMIC_STAT_MISC_EXT_CLK_STAT_SHIFT))

/*!
 * \brief  PMIC ENABLE_STAT register bit field Mask Values
 */
#define PMIC_STAT_STARTUP_ENABLE_STAT_MASK              \
                  ((uint8_t)(0x01U << PMIC_STAT_STARTUP_ENABLE_STAT_SHIFT))

/*!
 * \brief  PMIC STAT_READBACK_ERR register Mask Values
 */
#define PMIC_STAT_READBACK_ERR_EN_DRV_READBACK_STAT_MASK              \
    ((uint8_t)(0x01U << PMIC_STAT_READBACK_ERR_EN_DRV_READBACK_STAT_SHIFT))
#define PMIC_STAT_READBACK_ERR_NINT_READBACK_STAT_MASK              \
    ((uint8_t)(0x01U << PMIC_STAT_READBACK_ERR_NINT_READBACK_STAT_SHIFT))
#define PMIC_STAT_READBACK_ERR_NRSTOUT_READBACK_STAT_MASK              \
    ((uint8_t)(0x01U << PMIC_STAT_READBACK_ERR_NRSTOUT_READBACK_STAT_SHIFT))
#define PMIC_STAT_READBACK_ERR_NRSTOUT_SOC_READBACK_STAT_MASK              \
    ((uint8_t)(0x01U << PMIC_STAT_READBACK_ERR_NRSTOUT_SOC_READBACK_STAT_SHIFT))

/*!
 * \brief  PMIC SERIAL_IF_CONFIG register Mask Values
 */
#define PMIC_SERIAL_IF_CONFIG_I2C1_SPI_CRC_EN_MASK              \
    ((uint8_t)(0x01U << PMIC_SERIAL_IF_CONFIG_I2C1_SPI_CRC_EN_SHIFT))
#define PMIC_SERIAL_IF_CONFIG_I2C2_CRC_EN_MASK              \
    ((uint8_t)(0x01U << PMIC_SERIAL_IF_CONFIG_I2C2_CRC_EN_SHIFT))

/*!
 * \brief   PMIC DEV_REV Register Mask Values
 */
/* Valid only PG 2.0 */
#define PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_MASK              \
    ((uint8_t)(0x7FU << PMIC_DEV_REV_TI_DEVICE_ID_PG_2_0_SILICON_REV_SHIFT))
/* Valid only PG 1.0 */
#define PMIC_DEV_REV_TI_DEVICE_ID_SILICON_REV_MASK              \
    ((uint8_t)(0xFFU << PMIC_DEV_REV_TI_DEVICE_ID_SILICON_REV_SHIFT))

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
