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
 *  @file pmic_core_tps6594x.h
 *
 *  @brief  The macro definitions for TPS6594x Leo PMIC driver specific
 *          PMIC common configuration
 */

#ifndef PMIC_CORE_TPS65386X_H_
#define PMIC_CORE_TPS65386X_H_

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Sub address command as per tps653860xx datasheet */
#define CRC_POLY (0x107U)
#define CRC_LEN (9U)
#define CMD_SHIFT (24U)
#define RW_SHIFT (16U)
#define DAT_SHIFT (8U)
#define CRC_SHIFT (0U)
#define CMD_RD_EN (0x10)
#define CMD_WR_EN (0x00)

#define CMD_DEVICE_ID (0x00)
#define CMD_REV_ID (0x01)

#define CMD_REG_UNLOCK (0x03)
#define CNT_REG_UNLOCK (0x04)
#define CMD_REG_STATUS (0x09)

#define CMD_LDO2_CFG (0x1F)
#define CMD_LDO3_CFG (0x20)
#define CMD_LDO4_CFG (0x21)
#define CMD_LDO_CTRL (0x26)

#define DAT_REG_UNLOCK_1 (0x98)
#define DAT_REG_UNLOCK_2 (0xB8)

#define DAT_REG_LOCK (0x10)

#define CNT_REG_UNLOCK_1 (0x13)
#define CNT_REG_UNLOCK_2 (0x7D)

#define DAT_LDO2_CFG (0x93)
#define DAT_LDO3_CFG (0x94)
#define DAT_LDO4_CFG (0x90)
#define DAT_LDO_CTRL (0x55)

#define CFG_REG_LOCK_MASK (0x01) /* Bit mask for CFG_REG_LOCK */
#define CNT_REG_LOCK_MASK (0x02) /* Bit mask for CNT_REG_LOCK */

#define PMIC_MCSPI_MSGSIZE (1U)

#define PMIC_TPS65386X_DEV_ID (0x3CU)

/**
 *  @name   PMIC DIAG OUT Pin Control Configuration
 *
 *  @{
 */
/** @brief AMUX Enabled */
#define PMIC_TPS65386X_DIAG_OUT_AMUX_ENABLE (1U)
/** @brief DMUX Enabled */
#define PMIC_TPS65386X_DIAG_OUT_DMUX_ENABLE (2U)
/** @brief AMUX/DMUX Disabled */
#define PMIC_TPS65386X_DIAG_OUT_DISABLE (0U)
/*  @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_CORE_TPS65386X_H_ */
