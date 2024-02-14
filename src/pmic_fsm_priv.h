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
 *   @file    pmic_fsm_priv.h
 *
 *   @brief   This file contains the private MACRO's and function definitions
 * for PMIC FSM state configuration
 *
 */

#ifndef PMIC_INC_PMIC_FSM_PRIV_H_
#define PMIC_INC_PMIC_FSM_PRIV_H_

#define PMIC_FSM_STATE_MAX (6)

#define PMIC_STATE_CTRL_REGADDR (0x16U)
#define PMIC_STATE_CTRL_SHIFT (0x0U)
#define PMIC_STATE_CTRL_MASK (0x07U)
#define PMIC_STATE_CTRL_NO_CHANGE (0x00U)
#define PMIC_STATE_CTRL_SAFE_TO_ACTIVE (0x01U)
#define PMIC_STATE_CTRL_ACTIVE_TO_SAFE (0x02U)
#define PMIC_STATE_CTRL_RESET_MCU (0x03U)
#define PMIC_STATE_CTRL_STANDBY (0x04U)
#define PMIC_STATE_CTRL_OFF (0x05U)

#define PMIC_STATE_STAT_REGADDR (0x17U)
#define PMIC_STATE_STAT_MASK (0x0FU)
#define PMIC_STATE_STAT_OFF_STATE (0x00U)
#define PMIC_STATE_STAT_INIT_STATE (0x01U | 0x02U | 0x03U | 0x04U)
#define PMIC_STATE_STAT_PWRU_SEQ_STATE (0x05U)
#define PMIC_STATE_STAT_RESET_MCU_STATE (0x06U)
#define PMIC_STATE_STAT_AUTO_BIST_STATE (0x07U)
#define PMIC_STATE_STAT_ACTIVE_STATE (0x08U)
#define PMIC_STATE_STAT_SAFE_STATE (0x09U)
#define PMIC_STATE_STAT_RT_BIST_STATE (0x0AU)
#define PMIC_STATE_STAT_PWRD_SEQ_STATE (0x0CU)
#define PMIC_STATE_STAT_STANDBY_STATE (0x0DU)

#define PMIC_STBY_CFG_REGADDR (0x19U)
#define PMIC_STBY_CFG_STBY_SEL_SHIFT (0x04U)
#define PMIC_STBY_CFG_STBY_SEL_MASK (0x01U << PMIC_STBY_CFG_STBY_SEL_SHIFT)
#define PMIC_STBY_CFG_STBY_VAL (0x01U << PMIC_STBY_CFG_STBY_SEL_SHIFT)

#define PMIC_WAKE_CFG_REGADDR (0x12U)
#define PMIC_WAKE1_DGL_CFG_SHIFT (0x00U)
#define PMIC_WAKE2_DGL_CFG_SHIFT (0x01U)
#define PMIC_WAKE1_DGL_CFG_MASK (uint8_t)(0x01U << PMIC_WAKE1_DGL_CFG_SHIFT)
#define PMIC_WAKE2_DGL_CFG_MASK (uint8_t)(0x01U << PMIC_WAKE2_DGL_CFG_SHIFT)

#define PMIC_BIST_CTRL_REGADDR (0x53U)
#define PMIC_BIST_CTRL_EN_SHIFT (0x04U)
#define PMIC_BIST_CTRL_CFG_MASK (0x07U)

#define PMIC_ILIM_CFG_REGADDR (0x60U)
#define PMIC_ILIM_LDO1_CFG_MASK (0x01U)
#define PMIC_ILIM_LDO2_CFG_MASK (0x02U)
#define PMIC_ILIM_LDO3_CFG_MASK (0x04U)
#define PMIC_ILIM_LDO4_CFG_MASK (0x08U)
#define PMIC_ILIM_PLDO1_CFG_MASK (0x10U)
#define PMIC_ILIM_PLDO2_CFG_MASK (0x2FU)
#define PMIC_BB_ILIM_LVL_CFG_MASK (0x08U)

#define PMIC_DCDC_STAT_REGADDR (0x5AU)

#define PMIC_FSM_OFF_STATE (0U)
#define PMIC_FSM_STANBY_STATE (1U)
#define PMIC_FSM_SAFE_STATE (2U)
#define PMIC_FSM_ACTIVE_STATE (3U)
#define PMIC_FSM_MCU_ONLY_STATE (4U)
#define PMIC_FSM_S2R_STATE (5U)

#define PMIC_FSM_STAT_OFF_1 (0U)
#define PMIC_FSM_STAT_INIT_1 (1U)
#define PMIC_FSM_STAT_INIT_2 (2U)
#define PMIC_FSM_STAT_INIT_3 (3U)
#define PMIC_FSM_STAT_INIT_4 (4U)
#define PMIC_FSM_STAT_PWRU_SEQ (5U)
#define PMIC_FSM_STAT_RESET_MCU (6U)
#define PMIC_FSM_STAT_AUTO_BIST (7U)
#define PMIC_FSM_STAT_ACTIVE (8U)
#define PMIC_FSM_STAT_SAFE (9U)
#define PMIC_FSM_STAT_RT_BIST (10U)
#define PMIC_FSM_STAT_OTP (11U)
#define PMIC_FSM_STAT_PWRD_SEQ (12U)
#define PMIC_FSM_STAT_STANDBY (13U)
#define PMIC_FSM_STAT_OFF_2 (14U)

#define PMIC_FSM_FAST_BIST_ENABLE (1U)
#define PMIC_FSM_FAST_BIST_DISABLE (0U)

#define PMIC_FSM_SELECT_LPSTANDBY_STATE (1U)
#define PMIC_FSM_SELECT_STANDBY_STATE (0U)

#define PMIC_NSLEEP1_SIGNAL (bool)false
#define PMIC_NSLEEP2_SIGNAL (bool)true

#define PMIC_FSM_ILIM_INT_FSMCTRL_ENABLE (1U)
#define PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE (0U)

#define PMIC_FSM_CFG_FAST_BIST_EN_VALID (0U)
#define PMIC_FSM_CFG_LP_STANDBYSEL_VALID (1U)
#define PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID (2U)
#define PMIC_FSM_CFG_FSM_STARTUP_DEST_SEL_VALID (3U)
#define PMIC_FSM_STARTUPDEST_ACTIVE (3U)

#define PMIC_FSM_CFG_FAST_BIST_EN_VALID (0U)
#define PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID (2U)

#define PMIC_FSM_CFG_FAST_BIST_EN_VALID_SHIFT                                  \
  (1U << PMIC_FSM_CFG_FAST_BIST_EN_VALID)
#define PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID_SHIFT                           \
  (1U << PMIC_FSM_CFG_ILIM_INT_FSMCTRL_EN_VALID)

#endif /* PMIC_INC_PMIC_FSM_PRIV_H_ */
