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
 *  \file   pmic_fsm_priv.h
 *
 *  \brief  The macro definitions for configuring PMIC FSM Registers
 *
 */

#ifndef PMIC_FSM_PRIV_H_
#define PMIC_FSM_PRIV_H_

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/*!
 * \brief   PMIC FSM I2C TRIGGER Register Address
 */
#define PMIC_FSM_I2C_TRIGGERS_REGADDR          (0x85U)

/*!
 * \brief   PMIC FSM NSLEEP TRIGGER Register Address
 */
#define PMIC_FSM_NSLEEP_TRIGGERS_REGADDR       (0x86U)

/*!
 * \brief   PMIC MISC Control Register Address
 */
#define PMIC_FSM_MISC_CTRL_REGADDR         (0x81U)

/*!
 * \brief   PMIC PFSM Delay Register Address
 */
#define PMIC_FSM_PFSM_DELAY_REG_1_REGADDR      (0xCDU)
#define PMIC_FSM_PFSM_DELAY_REG_2_REGADDR      (0xCEU)
#define PMIC_FSM_PFSM_DELAY_REG_3_REGADDR      (0xCFU)
#define PMIC_FSM_PFSM_DELAY_REG_4_REGADDR      (0xD0U)

/*!
 * \brief   PMIC MISC Control Register Bit Fields
 */
#define PMIC_FSM_MISC_CTRL_LPM_EN_SHIFT    (0x2U)

/*!
 * \brief   PMIC FSM I2C TRIGGER Register Bit Fields
 */
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_SHIFT    (0x0U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_SHIFT    (0x1U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_2_SHIFT    (0x2U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_3_SHIFT    (0x3U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_4_SHIFT    (0x4U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_5_SHIFT    (0x5U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_6_SHIFT    (0x6U)
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_7_SHIFT    (0x7U)

/*!
 * \brief   PMIC FSM NSLEEP1/2 Register Bit Fields
 */
#define PMIC_FSM_NSLEEPX_SET           (0x1U)
#define PMIC_FSM_NSLEEPX_RESET         (0x0U)

/**
 *  \anchor Pmic_Fsm_Mission_State_Max
 *  \name   Maximum limit for mission state
 *
 *  @{
 */
#define PMIC_FSM_STATE_MAX       PMIC_FSM_S2R_STATE
/* @} */

/*!
 * \brief   PMIC FSM NSLEEP TRIGGER Register Bit Fields
 */
#define PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT    (0x0U)
#define PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_SHIFT    (0x1U)

/*!
 * \brief   PMIC MISC Control Register Bit Mask
 */
#define PMIC_FSM_MISC_CTRL_LPM_EN_MASK              ((uint8_t)  \
                                     (0x01U << PMIC_FSM_MISC_CTRL_LPM_EN_SHIFT))

/*!
 * \brief   PMIC FSM NSLEEP TRIGGER Register Bit Mask
 */
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_0_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_1_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_2_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_2_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_3_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_3_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_4_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_4_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_5_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_5_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_6_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_6_SHIFT))
#define PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_7_MASK    ((uint8_t)  \
                           (0x01U << PMIC_FSM_I2C_TRIGGERS_TRIGGER_I2C_7_SHIFT))
/*!
 * \brief   PMIC FSM NSLEEP TRIGGER Register Bit Mask
 */
#define PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_MASK  ((uint8_t)  \
                            (0x01U << PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP1B_SHIFT))
#define PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_MASK  ((uint8_t)  \
                            (0x01U << PMIC_FSM_NSLEEP_TRIGGERS_NSLEEP2B_SHIFT))

/**
 *  \anchor Pmic_Fsm_Pfsm_Delay_Step_Max
 *  \name   Maximum limit of PFSM Delay Step
 *
 *  @{
 */
#define PMIC_FSM_PFSM_DELAY_STEP_MAX       (31U)
/* @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* PMIC_FSM_PRIV_H_ */
