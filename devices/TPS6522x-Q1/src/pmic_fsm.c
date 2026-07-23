/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_fsm.h"
#include "pmic_io.h"
#include "regmap/fsm.h"

#include <string.h>

/* ========================================================================== */
/*                           Internal Helper Functions                        */
/* ========================================================================== */

/**
 * @brief Copy Pmic_FsmTriggerCfg_t structure member-wise
 */
static inline void FSM_copyFsmTriggerCfg(const Pmic_FsmTriggerCfg_t *src, Pmic_FsmTriggerCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmTriggerCfg_t));
}

/**
 * @brief Copy Pmic_FsmGpioTriggerCfg_t structure member-wise
 */
static inline void FSM_copyFsmGpioTriggerCfg(const Pmic_FsmGpioTriggerCfg_t *src, Pmic_FsmGpioTriggerCfg_t *dst)
{
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmGpioTriggerCfg_t));
}

/* GPIO pin mapping table entry */
typedef struct
{
    uint16_t regAddr;
    uint8_t maskShift;
    uint8_t maskPolShift;
} Pmic_FsmGpioPinMap_t;

/* Get GPIO pin mapping from table. */
static int32_t Pmic_fsmGetGpioPinMapping(uint8_t pinNum, const Pmic_FsmGpioPinMap_t **mapping)
{
    int32_t status = PMIC_ST_SUCCESS;

    /* GPIO pin to register mapping table */
    static const Pmic_FsmGpioPinMap_t gpioPinMap[] = {
        { (uint16_t)FSM_TRIG_MASK_1_REG, (uint8_t)GPIO1_FSM_MASK_SHIFT, (uint8_t)GPIO1_FSM_MASK_POL_SHIFT },  /* GPIO1 */
        { (uint16_t)FSM_TRIG_MASK_1_REG, (uint8_t)GPIO2_FSM_MASK_SHIFT, (uint8_t)GPIO2_FSM_MASK_POL_SHIFT },  /* GPIO2 */
        { (uint16_t)FSM_TRIG_MASK_1_REG, (uint8_t)GPIO3_FSM_MASK_SHIFT, (uint8_t)GPIO3_FSM_MASK_POL_SHIFT },  /* GPIO3 */
        { (uint16_t)FSM_TRIG_MASK_1_REG, (uint8_t)GPIO4_FSM_MASK_SHIFT, (uint8_t)GPIO4_FSM_MASK_POL_SHIFT },  /* GPIO4 */
        { (uint16_t)FSM_TRIG_MASK_2_REG, (uint8_t)GPIO5_FSM_MASK_SHIFT, (uint8_t)GPIO5_FSM_MASK_POL_SHIFT },  /* GPIO5 */
        { (uint16_t)FSM_TRIG_MASK_2_REG, (uint8_t)GPIO6_FSM_MASK_SHIFT, (uint8_t)GPIO6_FSM_MASK_POL_SHIFT }   /* GPIO6 */
    };

    if ((pinNum < PMIC_FSM_GPIO_PIN_MIN) || (pinNum > PMIC_FSM_GPIO_PIN_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        *mapping = &gpioPinMap[pinNum - PMIC_FSM_GPIO_PIN_MIN];
    }

    return status;
}

/* Validate FSM trigger configuration parameters. */
static int32_t Pmic_fsmValidateTriggerCfg(const Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SEVERE_ERR_TRIG_VALID) &&
        (triggerCfg->severeErrTrig > PMIC_FSM_TRIGGER_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_OTHER_RAIL_TRIG_VALID) &&
        (triggerCfg->otherRailTrig > PMIC_FSM_TRIGGER_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SOC_RAIL_TRIG_VALID) &&
        (triggerCfg->socRailTrig > PMIC_FSM_TRIGGER_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MCU_RAIL_TRIG_VALID) &&
        (triggerCfg->mcuRailTrig > PMIC_FSM_TRIGGER_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MODERATE_ERR_TRIG_VALID) &&
        (triggerCfg->moderateErrTrig > PMIC_FSM_TRIGGER_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

/* Configure FSM_TRIG_SEL_1 register. */
static int32_t Pmic_fsmSetTrigSel1(const Pmic_Handle_t *handle, const Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SEVERE_ERR_TRIG_VALID) ||
        Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_OTHER_RAIL_TRIG_VALID) ||
        Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SOC_RAIL_TRIG_VALID) ||
        Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MCU_RAIL_TRIG_VALID))
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_1_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SEVERE_ERR_TRIG_VALID))
            {
                Pmic_setBitField(&regData, SEVERE_ERR_TRIG_SHIFT, SEVERE_ERR_TRIG_MASK, triggerCfg->severeErrTrig);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_OTHER_RAIL_TRIG_VALID))
            {
                Pmic_setBitField(&regData, OTHER_RAIL_TRIG_SHIFT, OTHER_RAIL_TRIG_MASK, triggerCfg->otherRailTrig);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SOC_RAIL_TRIG_VALID))
            {
                Pmic_setBitField(&regData, SOC_RAIL_TRIG_SHIFT, SOC_RAIL_TRIG_MASK, triggerCfg->socRailTrig);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MCU_RAIL_TRIG_VALID))
            {
                Pmic_setBitField(&regData, MCU_RAIL_TRIG_SHIFT, MCU_RAIL_TRIG_MASK, triggerCfg->mcuRailTrig);
            }

            status = Pmic_ioTxByte(handle, FSM_TRIG_SEL_1_REG, regData);
        }
    }

    return status;
}

/* Configure FSM_TRIG_SEL_2 register. */
static int32_t Pmic_fsmSetTrigSel2(const Pmic_Handle_t *handle, const Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MODERATE_ERR_TRIG_VALID))
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_2_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, MODERATE_ERR_TRIG_SHIFT, MODERATE_ERR_TRIG_MASK, triggerCfg->moderateErrTrig);
            status = Pmic_ioTxByte(handle, FSM_TRIG_SEL_2_REG, regData);
        }
    }

    return status;
}

/* Read FSM_TRIG_SEL_1 register. */
static int32_t Pmic_fsmGetTrigSel1(const Pmic_Handle_t *handle, Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SEVERE_ERR_TRIG_VALID) ||
        Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_OTHER_RAIL_TRIG_VALID) ||
        Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SOC_RAIL_TRIG_VALID) ||
        Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MCU_RAIL_TRIG_VALID))
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_1_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SEVERE_ERR_TRIG_VALID))
            {
                triggerCfg->severeErrTrig = Pmic_getBitField(regData, SEVERE_ERR_TRIG_SHIFT, SEVERE_ERR_TRIG_MASK);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_OTHER_RAIL_TRIG_VALID))
            {
                triggerCfg->otherRailTrig = Pmic_getBitField(regData, OTHER_RAIL_TRIG_SHIFT, OTHER_RAIL_TRIG_MASK);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_SOC_RAIL_TRIG_VALID))
            {
                triggerCfg->socRailTrig = Pmic_getBitField(regData, SOC_RAIL_TRIG_SHIFT, SOC_RAIL_TRIG_MASK);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MCU_RAIL_TRIG_VALID))
            {
                triggerCfg->mcuRailTrig = Pmic_getBitField(regData, MCU_RAIL_TRIG_SHIFT, MCU_RAIL_TRIG_MASK);
            }
        }
    }

    return status;
}

/* Read FSM_TRIG_SEL_2 register. */
static int32_t Pmic_fsmGetTrigSel2(const Pmic_Handle_t *handle, Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_CFG_FSM_MODERATE_ERR_TRIG_VALID))
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_2_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            triggerCfg->moderateErrTrig = Pmic_getBitField(regData, MODERATE_ERR_TRIG_SHIFT, MODERATE_ERR_TRIG_MASK);
        }
    }

    return status;
}

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_fsmSetTriggerCfg(const Pmic_Handle_t *handle, const Pmic_FsmTriggerCfg_t *triggerCfg)
{
    Pmic_FsmTriggerCfg_t triggerCfgLocal = (Pmic_FsmTriggerCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (triggerCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (triggerCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    FSM_copyFsmTriggerCfg(triggerCfg, &triggerCfgLocal);
    status = Pmic_fsmValidateTriggerCfg(&triggerCfgLocal);
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    // Status guaranteed SUCCESS here due to early return above
    status = Pmic_fsmSetTrigSel1(handle, &triggerCfgLocal);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_fsmSetTrigSel2(handle, &triggerCfgLocal);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetTriggerCfg(const Pmic_Handle_t *handle, Pmic_FsmTriggerCfg_t *triggerCfg)
{
    Pmic_FsmTriggerCfg_t triggerCfgLocal = (Pmic_FsmTriggerCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (triggerCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (triggerCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    FSM_copyFsmTriggerCfg(triggerCfg, &triggerCfgLocal);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    // Status guaranteed SUCCESS here - all validations passed
    status = Pmic_fsmGetTrigSel1(handle, &triggerCfgLocal);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_fsmGetTrigSel2(handle, &triggerCfgLocal);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmTriggerCfg(&triggerCfgLocal, triggerCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetGpioTriggerCfg(const Pmic_Handle_t *handle, const Pmic_FsmGpioTriggerCfg_t *gpioTriggerCfg)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfgLocal = (Pmic_FsmGpioTriggerCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);
    const Pmic_FsmGpioPinMap_t *pinMap = NULL;
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (gpioTriggerCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (gpioTriggerCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    FSM_copyFsmGpioTriggerCfg(gpioTriggerCfg, &gpioTriggerCfgLocal);

    if (Pmic_validParamCheck(gpioTriggerCfgLocal.validParams, PMIC_CFG_FSM_MASK_POL_VALID) &&
        (gpioTriggerCfgLocal.maskPol > PMIC_FSM_GPIO_MASK_POL_MAX))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    status = Pmic_fsmGetGpioPinMapping(gpioTriggerCfgLocal.pinNum, &pinMap);
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (pinMap == NULL) /* DA_JUSTIFY: PMICDRV-2356 */
    { /* DA_JUSTIFY: PMICDRV-2356 */
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM); /* DA_JUSTIFY: PMICDRV-2356 */
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, pinMap->regAddr, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(gpioTriggerCfgLocal.validParams, PMIC_CFG_FSM_MASK_VALID))
        {
            Pmic_setBitField_b(&regData, pinMap->maskShift, gpioTriggerCfgLocal.mask);
        }

        if (Pmic_validParamCheck(gpioTriggerCfgLocal.validParams, PMIC_CFG_FSM_MASK_POL_VALID))
        {
            Pmic_setBitField(&regData, pinMap->maskPolShift, (uint8_t)(1UL << pinMap->maskPolShift), gpioTriggerCfgLocal.maskPol);
        }

        status = Pmic_ioTxByte(handle, pinMap->regAddr, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetGpioTriggerCfg(const Pmic_Handle_t *handle, Pmic_FsmGpioTriggerCfg_t *gpioTriggerCfg)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfgLocal = (Pmic_FsmGpioTriggerCfg_t){0};
    int32_t status = Pmic_checkHandle(handle);
    const Pmic_FsmGpioPinMap_t *pinMap = NULL;
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (gpioTriggerCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (gpioTriggerCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    FSM_copyFsmGpioTriggerCfg(gpioTriggerCfg, &gpioTriggerCfgLocal);

    status = Pmic_fsmGetGpioPinMapping(gpioTriggerCfgLocal.pinNum, &pinMap);
    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (pinMap == NULL) /* DA_JUSTIFY: PMICDRV-2356 */
    { /* DA_JUSTIFY: PMICDRV-2356 */
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM); /* DA_JUSTIFY: PMICDRV-2356 */
    }

    status = Pmic_ioRxByte_CS(handle, pinMap->regAddr, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(gpioTriggerCfgLocal.validParams, PMIC_CFG_FSM_MASK_VALID))
        {
            gpioTriggerCfgLocal.mask = Pmic_getBitField_b(regData, pinMap->maskShift);
        }

        if (Pmic_validParamCheck(gpioTriggerCfgLocal.validParams, PMIC_CFG_FSM_MASK_POL_VALID))
        {
            gpioTriggerCfgLocal.maskPol = Pmic_getBitField(regData, pinMap->maskPolShift, (uint8_t)(1UL << pinMap->maskPolShift));
        }

        FSM_copyFsmGpioTriggerCfg(&gpioTriggerCfgLocal, gpioTriggerCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetRecovCnt(const Pmic_Handle_t *handle, uint8_t *recovCnt)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_1 and extract RECOV_CNT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, RECOV_CNT_REG_1_REG, &regData);
        *recovCnt = Pmic_getBitField(regData, RECOV_CNT_SHIFT, RECOV_CNT_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmClrRecovCnt(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);

    // Set RECOV_CNT_CLR bit field to 1 and write to RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioUpdateByte_bCS(handle, RECOV_CNT_REG_2_REG, RECOV_CNT_CLR_SHIFT, (bool)true);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetRecovCntThr(const Pmic_Handle_t *handle, uint8_t recovCntThr)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCntThr > PMIC_FSM_RECOV_CNT_THR_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read RECOV_CNT_REG_2
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, RECOV_CNT_REG_2_REG, &regData);
    }

    // Modify RECOV_CNT_THR and Write RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, RECOV_CNT_THR_SHIFT, RECOV_CNT_THR_MASK, recovCntThr);
        status = Pmic_ioTxByte(handle, RECOV_CNT_REG_2_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetRecovCntThr(const Pmic_Handle_t *handle, uint8_t *recovCntThr)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCntThr == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_2 and extract RECOV_CNT_THR
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, RECOV_CNT_REG_2_REG, &regData);
        *recovCntThr = Pmic_getBitField(regData, RECOV_CNT_THR_SHIFT, RECOV_CNT_THR_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSendSoftRebootReq(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Set SOFT_REBOOT bit field to 1 and write to SOFT_REBOOT_REG
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, SOFT_REBOOT_SHIFT, SOFT_REBOOT_MASK, 1U);
        status = Pmic_ioTxByte_CS(handle, SOFT_REBOOT_REG_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetStartupDest(const Pmic_Handle_t *handle, uint8_t destination)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (destination > PMIC_FSM_START_UP_DEST_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read STARTUP_CTRL
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, STARTUP_CTRL_REG, &regData);
    }

    // Modify STARTUP_DEST and write STARTUP_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, STARTUP_DEST_SHIFT, STARTUP_DEST_MASK, destination);
        status = Pmic_ioTxByte(handle, STARTUP_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetStartupDest(const Pmic_Handle_t *handle, uint8_t *destination)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (destination == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read STARTUP_CTRL and extract STARTUP_DEST
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, STARTUP_CTRL_REG, &regData);
        *destination = Pmic_getBitField(regData, STARTUP_DEST_SHIFT, STARTUP_DEST_MASK);
    }

    return Pmic_logStatus(handle, status);
}
