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
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_timer.h"
#include "pmic_io.h"
#include "regmap/timer.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

#define STOP_TIMER (0U)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
static inline void TIMER_copyTimerCfg(const Pmic_TimerCfg_t *src, Pmic_TimerCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_TimerCfg_t));
}

/**
 * @brief Check if Timer is in valid state for prescale configuration
 *
 * Validates that Timer is stopped before allowing prescale configuration changes.
 *
 * @return PMIC_ST_SUCCESS if state is valid, PMIC_ST_ERR_NOT_SUPPORTED otherwise
 */
static int32_t TIMER_checkPrescaleCfgState(const Pmic_Handle_t *handle) {
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_TimerCfg_t timerCfg = {0};

    timerCfg.validParams = PMIC_CFG_TMR_MODE_VALID;
    status = Pmic_timerGetCfg(handle, &timerCfg);

    if ((status == PMIC_ST_SUCCESS) && (timerCfg.mode != PMIC_TMR_MODE_STOPPED)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    return status;
}

static int32_t TIMER_validateCfgParams(const Pmic_TimerCfg_t *cfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(cfg->validParams, PMIC_CFG_TMR_PRESCALE_VALID))
    {
        if (cfg->prescale > PMIC_TMR_PRESCALE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if ((status == PMIC_ST_SUCCESS) &&
        Pmic_validParamCheck(cfg->validParams, PMIC_CFG_TMR_MODE_VALID))
    {
        if (cfg->mode > PMIC_TMR_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

static int32_t TIMER_applyTimerCfg(const Pmic_Handle_t *handle, const Pmic_TimerCfg_t *localTimerCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, TMR_CFG_REG, &regData);

    if (Pmic_validParamStatusCheck(localTimerCfg->validParams, PMIC_CFG_TMR_PRESCALE_VALID, status))
    {
        Pmic_setBitField(&regData, TMR_PS_SHIFT, TMR_PS_MASK, localTimerCfg->prescale);
    }

    if (Pmic_validParamStatusCheck(localTimerCfg->validParams, PMIC_CFG_TMR_MODE_VALID, status))
    {
        Pmic_setBitField(&regData, TMR_CFG_SHIFT, TMR_CFG_MASK, localTimerCfg->mode);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, TMR_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_timerSetCfg(const Pmic_Handle_t *handle, const Pmic_TimerCfg_t *timerCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_TimerCfg_t localTimerCfg = (Pmic_TimerCfg_t){0};

    if ((status == PMIC_ST_SUCCESS) && (timerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (timerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        TIMER_copyTimerCfg(timerCfg, &localTimerCfg);
        status = TIMER_validateCfgParams(&localTimerCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        Pmic_validParamCheck(localTimerCfg.validParams, PMIC_CFG_TMR_PRESCALE_VALID))
    {
        status = TIMER_checkPrescaleCfgState(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = TIMER_applyTimerCfg(handle, &localTimerCfg);
    }

    return Pmic_logStatus(handle, status);
}

static void TIMER_extractTimerCfgFields(uint8_t regData, Pmic_TimerCfg_t *localTimerCfg)
{
    if (Pmic_validParamCheck(localTimerCfg->validParams, PMIC_CFG_TMR_PRESCALE_VALID))
    {
        localTimerCfg->prescale = Pmic_getBitField(regData, TMR_PS_SHIFT, TMR_PS_MASK);
    }

    if (Pmic_validParamCheck(localTimerCfg->validParams, PMIC_CFG_TMR_MODE_VALID))
    {
        localTimerCfg->mode = Pmic_getBitField(regData, TMR_CFG_SHIFT, TMR_CFG_MASK);
    }
}

int32_t Pmic_timerGetCfg(const Pmic_Handle_t *handle, Pmic_TimerCfg_t *timerCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    Pmic_TimerCfg_t localTimerCfg = (Pmic_TimerCfg_t){0};

    if ((status == PMIC_ST_SUCCESS) && (timerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (timerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        TIMER_copyTimerCfg(timerCfg, &localTimerCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, TMR_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        TIMER_extractTimerCfgFields(regData, &localTimerCfg);
        TIMER_copyTimerCfg(&localTimerCfg, timerCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_timerStop(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS)
    {
        // Read TMR_CFG_REG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, TMR_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Set TMR_CFG to zero to stop timer
            Pmic_setBitField(&regData, TMR_CFG_SHIFT, TMR_CFG_MASK, STOP_TIMER);

            // Write TMR_CFG_REG
            status = Pmic_ioTxByte(handle, TMR_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_timerClr(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS)
    {
        // Read TMR_CFG_REG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, TMR_CFG_REG, &regData);

        // Set TMR_CLR bit field to 1 to clear the timer counter then write new
        // register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, TMR_CLR_SHIFT, TMR_CLR_MASK, 1U);
            status = Pmic_ioTxByte(handle, TMR_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_timerSetCnt(const Pmic_Handle_t *handle, uint32_t tmrCnt)
{
    int32_t status = Pmic_checkHandle(handle);

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (tmrCnt > PMIC_TMR_CNT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Overwrite current timer count with new value (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxWordSeq(handle, TMR_CNT0_REG, tmrCnt, TMR_CNT_REG_CNT);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_timerGetCnt(const Pmic_Handle_t *handle, uint32_t *tmrCnt)
{
    int32_t status = Pmic_checkHandle(handle);
    uint32_t value = 0U;

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (tmrCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Get current timer count (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxWordSeq(handle, TMR_CNT0_REG, &value, TMR_CNT_REG_CNT);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *tmrCnt = value;
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_timerSetWakeupValue(const Pmic_Handle_t *handle, uint32_t wakeupVal)
{
    int32_t status = Pmic_checkHandle(handle);

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (wakeupVal > PMIC_TMR_WAKEUP_VAL_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Overwrite current wakeup value with new value (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioTxWordSeq(handle, TMR_LP_WAKE0_REG, wakeupVal, TMR_LP_WAKE_REG_CNT);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_timerGetWakeupValue(const Pmic_Handle_t *handle, uint32_t *wakeupVal)
{
    int32_t status = Pmic_checkHandle(handle);
    uint32_t value = 0U;

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (wakeupVal == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Get current wakeup value (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxWordSeq(handle, TMR_LP_WAKE0_REG, &value, TMR_LP_WAKE_REG_CNT);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *wakeupVal = value;
    }

    return Pmic_logStatus(handle, status);
}
