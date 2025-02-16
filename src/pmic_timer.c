/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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

#include "pmic_timer.h"
#include "pmic_io.h"
#include "regmap/timer.h"

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_timerSetCfg(Pmic_CoreHandle_t *handle, const Pmic_timerCfg_t *timerCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Parameter check
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
        // Read TMR_CFG_REG
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, TMR_CFG_REG, &regData);

        // Modify timer prescale
        if (Pmic_validParamStatusCheck(timerCfg->validParams, PMIC_CFG_TMR_PRESCALE_VALID, status))
        {
            if (timerCfg->prescale > PMIC_TMR_PRESCALE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, TMR_PS_SHIFT, TMR_PS_MASK, timerCfg->prescale);
            }
        }

        // Modify timer mode
        if (Pmic_validParamStatusCheck(timerCfg->validParams, PMIC_CFG_TMR_MODE_VALID, status))
        {
            if (timerCfg->mode > PMIC_TMR_MODE_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, TMR_CFG_SHIFT, TMR_CFG_MASK, timerCfg->mode);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, TMR_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_timerGetCfg(Pmic_CoreHandle_t *handle, Pmic_timerCfg_t *timerCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (timerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (timerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read TMR_CFG_REG
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, TMR_CFG_REG, &regData);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Get timer prescale
        if (Pmic_validParamCheck(timerCfg->validParams, PMIC_CFG_TMR_PRESCALE_VALID))
        {
            timerCfg->prescale = Pmic_getBitField(regData, TMR_PS_SHIFT, TMR_PS_MASK);
        }

        // Get timer mode
        if (Pmic_validParamCheck(timerCfg->validParams, PMIC_CFG_TMR_MODE_VALID))
        {
            timerCfg->mode = Pmic_getBitField(regData, TMR_CFG_SHIFT, TMR_CFG_MASK);
        }
    }

    return status;
}

int32_t Pmic_timerClr(Pmic_CoreHandle_t *handle)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read TMR_CFG_REG
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, TMR_CFG_REG, &regData);
    }

    // Set TMR_CLR bit field to 1 to clear the timer counter then write new
    // register value back to PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, TMR_CLR_SHIFT, TMR_CLR_MASK, 1U);
        status = Pmic_ioTxByte(handle, TMR_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_timerSetCnt(Pmic_CoreHandle_t *handle, uint32_t tmrCnt)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (tmrCnt > PMIC_TMR_CNT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Overwrite current timer count with new value (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxWordSeq(handle, TMR_CNT0_REG, tmrCnt, TMR_CNT_REG_CNT);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_timerGetCnt(Pmic_CoreHandle_t *handle, uint32_t *tmrCnt)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint32_t value = 0U;

    // Get current timer count (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxWordSeq(handle, TMR_CNT0_REG, &value, TMR_CNT_REG_CNT);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *tmrCnt = value;
    }

    return status;
}

int32_t Pmic_timerSetWakeupVal(Pmic_CoreHandle_t *handle, uint32_t wakeupVal)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // Parameter check
    if ((status == PMIC_ST_SUCCESS) && (wakeupVal > PMIC_TMR_WAKEUP_VAL_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Overwrite current wakeup value with new value (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxWordSeq(handle, TMR_LP_WAKE0_REG, wakeupVal, TMR_LP_WAKE_REG_CNT);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_timerGetWakeupVal(Pmic_CoreHandle_t *handle, uint32_t *wakeupVal)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint32_t value = 0U;

    // Get current wakeup value (bytes 0-2)
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxWordSeq(handle, TMR_LP_WAKE0_REG, &value, TMR_LP_WAKE_REG_CNT);
        Pmic_criticalSectionStop(handle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        *wakeupVal = value;
    }

    return status;
}
