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
 * @file pmic.c
 *
 * @brief This file contains definitions of the PMIC LLD initialization and
 * deinitialization APIs.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_common.h"

static inline int32_t setPmicCommsHandleMembers(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(pmicCfg->validParams, PMIC_COMM_HANDLE_VALID))
    {
        if (pmicCfg->commHandle != NULL)
        {
            pmicHandle->commHandle = pmicCfg->commHandle;
        }
        else
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(pmicCfg->validParams, PMIC_IO_TRANSFER_VALID))
    {
        if (pmicCfg->ioTransfer != NULL)
        {
            pmicHandle->ioTransfer = pmicCfg->ioTransfer;
        }
        else
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(pmicCfg->validParams, PMIC_IRQ_RESPONSE_VALID))
    {
        if (pmicCfg->irqResponse != NULL)
        {
            pmicHandle->irqResponse = pmicCfg->irqResponse;
        }
        else
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    return status;
}

static inline int32_t setPmicSysHandleMembers(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(pmicCfg->validParams, PMIC_CRIT_SEC_START_VALID))
    {
        if (pmicCfg->critSecStart != NULL)
        {
            pmicHandle->critSecStart = pmicCfg->critSecStart;
        }
        else
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    if (Pmic_validParamCheck(pmicCfg->validParams, PMIC_CRIT_SEC_STOP_VALID))
    {
        if (pmicCfg->critSecStop != NULL)
        {
            pmicHandle->critSecStop = pmicCfg->critSecStop;
        }
        else
        {
            status = PMIC_ST_ERR_NULL_PARAM;
        }
    }

    return status;
}

static int32_t getPmicInfo(Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (status == PMIC_ST_SUCCESS)
    {
        // Read DEV_ID register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_DEV_ID, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract DEV_ID bit field (the entire register is the bit field)
        pmicHandle->devId = regData;

        // Read DEV_REV register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_DEV_REV, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract DEV_REV bit field (the entire register is the bit field)
        pmicHandle->devRev = regData;
    }

    return status;
}

int32_t Pmic_init(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Check whether parameters are valid
    if ((pmicHandle == NULL) || (pmicCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pmicCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Initialize PMIC handle with values from pmicCfg
    if (status == PMIC_ST_SUCCESS)
    {
        setPmicCommsHandleMembers(pmicCfg, pmicHandle);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        setPmicSysHandleMembers(pmicCfg, pmicHandle);
    }

    // Initialize PMIC command lookup table and obtain information about the PMIC
    if (status == PMIC_ST_SUCCESS)
    {
        status = getPmicInfo(pmicHandle);
    }

    // Set the driver initialization status
    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->drvInitStat = PMIC_DRV_INIT;
    }

    return status;
}

int32_t Pmic_deinit(Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        pmicHandle->drvInitStat = PMIC_DRV_UNINIT;
        pmicHandle->devRev = 0U;
        pmicHandle->devId = 0U;
        pmicHandle->commHandle = NULL;
        pmicHandle->ioTransfer = NULL;
        pmicHandle->critSecStart = NULL;
        pmicHandle->critSecStop = NULL;
        pmicHandle->irqResponse = NULL;
    }

    return status;
}
