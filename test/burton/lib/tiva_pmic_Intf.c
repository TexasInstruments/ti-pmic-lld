/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
#include "tiva_priv.h"
#include "tiva_pmic_intf.h"

int32_t
pmicI2CWrite(Pmic_CoreHandle_t *pmicHandle, uint8_t instType, uint16_t regAddr, uint8_t *pTxBuf, uint8_t bufLen)
{
    // Variable declaration/initialization
    int32_t status = PMIC_ST_SUCCESS;
    i2cHandle_t *I2C1Handle = NULL;
    i2cHandle_t *I2C2Handle = NULL;

    // Parameter check
    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }
    if ((status == PMIC_ST_SUCCESS) && ((bufLen == 0) || (pmicHandle->crcEnable == true)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pTxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (regAddr >= 0x401)
        {
            instType = PMIC_QA_INST;
            regAddr -= 0x400;
        }

        // Obtain communication handles as they will be passed into helper functions
        I2C1Handle = (i2cHandle_t *)pmicHandle->pCommHandle;
        I2C2Handle = (i2cHandle_t *)pmicHandle->pQACommHandle;
        
        // Main instance
        if (instType == PMIC_MAIN_INST)
        {
            I2C1Handle->slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS;
            status = I2CWrite(I2C1Handle, (uint8_t)regAddr, bufLen, pTxBuf);
        }
        // Q&A instance
        else if (instType == PMIC_QA_INST)
        {
            // Dual I2C
            if (pmicHandle->commMode == PMIC_INTF_DUAL_I2C)
            {
                status = I2CWrite(I2C2Handle, (uint8_t)regAddr, bufLen, pTxBuf);
            }
            // Single I2C
            else
            {
                I2C1Handle->slaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS;
                status = I2CWrite(I2C1Handle, (uint8_t)regAddr, bufLen, pTxBuf);
            }
        }
        // NVM instance
        else if (instType == PMIC_NVM_INST)
        {
            I2C1Handle->slaveAddr = BURTON_I2C_NVM_PAGE_ADDRESS;
            status = I2CWrite(I2C1Handle, (uint8_t)regAddr, bufLen, pTxBuf);
        }
        // Unrecognized instance
        else
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

int32_t
pmicI2CRead(Pmic_CoreHandle_t *pmicHandle, uint8_t instType, uint16_t regAddr, uint8_t *pRxBuf, uint8_t bufLen)
{
    // Variable declaration/initialization
    int32_t status = PMIC_ST_SUCCESS;
    i2cHandle_t *I2C1Handle = NULL;
    i2cHandle_t *I2C2Handle = NULL;

    // Parameter check
    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }
    if ((status == PMIC_ST_SUCCESS) && ((bufLen == 0) || (pmicHandle->crcEnable == true)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    if ((status == PMIC_ST_SUCCESS) && (pRxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        if (regAddr >= 0x401)
        {
            instType = PMIC_QA_INST;
            regAddr -= 0x400;
        }

        // Obtain communication handles as they will be passed into helper functions
        I2C1Handle = (i2cHandle_t *)pmicHandle->pCommHandle;
        I2C2Handle = (i2cHandle_t *)pmicHandle->pQACommHandle;

        // Main instance
        if (instType == PMIC_MAIN_INST)
        {
            I2C1Handle->slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS;
            status = I2CRead(I2C1Handle, (uint8_t)regAddr, bufLen, pRxBuf);
        }
        // Q&A instance
        else if (instType == PMIC_QA_INST)
        {
            if (pmicHandle->commMode == PMIC_INTF_DUAL_I2C)
            {
                status = I2CRead(I2C2Handle, (uint8_t)regAddr, bufLen, pRxBuf);
            }
            else
            {
                I2C1Handle->slaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS;
                status = I2CRead(I2C1Handle, (uint8_t)regAddr, bufLen, pRxBuf);
            }
        }
        // NVM instance
        else if (instType == PMIC_NVM_INST)
        {
            I2C1Handle->slaveAddr = BURTON_I2C_NVM_PAGE_ADDRESS;
            status = I2CRead(I2C1Handle, (uint8_t)regAddr, bufLen, pRxBuf);
        }
        // Unrecognized instance
        else
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

void pmicCritSecStart(void)
{
}

void pmicCritSecStop(void)
{
}

void initializePmicCoreHandle(Pmic_CoreHandle_t *pmicHandle)
{
    pmicHandle->pPmic_SubSysInfo = NULL;
    pmicHandle->drvInitStatus = 0x00;
    pmicHandle->pmicDeviceType = 0xFF;
    pmicHandle->pmicDevRev = 0xFF;
    pmicHandle->pmicDevSiliconRev = 0xFF;
    pmicHandle->commMode = 0xFF;
    pmicHandle->slaveAddr = 0xFF;
    pmicHandle->qaSlaveAddr = 0xFF;
    pmicHandle->nvmSlaveAddr = 0xFF;
    pmicHandle->i2c1Speed = 0xFF;
    pmicHandle->i2c2Speed = 0xFF;
    pmicHandle->crcEnable = false;
    pmicHandle->pCommHandle = NULL;
    pmicHandle->pQACommHandle = NULL;
    pmicHandle->pFnPmicCommIoRead = NULL;
    pmicHandle->pFnPmicCommIoWrite = NULL;
    pmicHandle->pFnPmicCritSecStart = &pmicCritSecStart;
    pmicHandle->pFnPmicCritSecStop = &pmicCritSecStop;
}
