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
