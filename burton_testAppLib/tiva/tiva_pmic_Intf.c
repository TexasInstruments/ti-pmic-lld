#include "tiva_priv.h"
#include "tiva_pmic_intf.h"

int32_t
pmicI2CWrite(Pmic_CoreHandle_t *pmicCorehandle, uint8_t instType, uint16_t regAddr, uint8_t *pTxBuf, uint8_t bufLen)
{
    // Variable declaration/initialization
    int32_t      status = PMIC_ST_SUCCESS;
    i2cHandle_t *I2C1Handle = NULL;
    i2cHandle_t *I2C2Handle = NULL;

    // Parameter check
    if (pmicCorehandle == NULL)
    {
        return PMIC_ST_ERR_INV_HANDLE;
    }
    if ((bufLen == 0) || (pmicCorehandle->crcEnable == true) || (instType == PMIC_NVM_INST))
    {
        return PMIC_ST_ERR_INV_PARAM;
    }
    if (pTxBuf == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }
    if (regAddr >= 0x401)
    {
        instType = PMIC_QA_INST;
        regAddr -= 0x400;
    }

    // Obtain communication handles as they will be passed into helper functions
    I2C1Handle = (i2cHandle_t *)pmicCorehandle->pCommHandle;
    I2C2Handle = (i2cHandle_t *)pmicCorehandle->pQACommHandle;

    // Main instance
    if (instType == PMIC_MAIN_INST)
    {
        I2C1Handle->slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS;
        status = ((bufLen == 1) ? I2CSingleWrite(I2C1Handle, (uint8_t)regAddr, pTxBuf) :
                                  I2CBurstWrite(I2C1Handle, (uint8_t)regAddr, bufLen, pTxBuf));
    }
    // Q&A instance
    else if (instType == PMIC_QA_INST)
    {
        if (pmicCorehandle->commMode == PMIC_INTF_DUAL_I2C)
        {
            status = ((bufLen == 1) ? I2CSingleWrite(I2C2Handle, (uint8_t)regAddr, pTxBuf) :
                                      I2CBurstWrite(I2C2Handle, (uint8_t)regAddr, bufLen, pTxBuf));
        }
        else
        {
            I2C1Handle->slaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS;
            status = ((bufLen == 1) ? I2CSingleWrite(I2C1Handle, (uint8_t)regAddr, pTxBuf) :
                                      I2CBurstWrite(I2C1Handle, (uint8_t)regAddr, bufLen, pTxBuf));
        }
    }
    // NVM instance
    else if (instType == PMIC_NVM_INST)
    {
        I2C1Handle->slaveAddr = BURTON_I2C_NVM_PAGE_ADDRESS;
        status = ((bufLen == 1) ? I2CSingleWrite(I2C1Handle, (uint8_t)regAddr, pTxBuf) :
                                  I2CBurstWrite(I2C1Handle, (uint8_t)regAddr, bufLen, pTxBuf));
    }
    // Unrecognized instance
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

int32_t
pmicI2CRead(Pmic_CoreHandle_t *pmicCorehandle, uint8_t instType, uint16_t regAddr, uint8_t *pRxBuf, uint8_t bufLen)
{
    // Variable declaration/initialization
    int32_t      status = PMIC_ST_SUCCESS;
    i2cHandle_t *I2C1Handle = NULL;
    i2cHandle_t *I2C2Handle = NULL;

    // Parameter check
    if (pmicCorehandle == NULL)
    {
        return PMIC_ST_ERR_INV_HANDLE;
    }
    if ((bufLen == 0) || (pmicCorehandle->crcEnable == true))
    {
        return PMIC_ST_ERR_INV_PARAM;
    }
    if (pRxBuf == NULL)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }
    if (regAddr >= 0x401)
    {
        instType = PMIC_QA_INST;
        regAddr -= 0x400;
    }

    // Obtain communication handles as they will be passed into helper functions
    I2C1Handle = (i2cHandle_t *)pmicCorehandle->pCommHandle;
    I2C2Handle = (i2cHandle_t *)pmicCorehandle->pQACommHandle;

    // Main instance
    if (instType == PMIC_MAIN_INST)
    {
        I2C1Handle->slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS;
        status = ((bufLen == 1) ? I2CSingleRead(I2C1Handle, (uint8_t)regAddr, pRxBuf) :
                                  I2CBurstRead(I2C1Handle, (uint8_t)regAddr, bufLen, pRxBuf));
    }
    // Q&A instance
    else if (instType == PMIC_QA_INST)
    {
        if (pmicCorehandle->commMode == PMIC_INTF_DUAL_I2C)
        {
            status = ((bufLen == 1) ? I2CSingleRead(I2C2Handle, (uint8_t)regAddr, pRxBuf) :
                                      I2CBurstRead(I2C2Handle, (uint8_t)regAddr, bufLen, pRxBuf));
        }
        else
        {
            I2C1Handle->slaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS;
            status = ((bufLen == 1) ? I2CSingleRead(I2C1Handle, (uint8_t)regAddr, pRxBuf) :
                                      I2CBurstRead(I2C1Handle, (uint8_t)regAddr, bufLen, pRxBuf));
        }
    }
    // NVM instance
    else if (instType == PMIC_NVM_INST)
    {
        I2C1Handle->slaveAddr = BURTON_I2C_NVM_PAGE_ADDRESS;
        status = ((bufLen == 1) ? I2CSingleRead(I2C1Handle, (uint8_t)regAddr, pRxBuf) :
                                  I2CBurstRead(I2C1Handle, (uint8_t)regAddr, bufLen, pRxBuf));
    }
    // Unrecognized instance
    else
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

void pmicCritSecStart(void)
{
}

void pmicCritSecStop(void)
{
}

void initializePmicCoreHandle(Pmic_CoreHandle_t *pmicCoreHandle)
{
    pmicCoreHandle->pPmic_SubSysInfo = NULL;
    pmicCoreHandle->drvInitStatus = 0x00;
    pmicCoreHandle->pmicDeviceType = 0xFF;
    pmicCoreHandle->pmicDevRev = 0xFF;
    pmicCoreHandle->pmicDevSiliconRev = 0xFF;
    pmicCoreHandle->commMode = 0xFF;
    pmicCoreHandle->slaveAddr = 0xFF;
    pmicCoreHandle->qaSlaveAddr = 0xFF;
    pmicCoreHandle->nvmSlaveAddr = 0xFF;
    pmicCoreHandle->i2c1Speed = 0xFF;
    pmicCoreHandle->i2c2Speed = 0xFF;
    pmicCoreHandle->crcEnable = false;
    pmicCoreHandle->pCommHandle = NULL;
    pmicCoreHandle->pQACommHandle = NULL;
    pmicCoreHandle->pFnPmicCommIoRead = NULL;
    pmicCoreHandle->pFnPmicCommIoWrite = NULL;
    pmicCoreHandle->pFnPmicCritSecStart = &pmicCritSecStart;
    pmicCoreHandle->pFnPmicCritSecStop = &pmicCritSecStop;
}
