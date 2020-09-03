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
 *  \file   pmic_ut_common.h
 *
 *  \brief  Unit test related common APIs
 *
 */

#include <pmic_ut_common.h>

/* Pointer holds the pPmicCoreHandle for I2C */
Pmic_CoreHandle_t *pPmicCoreHandleI2C = NULL;

uint8_t startup_type = 0U;

/*!
 * \brief   OS specific Critical section locking Variable
 *          Should be OS specific locking varaible to
 *          use OS locking system for PMIC
 */
static SemaphoreP_Handle pmic_Sem = NULL;

/*!
 * \brief   GPIO Driver board specific pin configuration structure
 */
GPIO_PinConfig gpioPinConfigs[] =
{
    /* Input pin with interrupt enabled */
    GPIO_DEVICE_CONFIG(J7_WAKEUP_GPIO0_PORT_NUM, J7_WAKEUP_GPIO0_9_PIN_NUM) |
    GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INPUT
};

/*!
 * \brief   GPIO Driver call back functions
 */
GPIO_CallbackFxn gpioCallbackFunctions[] =
{
    NULL
};

/*!
 * \brief   GPIO Driver configuration structure
 */
GPIO_v0_Config GPIO_v0_config =
{
    gpioPinConfigs,
    gpioCallbackFunctions,
    sizeof(gpioPinConfigs) / sizeof(GPIO_PinConfig),
    sizeof(gpioCallbackFunctions) / sizeof(GPIO_CallbackFxn),
#ifdef __TI_ARM_V7R4__
    0x8U
#else
#if defined(__C7100__)
    0x01U
#else
    0x20U
#endif
#endif
};

volatile uint32_t pmic_intr_triggered = 0U;
volatile uint8_t irqNumToClr          = 0U;

/* Pointer holds the pPmicCoreHandle for AppPmicCallbackFxn */
Pmic_CoreHandle_t *pPmicCoreHandleISR = NULL;

/**
 * \brief    This API Set Config for TI HW I2C instances
 *
 * \param    instance [IN] I2C instance number
 * \param    baseAddr [IN] Register base address of the I2C instance
 */
static void test_pmic_setConfigI2C(uint8_t instance, uint32_t baseAddr)
{
    I2C_HwAttrs i2cCfg;

    I2C_socGetInitCfg(instance, &i2cCfg);
    i2cCfg.baseAddr   = baseAddr;
    i2cCfg.enableIntr = 0U;
    I2C_socSetInitCfg(instance, &i2cCfg);
}

/**
 *
 * \brief    Initalize SPI stub function to access PMIC registers using
 *           I2C Communication Handle
 */
static int32_t test_pmic_spi_stubInit(Pmic_CoreCfg_t  *pPmicConfigData)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigDataI2c = {0U};

    /* Fill parameters to pmicConfigDataI2C */
    pmicConfigDataI2c.pmicDeviceType      = pPmicConfigData->pmicDeviceType;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigDataI2c.commMode            = PMIC_INTF_DUAL_I2C;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigDataI2c.slaveAddr           = LEO_PMICA_SLAVE_ADDR;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigDataI2c.qaSlaveAddr         = LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigDataI2c.crcEnable           = pPmicConfigData->crcEnable;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_CRC_ENABLE_VALID_SHIFT;

    pmicConfigDataI2c.pFnPmicCommIoRead   = pPmicConfigData->pFnPmicCommIoRead;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigDataI2c.pFnPmicCommIoWrite  = pPmicConfigData->pFnPmicCommIoWrite;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigDataI2c.pFnPmicCritSecStart =
                                          pPmicConfigData->pFnPmicCritSecStart;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigDataI2c.pFnPmicCritSecStop  = pPmicConfigData->pFnPmicCritSecStop;
    pmicConfigDataI2c.validParams        |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    /*
     * Recalling appInit function to initialize I2C PMIC Core handle
     * for Stub Operations
     */
    pmicStatus = test_pmic_appInit(&pPmicCoreHandleI2C, &pmicConfigDataI2c);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /*
         * Update Valid Communication Handle to make SPI stub
         * PMIC core handle Init Success
         */
        pPmicConfigData->pCommHandle  = pPmicCoreHandleI2C->pCommHandle;
        pPmicConfigData->validParams |= PMIC_CFG_COMM_HANDLE_VALID_SHIFT;
    }
    return pmicStatus;
}

/**
 * \brief    Deinitalize SPI stub function
 */
static int32_t test_pmic_spi_stubDeinit(void **pCommHandle)
{
    test_pmic_appDeInit(pPmicCoreHandleI2C);

    if(NULL == (I2C_Handle)*pCommHandle)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    *pCommHandle = NULL;
    return PMIC_ST_SUCCESS;
}

/**
 * \brief  SPI stub function read operation to read PMIC registers
 *         using I2C interface
 */
int32_t test_pmic_spi_stubRead(Pmic_CoreHandle_t  *pPmicCorehandle,
                           uint8_t            *pBuf,
                           uint8_t             bufLen)
{
    int32_t pmicStatus = 0;
    uint8_t instType = 0U;
    uint16_t regAddr = 0U;
    bool wdgopn = 0;
    uint8_t rxBuf[4U] = {0U};

    /* Check for WatchDog Operation */
    if(0U != (pBuf[1U] & 0x04))
    {
        wdgopn = true;
    }

    /* Update register Address from spi buffer */
    regAddr = (uint16_t)pBuf[0U];
    bufLen = 1U;

    /* Find Instance type from wdg operation */
    if(true == wdgopn)
    {
        instType = PMIC_QA_INST;
    }
    else
    {
        instType = PMIC_MAIN_INST;
    }

    /* Increase buffer lenth 1 more to get CRC, if CRC is Enabled */
    if(true == pPmicCorehandle->crcEnable)
    {
        bufLen++;
    }

    /* Call PMIC read with I2C Instance */
    pmicStatus = test_pmic_regRead(pPmicCoreHandleI2C,
                                   instType,
                                   regAddr,
                                   rxBuf,
                                   bufLen);

    /* Updating the Recieved Reg Value to SPI Buffer */
    pBuf[2U] = rxBuf[0U];

    /* Updating the Recieved CRC to SPI Buffer */
    if(true == pPmicCorehandle->crcEnable)
    {
        pBuf[3U] = rxBuf[1U];
    }

    return pmicStatus;
}

/**
 * \brief  SPI stub function write operation to write PMIC registers
 *         using I2C interface
 */
int32_t test_pmic_spi_write(Pmic_CoreHandle_t  *pPmicCorehandle,
                            uint8_t            *pBuf,
                            uint8_t             bufLen)
{
    int32_t  pmicStatus = 0;
    uint8_t  instType = 0U;
    bool     wdgopn = 0;
    uint16_t regAddr = 0U;
    uint8_t  txBuf[4U] = {0U};

    /* Update register Address from spi buffer */
    regAddr = (uint16_t)pBuf[0U];

    /* Check for WatchDog Operation */
    if(0U != (pBuf[1U] & 0x04U))
    {
        wdgopn = true;
    }

    /* Find Instance type from wdg operation */
    if(true == wdgopn)
    {
        instType = PMIC_QA_INST;
    }
    else
    {
        instType = PMIC_MAIN_INST;
    }

    /* Updating the SPI Buffer Reg Value to I2C Buffer */
    txBuf[0U] = pBuf[2U];
    bufLen = 1U;

    /* Updating the Recieved CRC to SPI Buffer */
    if(true == pPmicCorehandle->crcEnable)
    {
        txBuf[1U] = pBuf[3U];
        bufLen++;
    }

    /* Call PMIC write with I2C Instance */
    pmicStatus = test_pmic_regWrite(pPmicCoreHandleI2C,
                                    instType,
                                    regAddr,
                                    txBuf,
                                    bufLen);

    return pmicStatus;
}

/*!
 * \brief   Function to setup the I2C lld interface for PMIC
 *
 * \param   pPmicCoreHandle  [IN]     PMIC Interface Handle.
 * \param   instType         [IN]     Instance Type
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
static int32_t test_pmic_i2c_lld_intf_setup(Pmic_CoreCfg_t  *pPmicConfigData,
                                            uint8_t          instType)
{
    I2C_Params i2cParams;
    I2C_Handle i2cHandle  = NULL;
    uint8_t i2c_instance  = 0U;

    pmic_log("%s(): %d: %s I2C Setup...\n", __func__, __LINE__,
             (instType == PMIC_MAIN_INST)? "PMIC_MAIN_INST": "PMIC_QA_INST");

    /* Main I2c BUS */
    if(instType == PMIC_MAIN_INST)
    {
        i2c_instance = 0U;
        test_pmic_setConfigI2C(i2c_instance, CSL_WKUP_I2C0_CFG_BASE);
    }
    /* For WDG QA I2C BUS */
    else if(PMIC_QA_INST == instType)
    {
        i2c_instance = 1U;
        test_pmic_setConfigI2C(i2c_instance, CSL_MCU_I2C0_CFG_BASE);
    }

    /* Initialize i2c core instances */
    I2C_init();
    /* Configured i2cParams.bitRate with standard I2C_100kHz */
    I2C_Params_init(&i2cParams);

    i2cHandle = I2C_open(i2c_instance, &i2cParams);
    if(NULL == i2cHandle)
    {
        pmic_log("I2C_open is failed!!!\n");
        return PMIC_ST_ERR_COMM_INTF_INIT_FAIL;
    }

    /* Main I2c BUS */
    if(instType == PMIC_MAIN_INST)
    {
        pPmicConfigData->pCommHandle = i2cHandle;
    }
    /* For WDOG QA I2C BUS */
    else if(PMIC_QA_INST == instType)
    {
        pPmicConfigData->pQACommHandle = i2cHandle;
    }

    pmic_log("%s(): %d: done...\n", __func__, __LINE__);
    return PMIC_ST_SUCCESS;
}

/*!
 * \brief   Function to setup the QA I2c interface for LEO PMIC
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
static int32_t test_pmic_leo_dual_i2c_pin_setup(Pmic_CoreHandle_t *pPmicHandle)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg = {0U};


    gpioCfg.validParams      = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                               PMIC_GPIO_CFG_OD_VALID_SHIFT;
    gpioCfg.pinFunc          = PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI;
    gpioCfg.outputSignalType = PMIC_GPIO_OPEN_DRAIN_OUTPUT;

    pmicStatus = Pmic_gpioSetConfiguration(pPmicHandle,
                                           PMIC_TPS6594X_GPIO1_PIN,
                                           gpioCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT;
        gpioCfg.pinFunc     = PMIC_TPS6594X_GPIO_PINFUNC_GPIO2_SDA_I2C2_SDO_SPI;
        gpioCfg.outputSignalType = PMIC_GPIO_OPEN_DRAIN_OUTPUT;

        pmicStatus = Pmic_gpioSetConfiguration(pPmicHandle,
                                               PMIC_TPS6594X_GPIO2_PIN,
                                               gpioCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   Function to setup the QA I2c interface for HERA PMIC
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
static int32_t test_pmic_hera_dual_i2c_pin_setup(Pmic_CoreHandle_t *pPmicHandle)
{
    int32_t pmicStatus     = PMIC_ST_SUCCESS;
    Pmic_GpioCfg_t gpioCfg = {0U};


    gpioCfg.validParams      = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                               PMIC_GPIO_CFG_OD_VALID_SHIFT;
    gpioCfg.pinFunc          = PMIC_LP8764X_GPIO_PINFUNC_GPIO2_SCL_I2C2;
    gpioCfg.outputSignalType = PMIC_GPIO_OPEN_DRAIN_OUTPUT;

    pmicStatus = Pmic_gpioSetConfiguration(pPmicHandle,
                                           PMIC_LP8764X_GPIO2_PIN,
                                           gpioCfg);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        gpioCfg.validParams = PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT |
                              PMIC_GPIO_CFG_OD_VALID_SHIFT;
        gpioCfg.pinFunc     = PMIC_LP8764X_GPIO_PINFUNC_GPIO3_SDA_I2C2;
        gpioCfg.outputSignalType = PMIC_GPIO_OPEN_DRAIN_OUTPUT;

        pmicStatus = Pmic_gpioSetConfiguration(pPmicHandle,
                                               PMIC_LP8764X_GPIO3_PIN,
                                               gpioCfg);
    }

    return pmicStatus;
}

/*!
 * \brief   Function to release the I2C lld interface for PMIC
 *
 * \param   pCommHandle    [OUT]     PMIC Interface Handle.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
static int32_t test_pmic_i2c_lld_intf_release(void **pCommHandle)
{
    if(NULL == (I2C_Handle)*pCommHandle)
    {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    I2C_close((I2C_Handle)*pCommHandle);

    *pCommHandle = NULL;

    return PMIC_ST_SUCCESS;
}

/*!
 * \brief   PMIC I2C read function.
 *
 * \param   pmicCorehandle [IN]     PMIC Interface Handle.
 * \param   commMode       [IN]     Communication Mode
 * \param   slaveAddr      [IN]     I2c device slave address
 * \param   regAddr        [OUT]    Register address to read from.
 * \param   pBuf           [IN]     Buffer to store data
 * \param   bufLen         [IN]     Length of data to read.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
int32_t test_pmic_regRead(Pmic_CoreHandle_t  *pmicCorehandle,
                          uint8_t             instType,
                          uint16_t            regAddr,
                          uint8_t            *pBuf,
                          uint8_t             bufLen)
{
    int8_t ret     = 0U;

    if((PMIC_INTF_SINGLE_I2C == pmicCorehandle->commMode) ||
       (PMIC_INTF_DUAL_I2C   == pmicCorehandle->commMode))
    {
        I2C_Transaction transaction;
        I2C_transactionInit(&transaction);

        /* Set register offset for read and write */
        transaction.readBuf    = pBuf;
        transaction.readCount  = bufLen;
        transaction.writeBuf   = &regAddr;
        transaction.writeCount = 1U;

        /* Main I2c BUS */
        if(PMIC_MAIN_INST == instType)
        {
            transaction.slaveAddress = pmicCorehandle->slaveAddr;
            ret = I2C_transfer((I2C_Handle)pmicCorehandle->pCommHandle,
                                &transaction);
            if(ret != I2C_STS_SUCCESS)
            {
                return PMIC_ST_ERR_I2C_COMM_FAIL;
            }
        }

        /* For WDOG QA I2C BUS */
        if(PMIC_QA_INST == instType)
        {
            transaction.slaveAddress = pmicCorehandle->qaSlaveAddr;
            if(PMIC_INTF_SINGLE_I2C == pmicCorehandle->commMode)
            {
                ret = I2C_transfer((I2C_Handle)
                                   pmicCorehandle->pCommHandle,
                                   &transaction);
            }
            if(PMIC_INTF_DUAL_I2C == pmicCorehandle->commMode)
            {
                ret = I2C_transfer((I2C_Handle)
                                   pmicCorehandle->pQACommHandle,
                                   &transaction);
            }
            if(ret != I2C_STS_SUCCESS)
            {
                return PMIC_ST_ERR_I2C_COMM_FAIL;
            }
        }
    }

    if(PMIC_INTF_SPI == pmicCorehandle->commMode)
    {
        if(PMIC_ST_SUCCESS !=
                 test_pmic_spi_stubRead(pmicCorehandle, pBuf, bufLen))
        {
            return PMIC_ST_ERR_SPI_COMM_FAIL;
        }
    }

    return PMIC_ST_SUCCESS;
}

/*!
 * \brief   PMIC I2C write function.
 *
 * \param   pmicCorehandle   [IN]     PMIC Interface Handle.
 * \param   commMode         [IN]     Communication Mode
 * \param   slaveAddr        [IN]     I2c device slave address
 * \param   regAddr          [IN]     Register address to write.
 * \param   pBuf             [IN]     Buffer to store data to write
 * \param   bufLen           [IN]     Length of data to write.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code
 *          For valid values see \ref Pmic_ErrorCodes
 */
int32_t test_pmic_regWrite(Pmic_CoreHandle_t  *pmicCorehandle,
                           uint8_t             instType,
                           uint16_t            regAddr,
                           uint8_t            *pBuf,
                           uint8_t             bufLen)
{
    int8_t  ret = 0U;

    if((PMIC_INTF_SINGLE_I2C == pmicCorehandle->commMode) ||
       (PMIC_INTF_DUAL_I2C   == pmicCorehandle->commMode))
    {
        I2C_Transaction transaction;

        /* regAddr, 8-bit-Data, 8-bit-CRC(optional) => max 3 bytes
         * Taking a databuff of size 3 only as it is enough
         */
        uint8_t dataBuff[16U] = {0U};

        /* Initializes the I2C transaction structure with default values */
        I2C_transactionInit(&transaction);

        if(PMIC_MAIN_INST == instType)
        {
            transaction.slaveAddress = pmicCorehandle->slaveAddr;
        }
        if(PMIC_QA_INST == instType)
        {
            transaction.slaveAddress = pmicCorehandle->qaSlaveAddr;
        }

        dataBuff[0U] = regAddr;
        memcpy(&dataBuff[1U], pBuf, bufLen);

        /* Control Byte followed by write bit */
        transaction.writeBuf     = dataBuff;
        transaction.writeCount   = bufLen + 1U;
        transaction.readCount    = 0U;
        transaction.readBuf      = NULL;

        /* Main I2c BUS */
        if(PMIC_MAIN_INST == instType)
        {
            ret = I2C_transfer((I2C_Handle)pmicCorehandle->pCommHandle,
                                &transaction);
            if(ret != I2C_STS_SUCCESS)
            {
                return PMIC_ST_ERR_I2C_COMM_FAIL;
            }
        }

        /* For WDOG QA I2C BUS */
        if(PMIC_QA_INST == instType)
        {
            if(PMIC_INTF_SINGLE_I2C == pmicCorehandle->commMode)
            {
                ret = I2C_transfer((I2C_Handle)
                                   pmicCorehandle->pCommHandle,
                                   &transaction);
            }
            if(PMIC_INTF_DUAL_I2C == pmicCorehandle->commMode)
            {
                ret = I2C_transfer((I2C_Handle)
                                   pmicCorehandle->pQACommHandle,
                                   &transaction);
            }
            if(ret != I2C_STS_SUCCESS)
            {
                return PMIC_ST_ERR_I2C_COMM_FAIL;
            }
        }
    }

    if(PMIC_INTF_SPI == pmicCorehandle->commMode)
    {
        if(PMIC_ST_SUCCESS !=
                   test_pmic_spi_write(pmicCorehandle, pBuf, bufLen))
        {
            return PMIC_ST_ERR_SPI_COMM_FAIL;
        }
    }

    return PMIC_ST_SUCCESS;
}

/*!
 * \brief   Interface setup function for PMIC to create instance and
 *          initialise the SPI Bus for PMIC Communication
 *
 * \param   pmicCorehandle    [IN]     PMIC Interface Handle.
 * \param   commMode          [IN]     Communication Mode
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
static int32_t test_pmic_spi_lld_intf_setup(Pmic_CoreCfg_t *pPmicConfigData)
{
    int32_t ret = PMIC_ST_SUCCESS;

    ret = test_pmic_spi_stubInit(pPmicConfigData);

    return ret;
}

/*!
 * \brief   Interface release function for PMIC to release the SPI Bus
 *
 * \param   pCommHandle    [IN]     PMIC Interface Handle.
 * \param   commMode       [IN]     Communication Mode
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
static int32_t test_pmic_spi_lld_intf_release(void **pCommHandle)
{
    int32_t ret = PMIC_ST_SUCCESS;

    ret = test_pmic_spi_stubDeinit(pCommHandle);

    return ret;
}

/*!
 * \brief   Initialize Semaphore Handle
 */
static void test_pmic_osalSemaphoreInit(void)
{
    SemaphoreP_Params pmic_SemParams;

    /* Create call back semaphore */
    SemaphoreP_Params_init(&pmic_SemParams);
    pmic_SemParams.mode = SemaphoreP_Mode_BINARY;
    pmic_Sem = SemaphoreP_create(1U, &pmic_SemParams);
}

/*!
 * \brief   DeInit Semaphore Handle
 */
static void test_pmic_osalSemaphoreDeInit(void)
{
    if(pmic_Sem)
    {
        SemaphoreP_delete(pmic_Sem);
        pmic_Sem = NULL;
    }
}

/*!
 * \brief   PMIC Critical section Lock function
 *          This function should have OS specific locking setup and should
 *          assigned to 'pmicConfigData->pFnPmicCritSecStart'
 */
void test_pmic_criticalSectionStartFn(void)
{

    if(SemaphoreP_OK != SemaphoreP_pend(pmic_Sem,
                                        SemaphoreP_WAIT_FOREVER))
    {
        pmic_log("%s(): Invalid Semaphore Handle\n", __func__);
    }
}

/*!
 * \brief   PMIC Critical section Unlock function
 *          This function should have OS specific locking setup and should
 *          assigned to 'pmicConfigData->pFnPmicCritSecStop'
 */
void test_pmic_criticalSectionStopFn(void)
{
    if(SemaphoreP_OK != SemaphoreP_post(pmic_Sem))
    {
        pmic_log("%s(): Invalid Semaphore Handle\n", __func__);
    }
}

/*!
 * \brief   Get PMIC StartUp Interrupt Type.
 *          This function deciphers all interrupts and find startup type.
 */
static uint32_t get_startup_type(Pmic_CoreHandle_t *pPmicCoreHandle)
{
    Pmic_IrqStatus_t errStat  = {0U};
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t type = 0U;

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, false);
    switch(pPmicCoreHandle->pmicDeviceType)
    {
        case PMIC_DEV_LEO_TPS6594X:
            switch(startup_type)
            {
                case PMIC_ENABLE_STARTUP_TYPE:
                    if((PMIC_ST_SUCCESS == pmicStatus) &&
                       ((errStat.intStatus[PMIC_TPS6594X_ENABLE_INT/32U] &
                        (1U << (PMIC_TPS6594X_ENABLE_INT % 32U))) != 0U))
                    {
                        type = PMIC_ENABLE_STARTUP_TYPE;
                    }
                    break;
                case PMIC_NPWRON_STARTUP_TYPE:
                    if((PMIC_ST_SUCCESS == pmicStatus) &&
                       ((errStat.intStatus[PMIC_TPS6594X_NPWRON_START_INT/32U] &
                        (1U << (PMIC_TPS6594X_NPWRON_START_INT % 32U))) != 0U))
                    {
                        type = PMIC_NPWRON_STARTUP_TYPE;
                    }
                    break;
                case PMIC_FSD_STARTUP_TYPE:
                    if((PMIC_ST_SUCCESS == pmicStatus) &&
                       ((errStat.intStatus[PMIC_TPS6594X_FSD_INT/32U] &
                        (1U << (PMIC_TPS6594X_FSD_INT % 32U))) != 0U))
                    {
                        type = PMIC_FSD_STARTUP_TYPE;
                    }
                    break;
                default:
                        type = 0U;
                    break;
            }
            break;
        case PMIC_DEV_HERA_LP8764X:
            switch(startup_type)
            {
                case PMIC_ENABLE_STARTUP_TYPE:
                    if((PMIC_ST_SUCCESS == pmicStatus) &&
                       ((errStat.intStatus[PMIC_LP8764X_ENABLE_INT/32U] &
                        (1U << (PMIC_LP8764X_ENABLE_INT % 32U))) != 0U))
                    {
                        type = PMIC_ENABLE_STARTUP_TYPE;
                    }
                    break;
                case PMIC_FSD_STARTUP_TYPE:
                    if((PMIC_ST_SUCCESS == pmicStatus) &&
                       ((errStat.intStatus[PMIC_LP8764X_FSD_INT/32U] &
                        (1U << (PMIC_LP8764X_FSD_INT % 32U))) != 0U))
                    {
                        type = PMIC_FSD_STARTUP_TYPE;
                    }
                    break;
                default:
                    type = 0U;
                    break;
            }
            break;
    }

    startup_type = type;

    return pmicStatus;
}
/*!
 * \brief   PMIC Interrupt decipher and clear function
 *          This function deciphers all interrupts and clears the status
 */
static int32_t Pmic_intrClr(Pmic_CoreHandle_t *pmicHandle)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t handle  = *(Pmic_CoreHandle_t *)pmicHandle;
    Pmic_IrqStatus_t errStat  = {0U};

    if(startup_type != 0U)
    {
        pmicStatus = get_startup_type(pmicHandle);
    }
    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_irqGetErrStatus(&handle, &errStat, true);
    }

    return pmicStatus;
}

/*!
 * \brief   Initialize PMIC Instance and corresponding Interface.
 *
 * \param   pmicCoreHandle    [OUT]     PMIC Core Handle.
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int32_t test_pmic_appInit(Pmic_CoreHandle_t **pmicCoreHandle,
                          Pmic_CoreCfg_t     *pmicConfigData)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *pmicHandle = NULL;

    /* Initialize Pmic Semaphore */
    test_pmic_osalSemaphoreInit();

    if(pmicCoreHandle == NULL)
    {
        pmic_log("Invalid PMIC core Handle Reference\n");
        return PMIC_ST_ERR_INV_HANDLE;
    }

    if(pmicConfigData == NULL)
    {
        pmic_log("Invalid PMIC config Data - NULL \n");
        return PMIC_ST_ERR_NULL_PARAM;
    }


    /* Allocate memory for PMIC core Handle */
    pmicHandle = malloc(sizeof(Pmic_CoreHandle_t));
    if(pmicHandle == NULL)
    {
        pmic_log("Failed to allocate memory to pmicHandle\n");
        return PMIC_ST_ERR_INV_HANDLE;
    }

    memset(pmicHandle, 0, sizeof(Pmic_CoreHandle_t));

    /* For single I2C Instance */
    if(PMIC_INTF_SINGLE_I2C == pmicConfigData->commMode)
    {
        /* Get PMIC valid Main I2C Instance */
        pmicStatus = test_pmic_i2c_lld_intf_setup(pmicConfigData,
                                                  PMIC_MAIN_INST);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicConfigData->validParams |= PMIC_CFG_COMM_HANDLE_VALID_SHIFT;
            /* Update instance type to pmicConfigData */
            pmicConfigData->instType = PMIC_MAIN_INST;
            /* Get PMIC core Handle for Main Instance */
            pmicStatus = Pmic_init(pmicConfigData, pmicHandle);
        }
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Setup nSLEEP signals */
            pmicStatus = Pmic_fsmDeviceOnRequest(pmicHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_intrClr(pmicHandle);
        }
    }
    /* For DUAL I2C Instance */
    else if(PMIC_INTF_DUAL_I2C == pmicConfigData->commMode)
    {
        /* Get PMIC valid Main I2C Instance */
        pmicStatus = test_pmic_i2c_lld_intf_setup(pmicConfigData,
                                                  PMIC_MAIN_INST);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicConfigData->validParams |= PMIC_CFG_COMM_HANDLE_VALID_SHIFT;
            /* Update instance type to pmicConfigData */
            pmicConfigData->instType = PMIC_MAIN_INST;
            /* Get PMIC core Handle for Main Instance */
            pmicStatus = Pmic_init(pmicConfigData, pmicHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Setup nSLEEP signals */
            pmicStatus = Pmic_fsmDeviceOnRequest(pmicHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_intrClr(pmicHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            if(PMIC_DEV_LEO_TPS6594X == pmicHandle->pmicDeviceType)
            {
                /* Setup leo pmic Dual I2C functionality to GPIO-1 and GPIO-2 pins */
                pmicStatus = test_pmic_leo_dual_i2c_pin_setup(pmicHandle);
            }
            if(PMIC_DEV_HERA_LP8764X == pmicHandle->pmicDeviceType)
            {
                /* Setup hera pmic Dual I2C functionality to GPIO-1 and GPIO-2 pins */
                pmicStatus = test_pmic_hera_dual_i2c_pin_setup(pmicHandle);
            }
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Get PMIC valid QA I2C Instance */
            pmicStatus = test_pmic_i2c_lld_intf_setup(pmicConfigData,
                                                      PMIC_QA_INST);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicConfigData->validParams |= PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT;
            /* Update instance type to pmicConfigData */
            pmicConfigData->instType = PMIC_QA_INST;
            /* Get PMIC core Handle for QA Instances */
            pmicStatus = Pmic_init(pmicConfigData, pmicHandle);
        }

    }
    /* For SPI Instance */
    else if(PMIC_INTF_SPI  == pmicConfigData->commMode)
    {
        /* Get PMIC valid Main SPI Communication Handle */
        pmicStatus = test_pmic_spi_lld_intf_setup(pmicConfigData);
        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_init(pmicConfigData, pmicHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Setup nSLEEP signals */
            pmicStatus = Pmic_fsmDeviceOnRequest(pmicHandle);
        }

        if(PMIC_ST_SUCCESS == pmicStatus)
        {
            pmicStatus = Pmic_intrClr(pmicHandle);
        }
    }

    if(PMIC_ST_SUCCESS != pmicStatus)
    {
        pmic_log("%s(): %d: FAILED with status: %d\n",
                            __func__, __LINE__,  pmicStatus);
    }

    *pmicCoreHandle = pmicHandle;

    return pmicStatus;
}

void test_pmic_appDeInit(Pmic_CoreHandle_t *pmicCoreHandle)
{
    if(PMIC_INTF_SINGLE_I2C == pmicCoreHandle->commMode)
    {
        test_pmic_i2c_lld_intf_release(&pmicCoreHandle->pCommHandle);
    }
    else if(PMIC_INTF_DUAL_I2C == pmicCoreHandle->commMode)
    {
        test_pmic_i2c_lld_intf_release(&pmicCoreHandle->pCommHandle);
        test_pmic_i2c_lld_intf_release(&pmicCoreHandle->pQACommHandle);
    }
    else if(PMIC_INTF_SPI  == pmicCoreHandle->commMode)
    {
        test_pmic_spi_lld_intf_release(&pmicCoreHandle->pCommHandle);
    }

    Pmic_deinit(pmicCoreHandle);

    /* PMIC Semaphore Clean-up */
    test_pmic_osalSemaphoreDeInit();
}

/*!
 * \brief   Unity setUp Function
 */
void setUp(void)
{
    /* Do nothing */
}

/*!
 * \brief   Unity tearDown Function
 */
void tearDown(void)
{
    /* Do nothing */
}


/*!
 * \brief   PMIC Application Callback Function
 */
void AppPmicCallbackFxn(void)
{
    int32_t status           = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat = {0U};
    uint8_t irqNum           = 0U;
    pmic_intr_triggered      = 1U;

    status = Pmic_irqGetErrStatus(pPmicCoreHandleISR, &errStat, false);
    if((PMIC_ST_SUCCESS == status) &&
       ((errStat.intStatus[irqNumToClr/32U] &
         (1U << (irqNumToClr % 32U))) != 0U))
    {
        while(irqNumToClr != irqNum)
        {
            status = Pmic_getNextErrorStatus(pPmicCoreHandleISR,
                                             &errStat,
                                             &irqNum);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            /* clear the interrupt */
            status = Pmic_irqClrErrStatus(pPmicCoreHandleISR,
                                          irqNumToClr);
        }
    }
}

/*!
 * \brief   GPIO Interrupt Router Configuration
 */
void GPIO_configIntRouter(uint32_t portNum, uint32_t pinNum, uint32_t gpioIntRtrOutIntNum, GPIO_v0_HwAttrs *cfg)
{
    GPIO_IntCfg       *intCfg;
    uint32_t           bankNum;

    intCfg = cfg->intCfg;

    cfg->baseAddr = CSL_WKUP_GPIO0_BASE;

    bankNum = pinNum/16; /* Each GPIO bank has 16 pins */

    /* WKUP GPIO int router input interrupt is the GPIO bank interrupt */
    intCfg[pinNum].intNum = CSLR_MCU_R5FSS0_CORE0_INTR_WKUP_GPIOMUX_INTRTR0_OUTP_0 + bankNum;
    intCfg[pinNum].intcMuxNum = INVALID_INTC_MUX_NUM;
    intCfg[pinNum].intcMuxInEvent = 0;
    intCfg[pinNum].intcMuxOutEvent = 0;
}

/*!
 * \brief   UART Configurations
 */
void Board_initUART(void)
{
    Board_initCfg boardCfg;

    boardCfg = BOARD_INIT_PINMUX_CONFIG |
               BOARD_INIT_MODULE_CLOCK  |
               BOARD_INIT_UART_STDIO;

    Board_init(boardCfg);
}

/*!
 * \brief   GPIO Configurations.
 *          This API is required for Asynchronous Interrupts only
 */
void App_initGPIO(void)
{
    GPIO_v0_HwAttrs gpio_cfg;

    /* Get the default SPI init configurations */
    GPIO_socGetInitCfg(J7_WAKEUP_GPIO0_PORT_NUM, &gpio_cfg);

    /* change default GPIO port from MAIN GPIO0 to WAKEUP GPIO0 to access TP45 */
    gpio_cfg.baseAddr = CSL_WKUP_GPIO0_BASE;
    gpio_cfg.intCfg->intNum = CSLR_MCU_R5FSS0_CORE0_INTR_WKUP_GPIOMUX_INTRTR0_OUTP_0;
    
    GPIO_configIntRouter(J7_WAKEUP_GPIO0_PORT_NUM, J7_WAKEUP_GPIO0_9_PIN_NUM, 0, &gpio_cfg);

    /* For J721E EVM, there is not GPIO pin directly connected to LEDs */
    /* J7ES: use WAKEUP GPIO0_6 --> TP45 for testing */
    /* Set the default GPIO init configurations */
    GPIO_socSetInitCfg(J7_WAKEUP_GPIO0_PORT_NUM, &gpio_cfg);

    /* GPIO initialization */
    GPIO_init();

    /* Set the callback function */
    GPIO_setCallback(0, AppPmicCallbackFxn);
}

#if defined(BUILD_MPU) || defined (__C7100__)
extern void Osal_initMmuDefault(void);
void InitMmu(void)
{
    Osal_initMmuDefault();
}
#endif

static uint32_t test_pmic_get_unity_testcase_index(
                                             uint32_t         testId,
                                             Pmic_Ut_Tests_t *pTest,
                                             uint32_t         num_testcases)
{
    uint32_t i;
    int32_t status = PMIC_ST_SUCCESS;

    for (i = 0U; ; i++)
    {
        if(pTest->testId == testId)
        {
            break;
        }
        else if(i >= num_testcases)
        {
            status = PMIC_ST_ERR_FAIL;
            /* Print test Result */
            pmic_log("\n |TEST RESULT|:: %d ::", testId);
            TEST_ASSERT_EQUAL_MESSAGE(PMIC_ST_SUCCESS,
                                      status,
                                      "Invalid Test ID");
        }
        pTest++;
    }

    return i;
}

/*!
 * \brief   Function to print testcase info
 */
void test_pmic_print_unity_testcase_info(uint32_t         testId,
                                         Pmic_Ut_Tests_t *pTest,
                                         uint32_t         num_testcases)
{
    uint32_t index = 0U;

    /* Print unit test ID */
    pmic_log("\n\n |TEST ID|:: %d ::\n", testId);

    index = test_pmic_get_unity_testcase_index(testId, pTest, num_testcases);

    /* Print test description */
    pmic_log("\n |TEST NAME|:: %s ::\n", pTest[index].testDesc);

    /* Print test Result */
    pmic_log("\n |TEST RESULT|:: %d ::", testId);
}
