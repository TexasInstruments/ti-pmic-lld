/****************************************************************************
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 ****************************************************************************/

/**
 *  @file  pmic_test_common.c
 *
 *  @brief  This file contains all the testing related files APIs for the common tests.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_test_common.h"

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

/**
 * @brief  Delay execution for a specified duration.
 * This function introduces a delay in the execution flow for the specified
 * duration. The delay is achieved through a simple loop that increments a
 * counter until the desired duration is reached. The loop is designed to
 * provide an approximate delay and may be affected by compiler optimizations.
 *
 * @param  milliseconds Duration of the delay in milliseconds.
 * @return NULL.
 */
void delay(uint32_t milliseconds) {
    volatile uint32_t counter = 0;
    uint32_t delayCount = milliseconds * 1000;

    while (counter < delayCount) {
        counter++;
    }
}

/**
 * @brief  Initiates the start of a critical section for PMIC operations.
 * This function attempts to acquire a semaphore, which is typically
 * used to ensure exclusive access to resources during PMIC operations.
 * If the semaphore acquisition fails, an error message is logged.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_criticalSectionStartFn(void)
{
    if (SemaphoreP_OK != SemaphoreP_pend(&gpmicCoreObj,
                                         SemaphoreP_WAIT_FOREVER))
    {
        DebugP_log("%s(): Invalid Semaphore Handle\n", __func__);
    }
}

/**
 * @brief  Concludes a critical section for PMIC operations.
 * This function releases the semaphore, signifying
 * the end of a critical section initiated by a
 * corresponding "start" function.
 *
 * @param   void
 * @return  NULL
 */
void test_pmic_criticalSectionStopFn(void)
{
    SemaphoreP_post(&gpmicCoreObj);
}

/**
 * @brief  Initializes the PMIC core for testing purposes.
 * This function initializes the PMIC core for testing by
 * configuring communication parameters, allocating memory
 * for the PMIC core handle, and initializing the PMIC using Pmic_init.
 *
 * @param pmicCoreHandle Pointer to the PMIC core handle structure.
 * @param pmicConfigData Pointer to the PMIC configuration data structure.
 * @return  Status of the initialization process.
 *          - #PMIC_ST_SUCCESS if initialization is successful.
 *          - An error code if initialization fails.
 */
int32_t test_pmic_appInit(Pmic_CoreHandle_t **pmicCoreHandle,
                          Pmic_CoreCfg_t     *pmicConfigData)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t *pmicHandle = NULL;

    /* Initialize Pmic Semaphore */
    pmicStatus = SemaphoreP_constructMutex(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("Error: SemaphoreP_constructMutex!!!\r\n");
        return PMIC_ST_ERR_INV_HANDLE;
    }

    if (pmicCoreHandle == NULL)
    {
        DebugP_log("Invalid PMIC core Handle Reference\r\n");

        return PMIC_ST_ERR_INV_HANDLE;
    }

    if (pmicConfigData == NULL)
    {
        DebugP_log("Invalid PMIC config Data - NULL\r\n");

        return PMIC_ST_ERR_NULL_PARAM;
    }

    /* Allocate memory for PMIC core Handle */
    pmicHandle = malloc(sizeof(Pmic_CoreHandle_t));

    if (pmicHandle == NULL)
    {
        DebugP_log("Failed to allocate memory to pmicHandle\r\n");

        return PMIC_ST_ERR_INV_HANDLE;
    }

    memset(pmicHandle, 0, sizeof(Pmic_CoreHandle_t));

    /* For SPI Instance */
    if (PMIC_INTF_SPI  == pmicConfigData->commMode)
    {
        if (PMIC_ST_SUCCESS == pmicStatus)
        {
            /* Update MAIN instance type to pmicConfigData for SPI */
            pmicConfigData->instType = PMIC_MAIN_INST;
            if (PMIC_ST_SUCCESS != pmicStatus)
            {
                DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                         __func__, __LINE__,  pmicStatus);
            }
            pmicStatus = Pmic_init(pmicConfigData, pmicHandle);
            if (PMIC_ST_SUCCESS != pmicStatus)
            {
                DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                         __func__, __LINE__,  pmicStatus);
            }
            /*
             * Check for Warning message due to Invalid Device ID.
             * And continue the application with WARNING message.
             */
            if (PMIC_ST_WARN_INV_DEVICE_ID == pmicStatus)
            {
                DebugP_log("\n*** WARNING: Found Invalid DEVICE ID ***\r\n\n");
                pmicStatus = PMIC_ST_SUCCESS;
            }
        }
    }

    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__,  pmicStatus);
    }

    *pmicCoreHandle = pmicHandle;

    return pmicStatus;
}

/**
 * @brief  Deinitializes the PMIC Semaphore.
 * This function deinitializes the PMIC Semaphore by removing
 * the underlying Semaphore object.
 *
 * @param  void
 * @return NULL
 */
static void test_pmic_SemaphoreDeInit(void)
{
    SemaphoreP_destruct(&gpmicCoreObj);
}

/**
 * @brief  Reads a register from the PMIC device.
 * This function performs a read operation on a specified register of
 * the PMIC device using the configured SPI communication. It utilizes
 * the MCSPI driver to execute the read transaction and updates the
 * received data in the provided buffer.
 *
 * @param pmicCorehandle  Pointer to the PMIC core handle.
 * @param instType        PMIC instance type.
 * @param regAddr         Register address to read.
 * @param pBuf            Pointer to the buffer to store the read data.
 * @param bufLen          Length of the buffer.
 * @return  Status of the register read operation.
 *          - #SystemP_SUCCESS if the read operation is successful.
 *          - #SystemP_FAILURE if the read operation fails.
 */
int32_t test_pmic_regRead(Pmic_CoreHandle_t  *pmicCorehandle,
                          uint8_t             instType,
                          uint16_t            regAddr,
                          uint8_t            *pBuf,
                          uint8_t             bufLen)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t pBuf_cmd      = *(pBuf);
    uint8_t pBuf_data     = 0;

    MCSPI_Transaction   spiTransaction;
    MCSPI_Transaction_init(&spiTransaction);
    spiTransaction.channel  = gConfigMcspi1ChCfg[0].chNum;
    spiTransaction.dataSize = (uint32_t)32;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = (uint32_t)1;
    spiTransaction.txBuf    = (void *)gPmicMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gPmicMcspiRxBuffer;
    spiTransaction.args     = NULL;

    status = PMIC_mcspiReadRegister(gMcspiHandle[CONFIG_MCSPI1], &spiTransaction, pBuf_cmd, &pBuf_data);
    if(status != SystemP_SUCCESS)
    {
        status = SystemP_FAILURE;
        return status;
    }
    *(pBuf + 2) = pBuf_data;
    return status;
}

/**
 * @brief  Writes data to a register of the PMIC device.
 * This function performs a write operation to a specified register of the
 * PMIC device using the configured SPI communication. It utilizes the
 * MCSPI driver to execute the write transaction with the provided data.
 *
 * @param pmicCorehandle Pointer to the PMIC core handle.
 * @param instType       PMIC instance type.
 * @param regAddr        Register address to write.
 * @param pBuf           Pointer to the buffer containing the data to write.
 * @param bufLen         Length of the buffer.
 * @return  Status of the register write operation.
 *          - #SystemP_SUCCESS if the write operation is successful.
 *          - #SystemP_FAILURE if the write operation fails.
 */
int32_t test_pmic_regWrite(Pmic_CoreHandle_t  *pmicCorehandle,
                           uint8_t             instType,
                           uint16_t            regAddr,
                           uint8_t            *pBuf,
                           uint8_t             bufLen)
{
    int32_t status = SystemP_SUCCESS;
    uint8_t pBuf_cmd     = *(pBuf);
    uint8_t pBuf_data     = *(pBuf + 2);

    MCSPI_Transaction   spiTransaction;
    MCSPI_Transaction_init(&spiTransaction);
    spiTransaction.channel  = gConfigMcspi1ChCfg[0].chNum;
    spiTransaction.dataSize = (uint32_t)32;
    spiTransaction.csDisable = TRUE;
    spiTransaction.count    = (uint32_t)1;
    spiTransaction.txBuf    = (void *)gPmicMcspiTxBuffer;
    spiTransaction.rxBuf    = (void *)gPmicMcspiRxBuffer;
    spiTransaction.args     = NULL;

    status = PMIC_mcspiWriteRegister(gMcspiHandle[CONFIG_MCSPI1], &spiTransaction, pBuf_cmd, pBuf_data);
    if(status != SystemP_SUCCESS)
    {
        status = SystemP_FAILURE;
        return status;
    }
    return status;
}

/**
 * @brief  Checks and logs the status of configuration registers lock.
 * This function checks and logs the status of configuration
 * registers lock by calling `Pmic_getRegLockStat`.
 *
 * @param pmicCorehandle Pointer to the PMIC core handle.
 * @return  NULL
 */
void test_check_lock_config_reg(Pmic_CoreHandle_t  *pmicCorehandle)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t   *pCommonCtrlStat = NULL;
    DebugP_log("Checking config register status...\r\n");
    pmicStatus = Pmic_getRegLockStat(pmicCorehandle, pCommonCtrlStat);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    if (pCommonCtrlStat->cfgregLockStat == 0x00)   {
    DebugP_log("Configuration registers are Unlocked!\r\n\n");
    }
    else    {
    DebugP_log("Configuration registers are Locked!\r\n\n");
    }
}

/**
 * @brief  Checks and logs the status of Timer and Rotational Counter registers lock.
 * This function checks and logs the status of Timer and Rotational Counter
 * registers lock by calling `Pmic_getTmrCntLockStat`.
 *
 * @param pmicCorehandle Pointer to the PMIC core handle.
 * @return  NULL
 */
void test_check_tmr_cnt_config_reg(Pmic_CoreHandle_t  *pmicCorehandle)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t   *pCommonCtrlStat = NULL;
    DebugP_log("\rChecking TMR and CNT register status...\r\n");
    pmicStatus = Pmic_getTmrCntLockStat(pmicCorehandle, pCommonCtrlStat);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    if (pCommonCtrlStat->cntregLockStat == 0x00)   {
    DebugP_log("Timer and Rotational Counter registers are Unlocked!\r\n\n");
    }
    else    {
    DebugP_log("Timer and Rotational Counter registers are Locked!\r\n\n");
    }
}

/**
 * @brief  Locks or unlocks configuration registers based on the provided flag.
 * This function either locks or unlocks configuration registers based
 * on the provided `unlock` flag by calling the `Pmic_setRegisterLockUnlock` function.
 *
 * @param pmicCorehandle Pointer to the PMIC core handle.
 * @param unlock Flag indicating whether to unlock (1) or lock (0) the configuration registers.
 * @return  NULL
 */
void test_pmic_LockUnlock(Pmic_CoreHandle_t  *pmicCorehandle, int unlock)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t  commonCtrlCfg;

    if(unlock) {
        commonCtrlCfg.regLock_1 = PMIC_REGISTER_UNLOCK_DATA1;
        commonCtrlCfg.regLock_2 = PMIC_REGISTER_UNLOCK_DATA2;
        DebugP_log("Unlocking config reg...\r\n");
    }
    else {
        commonCtrlCfg.regLock_1 = PMIC_REGISTER_LOCK_1;
        commonCtrlCfg.regLock_2 = PMIC_REGISTER_LOCK_2;
        DebugP_log("Locking config reg...\r\n");
    }

    pmicStatus = Pmic_setRegisterLockUnlock(pmicCorehandle, commonCtrlCfg);

    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }

    test_check_lock_config_reg(pmicCorehandle);
}

/**
 * @brief  Locks or unlocks Timer and Rotational Counter registers based on the
 * provided flag. This function either locks or unlocks Timer and Rotational
 * Counter registers based on the provided `unlock` flag by calling the
 * `Pmic_setCounterLockUnlock` function.
 *
 * @param   unlock [IN] unlock Flag indicating whether to unlock (1) or lock (0) Timer and Rotational Counter registers.
 * @retun   NULL
 */
void test_pmic_CNT_LockUnlock(Pmic_CoreHandle_t  *pmicCorehandle, int unlock)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t  commonCtrlCfg;

    if(unlock) {
        commonCtrlCfg.cntLock_1 = PMIC_TMR_COUNTER_UNLOCK_DATA1;
        commonCtrlCfg.cntLock_2 = PMIC_TMR_COUNTER_UNLOCK_DATA2;
        DebugP_log("Unlocking timer and counter reg...\r\n");
    }
    else {
        commonCtrlCfg.cntLock_1 = PMIC_REGISTER_LOCK_1;
        commonCtrlCfg.cntLock_2 = PMIC_REGISTER_LOCK_2;
        DebugP_log("Locking timer and counter reg...\r\n");
    }

    pmicStatus = Pmic_setCounterLockUnlock(pmicCorehandle, commonCtrlCfg);

    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n", __func__, __LINE__, pmicStatus);
    }

    test_check_tmr_cnt_config_reg(pmicCorehandle);
}


