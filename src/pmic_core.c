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
 * @file pmic_core.c
 *
 * @brief This file contains definitions to APIs that interact with the PMIC Core.
 */
#include "pmic.h"
#include "pmic_io.h"
#include "pmic_core.h"

#include "regmap/core.h"
#include "regmap/irq.h"

#define NUM_DATA_BYTES_FOR_CFG_REG_CRC ((uint8_t)16U)

/**
 * Used CRC Polynomial:  x^8 + x^2 + x + 1
 *   Evalution of CRC Polynomial value from equation:
 *     1*x^8 + 0*x^7 + 0*x^6 + 0*x^5 + 0*x^4 + 0*x^3 + 1*x^2 + 1*x + 1
 *  =  1     + 0     + 0     + 0     + 0     + 0     + 1     + 1   + 1
 *  =  0x107(After discarding most significant bit) = 0x7
 *
 * CRC Polynomial value: 0x07, Initial Value: 0x00, Final Value: 0x00
 * link to generate custom CRC table from polynomial:
 * http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 */
static const uint8_t CRC_8_TABLE[] =
{
    0x00U, 0x07U, 0x0eU, 0x09U, 0x1cU, 0x1bU, 0x12U, 0x15U,
    0x38U, 0x3fU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2aU, 0x2dU,
    0x70U, 0x77U, 0x7eU, 0x79U, 0x6cU, 0x6bU, 0x62U, 0x65U,
    0x48U, 0x4fU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5aU, 0x5dU,
    0xe0U, 0xe7U, 0xeeU, 0xe9U, 0xfcU, 0xfbU, 0xf2U, 0xf5U,
    0xd8U, 0xdfU, 0xd6U, 0xd1U, 0xc4U, 0xc3U, 0xcaU, 0xcdU,
    0x90U, 0x97U, 0x9eU, 0x99U, 0x8cU, 0x8bU, 0x82U, 0x85U,
    0xa8U, 0xafU, 0xa6U, 0xa1U, 0xb4U, 0xb3U, 0xbaU, 0xbdU,
    0xc7U, 0xc0U, 0xc9U, 0xceU, 0xdbU, 0xdcU, 0xd5U, 0xd2U,
    0xffU, 0xf8U, 0xf1U, 0xf6U, 0xe3U, 0xe4U, 0xedU, 0xeaU,
    0xb7U, 0xb0U, 0xb9U, 0xbeU, 0xabU, 0xacU, 0xa5U, 0xa2U,
    0x8fU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9dU, 0x9aU,
    0x27U, 0x20U, 0x29U, 0x2eU, 0x3bU, 0x3cU, 0x35U, 0x32U,
    0x1fU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0dU, 0x0aU,
    0x57U, 0x50U, 0x59U, 0x5eU, 0x4bU, 0x4cU, 0x45U, 0x42U,
    0x6fU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7dU, 0x7aU,
    0x89U, 0x8eU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9bU, 0x9cU,
    0xb1U, 0xb6U, 0xbfU, 0xb8U, 0xadU, 0xaaU, 0xa3U, 0xa4U,
    0xf9U, 0xfeU, 0xf7U, 0xf0U, 0xe5U, 0xe2U, 0xebU, 0xecU,
    0xc1U, 0xc6U, 0xcfU, 0xc8U, 0xddU, 0xdaU, 0xd3U, 0xd4U,
    0x69U, 0x6eU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7bU, 0x7cU,
    0x51U, 0x56U, 0x5fU, 0x58U, 0x4dU, 0x4aU, 0x43U, 0x44U,
    0x19U, 0x1eU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0bU, 0x0cU,
    0x21U, 0x26U, 0x2fU, 0x28U, 0x3dU, 0x3aU, 0x33U, 0x34U,
    0x4eU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5cU, 0x5bU,
    0x76U, 0x71U, 0x78U, 0x7fU, 0x6aU, 0x6dU, 0x64U, 0x63U,
    0x3eU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2cU, 0x2bU,
    0x06U, 0x01U, 0x08U, 0x0fU, 0x1aU, 0x1dU, 0x14U, 0x13U,
    0xaeU, 0xa9U, 0xa0U, 0xa7U, 0xb2U, 0xb5U, 0xbcU, 0xbbU,
    0x96U, 0x91U, 0x98U, 0x9fU, 0x8aU, 0x8dU, 0x84U, 0x83U,
    0xdeU, 0xd9U, 0xd0U, 0xd7U, 0xc2U, 0xc5U, 0xccU, 0xcbU,
    0xe6U, 0xe1U, 0xe8U, 0xefU, 0xfaU, 0xfdU, 0xf4U, 0xf3U
};

int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = PMIC_ST_SUCCESS;
    const bool invalidHandleConditions = (
        (pmicHandle->ioTransfer == NULL) ||
        (pmicHandle->drvInitStat != PMIC_DRV_INIT));

    if (pmicHandle == NULL)
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && invalidHandleConditions)
    {
        status = PMIC_ST_ERR_INV_HANDLE;
    }

    return status;
}

int32_t Pmic_setRegLock(const Pmic_CoreHandle_t *pmicHandle, bool lock)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        if (lock)
        {
            status = Pmic_ioTxByte_CS(pmicHandle, PMIC_CMD_SW_LOCK, PMIC_CMD_SW_LOCK_DATA);
        }
        else
        {
            status = Pmic_ioTxByte_CS(pmicHandle, PMIC_CMD_SW_UNLOCK, PMIC_CMD_SW_UNLOCK_DATA);
        }
    }

    return status;
}

int32_t Pmic_unlockRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setRegLock(pmicHandle, PMIC_UNLOCK);
}

int32_t Pmic_lockRegs(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setRegLock(pmicHandle, PMIC_LOCK);
}

static int32_t CORE_setAutoBistDisAndSafeLockToDis(const Pmic_CoreHandle_t *pmicHandle, const Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DISABLE_AUTO_BIST_VALID))
    {
        // Read SAFETY_BIST_CTRL register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_BIST_CTRL, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify AUTO_BIST_DIS bit field
            Pmic_setBitField_b(&regData, AUTO_BIST_DIS_SHIFT, deviceCfg->disableAutoBIST);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_BIST_CTRL, regData);
        }
    }

    if (Pmic_validParamStatusCheck(deviceCfg->validParams, PMIC_DISABLE_SAFE_LOCK_TIMEOUT_VALID, status))
    {
        // Read SAFETY_FUNC_CFG register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify SAFE_LOCK_TO_DIS bit field
            Pmic_setBitField_b(&regData, SAFE_LOCK_TO_DIS_SHIFT, deviceCfg->disableSafeLockTimeout);

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_FUNC_CFG, regData);
        }
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_writeSafetyCheckCtrl(const Pmic_CoreHandle_t *pmicHandle, const Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t safetyCheckCtrlValidParams = (PMIC_ENABLE_DRV_VALID | PMIC_DIAG_EXIT_VALID | PMIC_DIAG_EXIT_MASK_VALID);

    if (Pmic_validParamCheck(deviceCfg->validParams, safetyCheckCtrlValidParams))
    {
        // Read SAFETY_CHECK_CTRL register
        Pmic_criticalSectionStart(pmicHandle);
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify ENABLE_DRV bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_ENABLE_DRV_VALID))
            {
                Pmic_setBitField_b(&regData, ENABLE_DRV_SHIFT, deviceCfg->enableDrv);
            }

            // Modify DIAG_EXIT_MASK bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DIAG_EXIT_MASK_VALID))
            {
                Pmic_setBitField_b(&regData, DIAG_EXIT_MASK_SHIFT, deviceCfg->diagExitMask);
            }

            // Modify DIAG_EXIT bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DIAG_EXIT_VALID))
            {
                Pmic_setBitField_b(&regData, DIAG_EXIT_SHIFT, deviceCfg->diagExit);
            }

            // Write new register value back to PMIC
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_CHECK_CTRL, regData);
        }
        Pmic_criticalSectionStop(pmicHandle);
    }

    return status;
}

static int32_t CORE_writeSafetyErrCfg1(const Pmic_CoreHandle_t *pmicHandle, const Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t safetyErrCfg1ValidParams = (PMIC_SAFE_TIMEOUT_DURATION_VALID | PMIC_SAFE_LOCK_THR_VALID);

    // Read SAFETY_ERR_CFG_1 register
    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(deviceCfg->validParams, safetyErrCfg1ValidParams))
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_1, &regData);
    }

    // Modify SAFE_TO[2:0] bit field
    if (Pmic_validParamStatusCheck(deviceCfg->validParams, PMIC_SAFE_TIMEOUT_DURATION_VALID, status))
    {
        if (deviceCfg->safeTimeoutDuration > PMIC_SAFE_TO_DURATION_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, SAFE_TO_SHIFT, SAFE_TO_MASK, deviceCfg->safeTimeoutDuration);
        }
    }

    // Modify SAFE_LOCK_THR[3:0] bit field
    if (Pmic_validParamStatusCheck(deviceCfg->validParams, PMIC_SAFE_LOCK_THR_VALID, status))
    {
        if (deviceCfg->safeLockThr > PMIC_SAFE_LOCK_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, SAFE_LOCK_THR_SHIFT, SAFE_LOCK_THR_MASK, deviceCfg->safeLockThr);
        }
    }

    // Write new register value back to PMIC
    if (Pmic_validParamStatusCheck(deviceCfg->validParams, safetyErrCfg1ValidParams, status))
    {
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_CFG_1, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_setDevErrCntAndPwrDwnThr(const Pmic_CoreHandle_t *pmicHandle, const Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DEV_ERR_CNT_VALID))
    {
        // Read SAFETY_ERR_STAT_1 register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_STAT_1, &regData);

        // Modify DEV_ERR_CNT[3:0] bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (deviceCfg->devErrCnt > PMIC_DEV_ERR_CNT_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, DEV_ERR_CNT_SHIFT, DEV_ERR_CNT_MASK, deviceCfg->devErrCnt);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_ERR_STAT_1, regData);
        }
    }

    if (Pmic_validParamStatusCheck(deviceCfg->validParams, PMIC_PWR_DWN_THR_VALID, status))
    {
        // Read SAFETY_PWD_THR_CFG register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_PWD_THR_CFG, &regData);

        // Extract PWD_THR[3:0] bit field
        if (status == PMIC_ST_SUCCESS)
        {
            if (deviceCfg->pwrDwnThr > PMIC_PWR_DWN_THR_MAX)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, PWD_THR_SHIFT, PWD_THR_MASK, deviceCfg->pwrDwnThr);
            }
        }

        // Write new register value back to PMIC
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_PWD_THR_CFG, regData);
        }
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_setDeviceCfg(const Pmic_CoreHandle_t *pmicHandle, const Pmic_DeviceCfg_t *deviceCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (deviceCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Set AUTO_BIST_DIS and SAFE_LOCK_TO_DIS configurations
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_setAutoBistDisAndSafeLockToDis(pmicHandle, deviceCfg);
    }

    // Set SAFETY_CHECK_CTRL register configurations for ENABLE_DRV, DIAG_EXIT, and DIAG_EXIT_MASK
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_writeSafetyCheckCtrl(pmicHandle, deviceCfg);
    }

    // Set SAFETY_ERR_CFG_1 register configurations for SAFE_TO[2:0] and SAFE_LOCK_THR[3:0]
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_writeSafetyErrCfg1(pmicHandle, deviceCfg);
    }

    // Set device error count and power down threshold
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_setDevErrCntAndPwrDwnThr(pmicHandle, deviceCfg);
    }

    return status;
}

static int32_t CORE_getAutoBistDisAndSafeLockToDis(const Pmic_CoreHandle_t *pmicHandle, Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DISABLE_AUTO_BIST_VALID))
    {
        // Read SAFETY_BIST_CTRL register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_BIST_CTRL, &regData);

        // Extract AUTO_BIST_DIS bit field
        if (status == PMIC_ST_SUCCESS)
        {
            deviceCfg->disableAutoBIST = Pmic_getBitField_b(regData, AUTO_BIST_DIS_SHIFT);
        }
    }

    if (Pmic_validParamStatusCheck(deviceCfg->validParams, PMIC_DISABLE_SAFE_LOCK_TIMEOUT_VALID, status))
    {
        // Read SAFETY_FUNC_CFG register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);

        // Extract SAFE_LOCK_TO_DIS bit field
        if (status == PMIC_ST_SUCCESS)
        {
            deviceCfg->disableSafeLockTimeout = Pmic_getBitField_b(regData, SAFE_LOCK_TO_DIS_SHIFT);
        }
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_readSafetyCheckCtrl(const Pmic_CoreHandle_t *pmicHandle, Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    const uint32_t safetyCheckCtrlValidParams = (PMIC_ENABLE_DRV_VALID | PMIC_DIAG_EXIT_VALID | PMIC_DIAG_EXIT_MASK_VALID);

    if (Pmic_validParamCheck(deviceCfg->validParams, safetyCheckCtrlValidParams))
    {
        // Read SAFETY_CHECK_CTRL register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract ENABLE_DRV bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_ENABLE_DRV_VALID))
            {
                deviceCfg->enableDrv = Pmic_getBitField_b(regData, ENABLE_DRV_SHIFT);
            }

            // Extract DIAG_EXIT_MASK bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DIAG_EXIT_MASK_VALID))
            {
                deviceCfg->diagExitMask = Pmic_getBitField_b(regData, DIAG_EXIT_MASK_SHIFT);
            }

            // Extract DIAG_EXIT bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DIAG_EXIT_VALID))
            {
                deviceCfg->diagExit = Pmic_getBitField_b(regData, DIAG_EXIT_SHIFT);
            }
        }
    }

    return status;
}

static int32_t CORE_readSafetyErrCfg1(const Pmic_CoreHandle_t *pmicHandle, Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_SAFE_TIMEOUT_DURATION_VALID | PMIC_SAFE_LOCK_THR_VALID))
    {
        // Read SAFETY_ERR_CFG_1 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_1, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Extract SAFE_TO[2:0] bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_SAFE_TIMEOUT_DURATION_VALID))
            {
                deviceCfg->safeTimeoutDuration = Pmic_getBitField(regData, SAFE_TO_SHIFT, SAFE_TO_MASK);
            }

            // Extract SAFE_LOCK_THR[3:0] bit field
            if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_SAFE_LOCK_THR_VALID))
            {
                deviceCfg->safeLockThr = Pmic_getBitField(regData, SAFE_LOCK_THR_SHIFT, SAFE_LOCK_THR_MASK);
            }
        }
    }

    return status;
}

static int32_t CORE_getDevErrCntAndPwrDwnThr(const Pmic_CoreHandle_t *pmicHandle, Pmic_DeviceCfg_t *deviceCfg)
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);
    if (Pmic_validParamCheck(deviceCfg->validParams, PMIC_DEV_ERR_CNT_VALID))
    {
        // Read SAFETY_ERR_STAT_1 register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_STAT_1, &regData);

        // Extract DEV_ERR_CNT[3:0] bit field
        if (status == PMIC_ST_SUCCESS)
        {
            deviceCfg->devErrCnt = Pmic_getBitField(regData, DEV_ERR_CNT_SHIFT, DEV_ERR_CNT_MASK);
        }
    }

    if (Pmic_validParamStatusCheck(deviceCfg->validParams, PMIC_PWR_DWN_THR_VALID, status))
    {
        // Read SAFETY_PWD_THR_CFG register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_PWD_THR_CFG, &regData);

        // Extract PWD_THR[3:0] bit field
        if (status == PMIC_ST_SUCCESS)
        {
            deviceCfg->pwrDwnThr = Pmic_getBitField(regData, PWD_THR_SHIFT, PWD_THR_MASK);
        }
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_getDeviceCfg(const Pmic_CoreHandle_t *pmicHandle, Pmic_DeviceCfg_t *deviceCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (deviceCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Get AUTO_BIST_DIS and SAFE_LOCK_TO_DIS
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getAutoBistDisAndSafeLockToDis(pmicHandle, deviceCfg);
    }

    // Get configurations from SAFETY_CHECK_CTRL register (ENABLE_DRV, DIAG_EXIT, DIAG_EXIT_MASK)
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_readSafetyCheckCtrl(pmicHandle, deviceCfg);
    }

    // Get configurations from SAFETY_ERR_CFG_1 register (SAFE_TO[2:0], SAFE_LOCK_THR[3:0])
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_readSafetyErrCfg1(pmicHandle, deviceCfg);
    }

    // Get device error count and power down threshold
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getDevErrCntAndPwrDwnThr(pmicHandle, deviceCfg);
    }

    return status;
}

static int32_t CORE_getCfgRegByte0(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read LPSAM_SAMPLE register
    Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_LPSAM_SAMPLE, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte0[3:0] to be LPSAM_SAMPLE[3:0]
        Pmic_setBitField(&cfgRegBytes[0U], 0U, 0xFU, Pmic_getBitField(regData, 0U, 0xFU));

        // Read LPSAM_IDLE register
        Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_LPSAM_IDLE, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte0[7:4] to be LPSAM_IDLE[3:0]
        Pmic_setBitField(&cfgRegBytes[0U], 4U, (0xFU << 4U), Pmic_getBitField(regData, 0U, 0xFU));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte1(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read LPSAM_PERIOD register
    status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_LPSAM_PERIOD, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte1[3:0] to be LPSAM_PERIOD[3:0]
        Pmic_setBitField(&cfgRegBytes[1U], 0U, 0xFU, Pmic_getBitField(regData, 0U, 0xFU));

        // Set byte1[4] to be LPSAM_PERIOD[7]
        Pmic_setBitField(&cfgRegBytes[1U], 4U, (1U << 4U), Pmic_getBitField(regData, 7U, (1U << 7U)));

        // Read SAM_CFG register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAM_CFG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte1[5] to be SAM_CFG[4]
        Pmic_setBitField(&cfgRegBytes[1U], 5U, (1U << 5U), Pmic_getBitField(regData, 4U, (1U << 4U)));

        // read DEV_CFG_2 register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_DEV_CFG_2, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte1[6] to be DEV_CFG_2[0]
        Pmic_setBitField(&cfgRegBytes[1U], 6U, (1U << 6U), Pmic_getBitField(regData, 0U, 1U));

        // Read SENS_CTRL register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SENS_CTRL, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte1[7] to be SENS_CTRL[2]
        Pmic_setBitField(&cfgRegBytes[1U], 7U, (1U << 7U), Pmic_getBitField(regData, 2U, (1U << 2U)));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte2(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read SENS_CTRL register
    status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SENS_CTRL, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte2[0] to be SENS_CTRL[3]
        Pmic_setBitField(&cfgRegBytes[2U], 0U, 1U, Pmic_getBitField(regData, 3U, (1U << 3U)));

        // Set byte2[1] to be SENS_CTRL[4]
        Pmic_setBitField(&cfgRegBytes[2U], 1U, (1U << 1U), Pmic_getBitField(regData, 4U, (1U << 4U)));

        // Set byte2[2] to be SENS_CTRL[6]
        Pmic_setBitField(&cfgRegBytes[2U], 2U, (1U << 2U), Pmic_getBitField(regData, 6U, (1U << 6U)));

        // Read SAFETY_CHECK_CTRL register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte2[3] to be SAFETY_CHECK_CTRL[2]
        Pmic_setBitField(&cfgRegBytes[2U], 3U, (1U << 3U), Pmic_getBitField(regData, 2U, (1U << 2U)));

        // Read DEV_CFG_3 register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_DEV_CFG_3, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte2[7:4] to be DEV_CFG_3[3:0]
        Pmic_setBitField(&cfgRegBytes[2U], 4U, (0xFU << 4U), Pmic_getBitField(regData, 0U, 0xFU));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte3(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read SAFETY_ERR_CFG_2 register
    status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_2, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte3[2:0] to be SAFETY_ERR_CFG_2[2:0]
        Pmic_setBitField(&cfgRegBytes[3U], 0U, 7U, Pmic_getBitField(regData, 0U, 7U));

        // Read DEV_CFG_1 register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_DEV_CFG_1, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte3[7:3] to be DEV_CFG_1[5:1]
        Pmic_setBitField(&cfgRegBytes[3U], 3U, (0x1FU << 3U), Pmic_getBitField(regData, 1U, (0x1FU << 1U)));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte4(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read DEV_CFG_1 register
    status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_DEV_CFG_1, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte4[1:0] to be DEV_CFG_1[7:6]
        Pmic_setBitField(&cfgRegBytes[4U], 0U, 3U, Pmic_getBitField(regData, 6U, (3U << 6U)));

        // Read DEV_CFG_2 register
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_DEV_CFG_2, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte4[2] to be DEV_CFG_2[1]
        Pmic_setBitField(&cfgRegBytes[4U], 2U, (1U << 2U), Pmic_getBitField(regData, 1U, (1U << 1U)));

        // Set byte4[3] to be 0
        Pmic_setBitField(&cfgRegBytes[4U], 3U, (1U << 3U), 0U);

        // Set byte4[7:4] to be DEV_CFG_2[7:4]
        Pmic_setBitField(&cfgRegBytes[4U], 4U, (0xFU << 4U), Pmic_getBitField(regData, 4U, (0xFU << 4U)));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte5(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_HMIN, &regData);

    // Set byte5[7:0] to be SAFETY_ERR_PWM_HMIN[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[5U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegByte6(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_HMAX, &regData);

    // Set byte6[7:0] to be SAFETY_ERR_PWM_HMAX[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[6U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegByte7(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_LMIN, &regData);

    // Set byte7[7:0] to be SAFETY_ERR_PWM_LMIN[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[7U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegByte8(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_PWM_LMAX, &regData);

    // Set byte8[7:0] to be SAFETY_ERR_PWM_LMAX[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[8U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegByte9(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read WD_WIN1_CFG register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_WIN1_CFG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte9[6:0] to be WD_WIN1_CFG[6:0]
        Pmic_setBitField(&cfgRegBytes[9U], 0U, 0x7FU, Pmic_getBitField(regData, 0U, 0x7FU));

        // Read WD_WIN2_CFG register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_WIN2_CFG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte9[7] to be WD_WIN2_CFG[0]
        Pmic_setBitField(&cfgRegBytes[9U], 7U, (1U << 7U), Pmic_getBitField(regData, 0U, 1U));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte10(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read WD_WIN2_CFG register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_WIN2_CFG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte10[3:0] to be WD_WIN2_CFG[4:1]
        Pmic_setBitField(&cfgRegBytes[10U], 0U, 0xFU, Pmic_getBitField(regData, 1U, (0xFU << 1U)));

        // Read WD_QUESTION_FDBCK register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_QUESTION_FDBCK, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte10[7:4] to be WD_QUESTION_FDBCK[3:0]
        Pmic_setBitField(&cfgRegBytes[10U], 4U, (0xFU << 4U), Pmic_getBitField(regData, 0U, 0xFU));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte11(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read WD_QUESTION_FDBCK register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_WD_QUESTION_FDBCK, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte11[3:0] to be WD_QUESTION_FDBCK[7:4]
        Pmic_setBitField(&cfgRegBytes[11U], 0U, 0xFU, Pmic_getBitField(regData, 4U, (0xFU << 4U)));

        // Read SAFETY_ERR_CFG_1 register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_1, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte11[7:4] to be SAFETY_ERR_CFG_1[3:0]
        Pmic_setBitField(&cfgRegBytes[11U], 4U, (0xFU << 4U), Pmic_getBitField(regData, 0U, 0xFU));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte12(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(pmicHandle);

    // Read SAFETY_ERR_CFG_1 register
    status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_1, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte12[3:0] to be SAFETY_ERR_CFG_1[7:4]
        Pmic_setBitField(&cfgRegBytes[12U], 0U, 0xFU, Pmic_getBitField(regData, 4U, (0xFU << 4U)));

        // Read SAFETY_PWD_THR_CFG register
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_PWD_THR_CFG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Set byte12[7:4] to be SAFETY_PWD_THR_CFG[3:0]
        Pmic_setBitField(&cfgRegBytes[12U], 4U, (0xFU << 4U), Pmic_getBitField(regData, 0U, 0xFU));
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

static int32_t CORE_getCfgRegByte13(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_DEV_ID, &regData);

    // Set byte13[7:0] to be DEV_ID[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[13U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegByte14(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_DEV_REV, &regData);

    // Set byte14[7:0] to be DEV_REV[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[14U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegByte15(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_FUNC_CFG, &regData);

    // Set byte15[7:0] to be SAFETY_FUNC_CFG[7:0]
    if (status == PMIC_ST_SUCCESS)
    {
        cfgRegBytes[15U] = regData;
    }

    return status;
}

static int32_t CORE_getCfgRegBytes8to15(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    int32_t status = CORE_getCfgRegByte8(pmicHandle, cfgRegBytes);

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte9(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte10(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte11(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte12(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte13(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte14(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte15(pmicHandle, cfgRegBytes);
    }

    return status;
}

static int32_t CORE_getCfgRegBytes0to7(const Pmic_CoreHandle_t *pmicHandle, uint8_t cfgRegBytes[])
{
    int32_t status = CORE_getCfgRegByte0(pmicHandle, cfgRegBytes);

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte1(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte2(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte3(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte4(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte5(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte6(pmicHandle, cfgRegBytes);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegByte7(pmicHandle, cfgRegBytes);
    }

    return status;
}

int32_t Pmic_writeCfgCrc(const Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t cfgRegBytes[NUM_DATA_BYTES_FOR_CFG_REG_CRC] = {0U};
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    // Get configuration register bytes 0 to 7
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegBytes0to7(pmicHandle, cfgRegBytes);
    }

    // Get configuration register bytes 8 to 15
    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_getCfgRegBytes8to15(pmicHandle, cfgRegBytes);
    }

    // Calculate and send configuration register CRC
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(pmicHandle,
                                  PMIC_CMD_WR_SAFETY_CFG_CRC,
                                  Pmic_getCRC8Val(cfgRegBytes, NUM_DATA_BYTES_FOR_CFG_REG_CRC));
    }

    return status;
}

int32_t Pmic_setCfgCrc(const Pmic_CoreHandle_t *pmicHandle, bool CfgCrcEnable)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    Pmic_criticalSectionStart(pmicHandle);

    // Read SAFETY_CHECK_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_CHECK_CTRL, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify CFG_CRC_EN bit field
        Pmic_setBitField_b(&regData, CFG_CRC_EN_SHIFT, CfgCrcEnable);

        // Write new register value back to PMIC
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_CHECK_CTRL, regData);
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_enableCfgCrc(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setCfgCrc(pmicHandle, (bool)true);
}

int32_t Pmic_disableCfgCrc(const Pmic_CoreHandle_t *pmicHandle)
{
    return Pmic_setCfgCrc(pmicHandle, (bool)false);
}

int32_t Pmic_getCfgCrcErrStat(const Pmic_CoreHandle_t *pmicHandle, bool *CfgCrcErr)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (CfgCrcErr == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read SAFETY_STAT_3 register
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_3, &regData);
    }

    // Extract CFG_CRC_ERR flag
    if (status == PMIC_ST_SUCCESS)
    {
        *CfgCrcErr = Pmic_getBitField_b(regData, CFG_CRC_ERR_SHIFT);
    }

    return status;
}

int32_t Pmic_getRegLock(const Pmic_CoreHandle_t *pmicHandle, bool *locked)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (locked == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read SAFETY_ERR_CFG_1 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_ERR_CFG_1, &regData);
    }

    // Extract CFG_LOCK bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *locked = Pmic_getBitField_b(regData, CFG_LOCK_SHIFT);
    }

    return status;
}

int32_t Pmic_sendRstReq(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(pmicHandle, PMIC_CMD_MCU_RST_RQ, PMIC_CMD_MCU_RST_RQ_DATA);
    }

    return status;
}

int32_t Pmic_sendSafeExitReq(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(pmicHandle, PMIC_CMD_SAFE_EXIT, PMIC_CMD_SAFE_EXIT_DATA);
    }

    return status;
}

int32_t Pmic_getDevState(const Pmic_CoreHandle_t *pmicHandle, uint8_t *devState)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (devState == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read SAFETY_STAT_5 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_5, &regData);
    }

    // Extract STATE[2:0] bit field
    if (status == PMIC_ST_SUCCESS)
    {
        *devState = Pmic_getBitField(regData, STATE_SHIFT, STATE_MASK);
    }

    return status;
}

int32_t Pmic_getBistStat(const Pmic_CoreHandle_t *pmicHandle, Pmic_BistStat_t *bistStat)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    if ((status == PMIC_ST_SUCCESS) && (bistStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    Pmic_criticalSectionStart(pmicHandle);

    // Read SAFETY_STAT_3 register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_STAT_3, &regData);
    }

    // Extract LBIST_ERR, ABIST_ERR, LBIST_DONE, and ABIST_DONE
    if (status == PMIC_ST_SUCCESS)
    {
        bistStat->lbistErr = Pmic_getBitField_b(regData, LBIST_ERR_SHIFT);
        bistStat->abistErr = Pmic_getBitField_b(regData, ABIST_ERR_SHIFT);
        bistStat->lbistDone = Pmic_getBitField_b(regData, LBIST_DONE_SHIFT);
        bistStat->abistDone = Pmic_getBitField_b(regData, ABIST_DONE_SHIFT);
    }

    // Read SAM_STAT register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAM_STAT, &regData);
    }

    // Extract SAM_BIST_FAIL
    if (status == PMIC_ST_SUCCESS)
    {
        bistStat->samBistErr = Pmic_getBitField_b(regData, SAM_BIST_FAIL_SHIFT);
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_startLBIST(const Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    Pmic_criticalSectionStart(pmicHandle);

    // Read SAFETY_BIST_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_BIST_CTRL, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify LBIST_EN bit field to be 1
        Pmic_setBitField_b(&regData, LBIST_EN_SHIFT, (bool)true);

        // Write new register value back to PMIC
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_BIST_CTRL, regData);
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_startABIST(const Pmic_CoreHandle_t *pmicHandle)
{
    uint8_t regData = 0U;
    int32_t status = Pmic_checkPmicCoreHandle(pmicHandle);

    Pmic_criticalSectionStart(pmicHandle);

    // Read SAFETY_BIST_CTRL register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_CMD_RD_SAFETY_BIST_CTRL, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify ABIST_EN bit field to be 1
        Pmic_setBitField_b(&regData, ABIST_EN_SHIFT, (bool)true);

        // Write new register value back to PMIC
        status = Pmic_ioTxByte(pmicHandle, PMIC_CMD_WR_SAFETY_BIST_CTRL, regData);
    }

    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

void Pmic_getCrc8Table(const uint8_t **crc8Table)
{
    *crc8Table = CRC_8_TABLE;
}

uint8_t Pmic_getCRC8Val(const uint8_t *data, uint8_t length)
{
    const uint8_t *crc8Table;
    uint8_t crc = 0xFFU;

    Pmic_getCrc8Table(&crc8Table);

    for (uint8_t i = 0U; i < length; i++)
    {
        crc = crc8Table[data[i] ^ crc];
    }

    return crc;
}
