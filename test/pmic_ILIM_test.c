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
 *  @file  pmic_ilim_test.c
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC
 *          ILIM.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_ilim_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                             */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_ILIM = NULL;

/* INITIALIZATION FUNCTIONS - START */

int32_t test_pmic_ILIM_config_init(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_BB_TPS65386X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SPI;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle_ILIM, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__, status);
    }
    return status;
}

int32_t test_pmic_ILIM_config_deinit(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_ILIM);
    free(pPmicCoreHandle_ILIM);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__,  status);
    }
    return status;
}

void test_pmic_set_ilimConfig()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_ilimCfgReg_t  *pPmicILIMConfig = NULL;
    pPmicILIMConfig->pldo2ILIMCfg = PMIC_ILIM_CFG_DATA1;
    pPmicILIMConfig->pldo1ILIMCfg = PMIC_ILIM_CFG_DATA1;
    pPmicILIMConfig->ldo4ILIMCfg  = PMIC_ILIM_CFG_DATA1;
    pPmicILIMConfig->ldo3ILIMCfg  = PMIC_ILIM_CFG_DATA1;
    pPmicILIMConfig->ldo2ILIMCfg  = PMIC_ILIM_CFG_DATA1;
    pPmicILIMConfig->ldo1ILIMCfg  = PMIC_ILIM_CFG_DATA1;
    pmicStatus = Pmic_SetILIMConfig(pPmicCoreHandle_ILIM, pPmicILIMConfig);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

void test_pmic_get_ilimConfig()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_ilimCfgReg_t  *pPmicILIMConfig = NULL;
    pmicStatus = Pmic_GetILIMConfig(pPmicCoreHandle_ILIM, pPmicILIMConfig);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

void test_pmic_set_ilimDglConfig()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_ilimDglCfgReg_t  *pPmicILIMDglConfig = NULL;
    pPmicILIMDglConfig->pldo2ILIMdglCfg = PMIC_ILIM_DGL_CFG_DATA1;
    pPmicILIMDglConfig->pldo1ILIMdglCfg = PMIC_ILIM_DGL_CFG_DATA1;
    pPmicILIMDglConfig->ldo4ILIMdglCfg  = PMIC_ILIM_DGL_CFG_DATA1;
    pPmicILIMDglConfig->ldo3ILIMdglCfg  = PMIC_ILIM_DGL_CFG_DATA1;
    pPmicILIMDglConfig->ldo2ILIMdglCfg  = PMIC_ILIM_DGL_CFG_DATA1;
    pPmicILIMDglConfig->ldo1ILIMdglCfg  = PMIC_ILIM_DGL_CFG_DATA1;
    pmicStatus = Pmic_SetILIMDglConfig(pPmicCoreHandle_ILIM, pPmicILIMDglConfig);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

void test_pmic_get_ilimDglConfig()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_ilimDglCfgReg_t  *pPmicILIMDglConfig = NULL;
    pmicStatus = Pmic_GetILIMDglConfig(pPmicCoreHandle_ILIM, pPmicILIMDglConfig);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

void test_pmic_clear_ilimErrStat()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_ilimStatReg_t  *pPmicILIMStat = NULL;
    pPmicILIMStat->bbavgILIMErr = PMIC_ILIM_ERR_CLEAR_DATA;
    pPmicILIMStat->pldo2ILIMErr = PMIC_ILIM_ERR_CLEAR_DATA;
    pPmicILIMStat->pldo1ILIMErr = PMIC_ILIM_ERR_CLEAR_DATA;
    pPmicILIMStat->ldo4ILIMErr  = PMIC_ILIM_ERR_CLEAR_DATA;
    pPmicILIMStat->ldo3ILIMErr  = PMIC_ILIM_ERR_CLEAR_DATA;
    pPmicILIMStat->ldo2ILIMErr  = PMIC_ILIM_ERR_CLEAR_DATA;
    pPmicILIMStat->ldo1ILIMErr  = PMIC_ILIM_ERR_CLEAR_DATA;
    pmicStatus = Pmic_ClearILIMErrStat(pPmicCoreHandle_ILIM, pPmicILIMStat);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

void test_pmic_get_ilimErrStat()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_ilimStatReg_t  *pPmicILIMStat = NULL;
    pmicStatus = Pmic_GetILIMErrStat(pPmicCoreHandle_ILIM, pPmicILIMStat);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
}

void test_pmic_ilim_cfg_stat()
{
    test_pmic_set_ilimConfig();
    test_pmic_get_ilimConfig();

    test_pmic_set_ilimDglConfig();
    test_pmic_get_ilimDglConfig();

    test_pmic_get_ilimErrStat();
    test_pmic_clear_ilimErrStat();
}

void *test_pmic_ILIM(void *args)
{
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_ILIM_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_ILIM, 0);    //LOCK

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_ILIM, 1);    //UNLOCK

    /* Lock counter register initially */
    DebugP_log("[INIT] TC and RC Lock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_ILIM, 0);    //LOCK

    /* Unlock counter register initially */
    DebugP_log("[INIT] TC and RC Unlock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_ILIM, 1);    //UNLOCK

    /* Low IQ Timer Test Cases */
    DebugP_log("[TEST] :\r\n");
    test_pmic_ilim_cfg_stat();

    /* De-initialization */
    test_pmic_ILIM_config_deinit();
    DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
