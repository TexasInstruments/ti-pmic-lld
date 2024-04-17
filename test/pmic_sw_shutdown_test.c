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
 *  @file  pmic_sw_shutdown_test.c
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC
 *          SW SHUTDOWN.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "pmic_sw_shutdown_test.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* ========================================================================== */
/*                          Function Definitions                             */
/* ========================================================================== */

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle_sw_shutdown = NULL;

/* INITIALIZATION FUNCTIONS - START */

int32_t test_pmic_sw_shutdown_config_init(void)
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

    status = test_pmic_appInit(&pPmicCoreHandle_sw_shutdown, &pmicConfigData);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__, status);
    }
    return status;
}

int32_t test_pmic_sw_shutdown_config_deinit(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    status = Pmic_deinit(pPmicCoreHandle_sw_shutdown);
    free(pPmicCoreHandle_sw_shutdown);
    SemaphoreP_destruct(&gpmicCoreObj);
    if (PMIC_ST_SUCCESS != status)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\r\n",
                 __func__, __LINE__,  status);
    }
    return status;
}

void test_pmic_get_offstatestat1()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_offstatestat1Reg_t  *pPmicOffStateStat1Config = NULL;
    pmicStatus = Pmic_GetOffStateStat1Config(pPmicCoreHandle_sw_shutdown, pPmicOffStateStat1Config);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        if((pPmicOffStateStat1Config -> initTMO) == PMIC_INIT_AN_TMO_DATA1) {
            DebugP_log("Device timeout during INIT state flag : The device has not been in the unpowered state since this bit was cleared\r\n\n");
        }
        else    {
            DebugP_log("Device timeout during INIT state flag : ERROR!!! - The INIT_AN_TMO timeout indicates a powerup issue where the digital did not start before the timeout\r\n\n");
        }
        if((pPmicOffStateStat1Config -> internalOV) == PMIC_INTERNAL_OV_DATA1) {
            DebugP_log("Device Power-On error flag : The device has not been in the unpowered state since this bit was cleared\r\n\n");
        }
        else    {
            DebugP_log("Device Power-On error flag : ERROR!!! - An internal OV was detected in V1P8, VREG or VSAFETY\r\n\n");
        }
        if((pPmicOffStateStat1Config -> ClkErr) == PMIC_CLK_ERR_DATA1) {
            DebugP_log("No clock failure(No system clock or low power clock or low power clock error detected)\r\n\n");
        }
        else    {
            DebugP_log("ERROR!!! - Low power or system clock error occurred\r\n\n");
        }
        if((pPmicOffStateStat1Config -> firstPWROn) == PMIC_FIRST_PWR_ON_DATA1) {
            DebugP_log("The device has not been in the unpowered state since this bit was cleared\r\n\n");
        }
        if((pPmicOffStateStat1Config -> OffprotEvt) == PMIC_OFF_PROT_EVT_DATA1) {
            DebugP_log("No OFF state protection event error\r\n\n");
        }
        else    {
            DebugP_log("ERROR!!! - OFF state protection event error\r\n\n");
        }
        if((pPmicOffStateStat1Config -> OffinitEvtErr) == PMIC_OFF_INT_EVT_ERR_DATA1) {
            DebugP_log("No OFF state interrupt event error\r\n\n");
        }
        else    {
            DebugP_log("ERROR!!! - OFF state interrupt event error\r\n\n");
        }
        if((pPmicOffStateStat1Config -> normalOff) == PMIC_NORMAL_OFF_DATA1) {
            DebugP_log("No clock failure\r\n\n");
        }
        else    {
            DebugP_log("ERROR!!! - Too slow low-power clock\r\n\n");
        }
    }
}

void test_pmic_get_offstatestat2()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_offstatestat2Reg_t  *pPmicOffStateStat2Config = NULL;
    pmicStatus = Pmic_GetOffStateStat2Config(pPmicCoreHandle_sw_shutdown, pPmicOffStateStat2Config);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        if((pPmicOffStateStat2Config -> BBIlimErr) == PMIC_BB_PK_ILIM_ERR_DATA1) {
            DebugP_log("Buck-boost average current-limit error flag : No current-limit\r\n\n");
        }
        else    {
            DebugP_log("Buck-boost average current-limit error flag : ERROR!!! - Current-limit\r\n\n");
        }
        if((pPmicOffStateStat2Config -> BBTMO) == PMIC_BB_BST_TMO_DATA1) {
            DebugP_log("OFF state caused by boost mode time-out event from Buck-Boost flag : No buck-boost boost mode timeout\r\n\n");
        }
        else    {
            DebugP_log("OFF state caused by boost mode time-out event from Buck-Boost flag : ERROR!!! - Buck-boost operated in boost timeout event\r\n\n");
        }
        if((pPmicOffStateStat2Config -> BBOVPErr) == PMIC_BB_OVP_ERR_DATA1) {
            DebugP_log("Buck-Boost overvoltage protection flag : No overvoltage\r\n\n");
        }
        else    {
            DebugP_log("Buck-Boost overvoltage protection flag : ERROR!!! - Overvoltage\r\n\n");
        }
        if((pPmicOffStateStat2Config -> VbatOVPErr) == PMIC_VBAT_OVP_ERR_DATA1) {
            DebugP_log("VBAT overvoltage protection error flag : No overvoltage\r\n\n");
        }
        else    {
            DebugP_log("VBAT overvoltage protection error flag : ERROR!!! - Overvoltage\r\n\n");
        }
        if((pPmicOffStateStat2Config -> BGXMErr) == PMIC_BGXM_ERR_DATA1) {
            DebugP_log("Bandgap cross monitor (BG XMON) protection error flag : No BGXM error\r\n\n");
        }
        else    {
            DebugP_log("Bandgap cross monitor (BG XMON) protection error flag : ERROR!!! - BGXM error\r\n\n");
        }
        if((pPmicOffStateStat2Config -> RstMcuTMO) == PMIC_RST_MCU_TMO_DATA1) {
            DebugP_log("OFF state caused due to RESET-MCU state time-out event flag : No RESET-MCU Timeout\r\n\n");
        }
        else    {
            DebugP_log("OFF state caused due to RESET-MCU state time-out event flag : ERROR!!! - RESET-MCU Timeout\r\n\n");
        }
        if((pPmicOffStateStat2Config -> SysClkErr) == PMIC_SYS_CLK_ERR_PROT_DATA1) {
            DebugP_log("System Clock Protection error flag : No clock failure\r\n\n");
        }
        else    {
            DebugP_log("System Clock Protection error flag : ERROR!!! - Clock failure\r\n\n");
        }
        if((pPmicOffStateStat2Config -> CRCErr) == PMIC_CRC_ERR_DATA1) {
            DebugP_log("OFF state caused by a CRC error flag : No CRC Error\r\n\n");
        }
        else    {
            DebugP_log("OFF state caused by a CRC error flag : ERROR!!! - CRC Error\r\n\n");
        }
    }
}

void test_pmic_clear_offstatestat()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    pmicStatus = Pmic_ClearOffStateStatRegs(pPmicCoreHandle_sw_shutdown);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        DebugP_log("OFF_STATE_STAT1 and OFF_STATE_STAT2 registers are cleared!!!\r\n\n");
    }
}


void test_pmic_get_thermalstat1()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_thermalstat1Reg_t  *pPmicThermalStat1Config = NULL;
    pmicStatus = Pmic_GetThermalStat1Config(pPmicCoreHandle_sw_shutdown, pPmicThermalStat1Config);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        if((pPmicThermalStat1Config -> ldo4TSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("LDO4 overtemperature thermal shutdown flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO4 overtemperature thermal shutdown flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo4TpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("LDO4 temperature prewarning flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO4 temperature prewarning flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo3TSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("LDO3 overtemperature thermal shutdown flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO3 overtemperature thermal shutdown flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo3TpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("LDO3 temperature prewarning flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO3 temperature prewarning flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo2TSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("LDO2 overtemperature thermal shutdown flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO2 overtemperature thermal shutdown flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo2TpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("LDO2 temperature prewarning flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO2 temperature prewarning flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo1TSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("LDO1 overtemperature thermal shutdown flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO1 overtemperature thermal shutdown flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat1Config -> ldo1TpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("LDO1 temperature prewarning flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("LDO1 temperature prewarning flag: ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
    }
}

void test_pmic_get_thermalstat2()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    Pmic_thermalstat2Reg_t  *pPmicThermalStat2Config = NULL;
    pmicStatus = Pmic_GetThermalStat2Config(pPmicCoreHandle_sw_shutdown, pPmicThermalStat2Config);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        if((pPmicThermalStat2Config -> BBTSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("Buck-Boost overtemperature thermal shutdown flag : No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("Buck-Boost overtemperature thermal shutdown flag : ERROR!!! - LDO4 overtemperature thermal shutdown flag: Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat2Config -> BBTpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("Buck-Boost temperature prewarning flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("Buck-Boost temperature prewarning flag : ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat2Config -> pldo2TSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("PLDO2 overtemperature thermal shutdown flag : No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("PLDO2 overtemperature thermal shutdown flag : ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat2Config -> pldo2TpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("PLDO2 temperature prewarning flag : No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("PLDO2 temperature prewarning flag : ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat2Config -> pldo1TSDErr) == PMIC_THERMAL_STAT_TSD_ERR_DATA1) {
            DebugP_log("PLDO1 overtemperature thermal shutdown flag : No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("PLDO1 overtemperature thermal shutdown flag : ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
        if((pPmicThermalStat2Config -> pldo1TpreErr) == PMIC_THERMAL_STAT_TPRE_ER_DATA1) {
            DebugP_log("PLDO1 temperature prewarning flag: No overtemperature\r\n\n");
        }
        else    {
            DebugP_log("PLDO1 temperature prewarning flag : ERROR!!! - Overtemperature Thermal Shutdown\r\n\n");
        }
    }
}

void test_pmic_clear_thermalstat()
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    pmicStatus = Pmic_ClearThermalStat1flags(pPmicCoreHandle_sw_shutdown);
    if (PMIC_ST_SUCCESS != pmicStatus)
    {
        DebugP_log("%s(): %d: FAILED with status: %d\n",
                 __func__, __LINE__,  pmicStatus);
    }
    else
    {
        DebugP_log("All flags Thermal Stat are cleared!!!\r\n\n");
    }
}

void test_pmic_swshutdown()
{
    test_pmic_get_offstatestat1();
    test_pmic_get_offstatestat2();
    test_pmic_clear_offstatestat();

    test_pmic_get_thermalstat1();
    test_pmic_get_thermalstat2();
    test_pmic_clear_thermalstat();
}

void *test_pmic_SW_SHUTDOWN(void *args)
{
    Drivers_open();
    Board_driversOpen();

    mcspi_mux_pmic();

    /* Initialization */
    DebugP_log("Initializing...\r\n");
    test_pmic_sw_shutdown_config_init();
    DebugP_log("Initialization Completed!\r\n\n");

    /* Lock config register initially */
    DebugP_log("[INIT] Configuration Register Lock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_sw_shutdown, 0);    //LOCK

    /*API to unlock configuration register */
    DebugP_log("[INIT] Configuration Register Unlock Sequence:\r\n");
    test_pmic_LockUnlock(pPmicCoreHandle_sw_shutdown, 1);    //UNLOCK

    /* Lock counter register initially */
    DebugP_log("[INIT] TC and RC Lock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_sw_shutdown, 0);    //LOCK

    /* Unlock counter register initially */
    DebugP_log("[INIT] TC and RC Unlock Sequence:\r\n");
    test_pmic_CNT_LockUnlock(pPmicCoreHandle_sw_shutdown, 1);    //UNLOCK

    /* Low IQ Timer Test Cases */
    DebugP_log("[TEST] SW SHUTDOWN:\r\n");
    test_pmic_swshutdown();

    /* De-initialization */
    test_pmic_sw_shutdown_config_deinit();
    DebugP_log("\n[DE-INIT] De-initialization Completed\r\n");

    Board_driversClose();
    Drivers_close();

    return NULL;
}
