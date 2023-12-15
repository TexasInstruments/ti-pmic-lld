/**
 * \file power_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Unity testing application that tests the PMIC driver Power APIs
 * \version 1.0
 * \date 2023-11-13
 *
 * \copyright Copyright (c) 2023
 */

/* Standard includes */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

/* Tiva test app library */
#include "pmic_drv/burton_testAppLib/tiva/tiva_testLib.h"

/* Test specific include */
#include "pmic_drv/burton_testAppLib/tiva/burton_tests/power_test.h"

/* Unity testing library */
#include "unity/unity.h"

/* PMIC driver */
#include "pmic_drv/pmic.h"

timerHandle_t     timerHandle;
Pmic_CoreHandle_t pmicCoreHandle;

static void disablePmicPowerResources(void);

int main(void)
{
    /*** Variable declaration/initialization ***/
    // clang-format off
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT | PMIC_CFG_COMM_MODE_VALID_SHIFT    | PMIC_CFG_SLAVEADDR_VALID_SHIFT |
            PMIC_CFG_QASLAVEADDR_VALID_SHIFT  | PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT
            | PMIC_CFG_COMM_IO_RD_VALID_SHIFT   | PMIC_CFG_COMM_IO_WR_VALID_SHIFT   |
            PMIC_CFG_I2C1_SPEED_VALID_SHIFT),
        .instType           = PMIC_MAIN_INST,
        .pmicDeviceType     = PMIC_DEV_BURTON_TPS6522X,
        .commMode           = PMIC_INTF_SINGLE_I2C,
        .slaveAddr          = BURTON_I2C_USER_PAGE_ADDRESS,
        .qaSlaveAddr        = BURTON_I2C_WDG_PAGE_ADDRESS,
        .nvmSlaveAddr       = BURTON_I2C_NVM_PAGE_ADDRESS,
        .i2c1Speed          = PMIC_I2C_STANDARD_MODE,
        .pCommHandle        = &I2C1Handle,
        .pQACommHandle      = &I2C1Handle,
        .pFnPmicCommIoRead  = &pmicI2CRead,
        .pFnPmicCommIoWrite = &pmicI2CWrite};
    // clang-format on

    /*** System clock setup ***/
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 400 / 2 / 4 = 50 MHz clock rate

    /*** Virtual communication port setup ***/
    initializeVCPHandle(&vcpHandle);
    initializeVCP(&vcpHandle);

    /*** I2C setup ***/
    initializeI2C1Handle(&I2C1Handle);
    initializeI2C(&I2C1Handle);

    /*** PMIC setup ***/
    initializePmicCoreHandle(&pmicCoreHandle);
    Pmic_init(&pmicConfigData, &pmicCoreHandle);

    /*** Timer setup ***/
    initializeTimerHandle(&timerHandle);
    initializeTimer(&timerHandle);

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Put PMIC power resources into a known state for testing ***/
    disablePmicPowerResources();
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);

    /*** Ensure changes are propagated by waiting a certain period of time ***/
    delayTimeInMs(&timerHandle, 1000);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC Power tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_TEST(test_power_getConfiguration_pmicHandle_null);
    RUN_TEST(test_power_getConfiguration_pwrRsrcCfg_null);
    RUN_TEST(test_power_getConfiguration_pwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_getConfiguration_buckPwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_getConfiguration_ldoPwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_getConfiguration_vccaVmonPwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_getConfiguration_validParameters);
    RUN_TEST(test_power_setConfiguration_pmicHandle_null);
    RUN_TEST(test_power_setConfiguration_pwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_setConfiguration_buckPwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_setConfiguration_ldoPwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_setConfiguration_vccaVmonPwrRsrcCfg_noValidParam);
    RUN_TEST(test_power_setConfiguration_buckPldnEnableDisable);
    RUN_TEST(test_power_setConfiguration_buckVmonEnableDisable);
    RUN_TEST(test_power_setConfiguration_buckFPWM);
    RUN_TEST(test_power_setConfiguration_buckEn);
    RUN_TEST(test_power_setConfiguration_buckSlewRate);
    RUN_TEST(test_power_setConfiguration_buckVout_voltageBelowRange);
    RUN_TEST(test_power_setConfiguration_buckVout_voltageAboveRange);
    RUN_TEST(test_power_setConfiguration_buck1_buckVout);
    RUN_TEST(test_power_setConfiguration_buck2_3_4_buckVout);
    RUN_TEST(test_power_setConfiguration_buckVmonThr);
    RUN_TEST(test_power_setConfiguration_buckRailGrpSel);
    RUN_TEST(test_power_setConfiguration_ldoDischargeEnableDisable);
    RUN_TEST(test_power_setConfiguration_ldoVmonEnableDisable);
    RUN_TEST(test_power_setConfiguration_ldoEnableDisable);
    RUN_TEST(test_power_setConfiguration_ldoBypassConfig);
    RUN_TEST(test_power_setConfiguration_ldo1Vout_voltageBelowRange);
    RUN_TEST(test_power_setConfiguration_ldo1Vout_voltageAboveRange);
    RUN_TEST(test_power_setConfiguration_ldo2_3_Vout_voltageBelowRange);
    RUN_TEST(test_power_setConfiguration_ldo2_3_Vout_voltageAboveRange);
    RUN_TEST(test_power_setConfiguration_ldo1_ldoVout);
    RUN_TEST(test_power_setConfiguration_ldo2_3_ldoVout);
    RUN_TEST(test_power_setConfiguration_ldoVmonThr);
    RUN_TEST(test_power_setConfiguration_ldoRailGrpSel);
    RUN_TEST(test_power_setConfiguration_vmonDeglitch);
    RUN_TEST(test_power_setConfiguration_VMON1_2_VCCA_VMON_EnableDisable);
    RUN_TEST(test_power_setConfiguration_vccaPgLevel);
    RUN_TEST(test_power_setConfiguration_vccaVmonThr);
    RUN_TEST(test_power_setConfiguration_vccaRailGrpSel);
    RUN_TEST(test_power_setConfiguration_vmon1PgSet_voltageOutOfRange);
    RUN_TEST(test_power_setConfiguration_vmon2PgSet_voltageOutOfRange);
    RUN_TEST(test_power_setConfiguration_vmon1PgSet);
    RUN_TEST(test_power_setConfiguration_vmon2PgSet);
    RUN_TEST(test_power_setConfiguration_vmon1_2_RailGrpSel);
    RUN_TEST(test_power_setConfiguration_vmon1_2_Thr);
    RUN_TEST(test_power_getPwrRsrcStat_nullParam);
    RUN_TEST(test_power_getPwrRsrcStat_allPwrRsrc);
    RUN_TEST(test_power_getPwrRsrcStat_vmon1_2_UVOVStatDetection);
    RUN_TEST(test_power_getPwrThermalStat_nullParam);
    RUN_TEST(test_power_getPwrThermalStat_noValidParams);
    RUN_TEST(test_power_getPwrThermalStat_getAllStatus);
    RUN_TEST(test_power_getThermalCfg_nullParam);
    RUN_TEST(test_power_getThermalCfg_noValidParams);
    RUN_TEST(test_power_setThermalCfg_nullParam);
    RUN_TEST(test_power_setThermalCfg_noValidParams);
    RUN_TEST(test_power_setThermalCfg_TsdOrdLevel);
    RUN_TEST(test_power_setThermalCfg_TwarnLevel);

    /*** Finish unity testing ***/
    return UNITY_END();
}

static void resetAllTps6522xBuckRegisters(void)
{
    uint8_t                                  i = 0;
    uint8_t                                  txBuffer = 0;
    const Pmic_powerTps6522xBuckRegisters_t *pBuckRegisters = NULL;

    // Obtain BUCK registers
    Pmic_get_tps6522x_pwrBuckRegs(&pBuckRegisters);

    // Set values of all BUCK registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[0].buckRailSelRegAddr, &txBuffer, 1);
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckCtrlRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckConfRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckVoutRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pBuckRegisters[i].buckPgWindowRegAddr, &txBuffer, 1);
    }
}

static void resetAllTps6522xLdoRegisters(void)
{
    uint8_t                                 i = 0;
    uint8_t                                 txBuffer = 0;
    const Pmic_powerTps6522xLdoRegisters_t *pLdoRegisters = NULL;

    // Obtain LDO registers
    Pmic_get_tps6522x_pwrLdoRegs(&pLdoRegisters);

    // Set values of all LDO registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[0].ldoRailSelRegAddr, &txBuffer, 1);
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[i].ldoCtrlRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[i].ldoVoutRegAddr, &txBuffer, 1);
        (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pLdoRegisters[i].ldoPgWindowRegAddr, &txBuffer, 1);
    }
}

static void resetAllTps6522xVccaVmonRegisters(void)
{
    uint8_t                                      i = 0;
    uint8_t                                      txBuffer = 0;
    const Pmic_powerTps6522xVccaVmonRegisters_t *pVccaVmonRegisters = NULL;

    // Obtain VCCA_VMON/VMONx registers
    Pmic_get_tps6522x_PwrVccaVmonRegisters(&pVccaVmonRegisters);

    // Set values of all VCCA_MON/VMONx registers to be zero
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[0].vccaVmonCtrlRegAddr, &txBuffer, 1);
    (void)pmicI2CWrite(&pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[0].vccaVmonRailSelRegAddr, &txBuffer, 1);
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_VOLTAGE_MONITOR_NUM; i++)
    {
        switch (i)
        {
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[i].vmonPgLevelRegAddr, &txBuffer, 1);
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[i].vmonPgWindowRegAddr, &txBuffer, 1);

                break;
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON:
                (void)pmicI2CWrite(
                    &pmicCoreHandle, PMIC_MAIN_INST, pVccaVmonRegisters[i].vccaPgWindowRegAddr, &txBuffer, 1);

                break;
        }
    }
}

static void disablePmicPowerResources(void)
{
    const uint8_t devIdReg = 0x01;
    uint8_t       pmicDevId = 0x00;
    int32_t       status = PMIC_ST_SUCCESS;

    // Wait for PMIC connection
    while (1)
    {
        status = pmicI2CRead(&pmicCoreHandle, PMIC_MAIN_INST, devIdReg, &pmicDevId, 1);

        if (status == PMIC_ST_SUCCESS)
        {
            break;
        }
    }

    // Set all BUCK registers to zero
    resetAllTps6522xBuckRegisters();

    // Set all LDO registers to zero
    resetAllTps6522xLdoRegisters();

    // Set all VCCA_VMON/VMONx registers to zero
    resetAllTps6522xVccaVmonRegisters();
}

static void resetBurtonPwrCfg_withAllValidParams(Pmic_powerTps6522xPowerResourceCfg_t *burtonPwrRsrcCfg)
{
    // clang-format off
    uint8_t i = 0;

    burtonPwrRsrcCfg->validParams =
        PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT   | PMIC_POWER_TPS6522X_CFG_BUCK2_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_BUCK3_VALID_SHIFT   | PMIC_POWER_TPS6522X_CFG_BUCK4_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT    | PMIC_POWER_TPS6522X_CFG_LDO2_VALID_SHIFT  |
        PMIC_POWER_TPS6522X_CFG_LDO3_VALID_SHIFT    | PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_VMON2_VALID_SHIFT   | PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT;

    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].validParams =
            PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID_SHIFT       | PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID_SHIFT |
            PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID_SHIFT | PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID_SHIFT |
            PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID_SHIFT  | PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT
            | PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID_SHIFT   |
            PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID_SHIFT;

        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckPldn       = PMIC_POWER_TPS6522X_BUCK_PLDN_DISABLE;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckVmonEn     = PMIC_POWER_TPS6522X_BUCK_VMON_DISABLE;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckPwmOption  = PMIC_POWER_TPS6522X_BUCK_PWM_AUTO;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckEn         = PMIC_POWER_TPS6522X_BUCK_DISABLE;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckSlewRate   = PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckVoltage_mv = 500;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckVmonThr    = PMIC_POWER_TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckRailGrpSel = PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_NONE;
    }

    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].validParams =
            PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID_SHIFT | PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID_SHIFT  |
            PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID_SHIFT           | PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID_SHIFT     |
            PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT   | PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID_SHIFT |
            PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID_SHIFT;

        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoDischargeEn = PMIC_POWER_TPS6522X_LDO_DISCHARGE_DISABLE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoVmonEn      = PMIC_POWER_TPS6522X_LDO_VMON_DISABLE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoEn          = PMIC_POWER_TPS6522X_LDO_DISABLE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoMode        = PMIC_POWER_TPS6522X_LDO_BYP_CONFIG_LDO_MODE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoVoltage_mv  = 500;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoVmonThr     = PMIC_POWER_TPS6522X_LDO_VMON_THR_3_PCT;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoRailGrpSel  = PMIC_POWER_TPS6522X_LDO_RAIL_SEL_NONE;
    }

    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.validParams =
        PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID_SHIFT  | PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID_SHIFT           | PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID_SHIFT      | PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID_SHIFT  | PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID_SHIFT  |
        PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID_SHIFT | PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID_SHIFT |
        PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID_SHIFT  |
        PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID_SHIFT;

    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmonDeglitchSel =
        PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2En         = PMIC_POWER_TPS6522X_VMON2_DISABLE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1En         = PMIC_POWER_TPS6522X_VMON1_DISABLE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaVmonEn      = PMIC_POWER_TPS6522X_VCCA_VMON_DISABLE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaPgLevel     = PMIC_POWER_TPS6522X_VCCA_PG_LEVEL_3_3_V;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaVmonThr     = PMIC_POWER_TPS6522X_VCCA_VMON_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaRailGrpSel  = PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1Thr        = PMIC_POWER_TPS6522X_VMON1_THR_3_PCT_OR_30_MV;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1PgLevel_mv = 500;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1RailGrpSel = PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2Thr        = PMIC_POWER_TPS6522X_VMON2_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2PgLevel_mv = 500;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2RailGrpSel = PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_NONE;

    // clang-format on
}

static void resetBurtonPwrCfg_withNoValidParams(Pmic_powerTps6522xPowerResourceCfg_t *burtonPwrRsrcCfg)
{
    // clang-format off
    uint8_t i = 0;

    burtonPwrRsrcCfg->validParams = 0;

    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].validParams = 0;

        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckPldn       = PMIC_POWER_TPS6522X_BUCK_PLDN_DISABLE;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckVmonEn     = PMIC_POWER_TPS6522X_BUCK_VMON_DISABLE;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckPwmOption  = PMIC_POWER_TPS6522X_BUCK_PWM_AUTO;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckEn         = PMIC_POWER_TPS6522X_BUCK_DISABLE;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckSlewRate   = PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckVoltage_mv = 500;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckVmonThr    = PMIC_POWER_TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV;
        burtonPwrRsrcCfg->buckPwrRsrcCfg[i].buckRailGrpSel = PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_NONE;
    }

    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].validParams = 0;

        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoDischargeEn = PMIC_POWER_TPS6522X_LDO_DISCHARGE_DISABLE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoVmonEn      = PMIC_POWER_TPS6522X_LDO_VMON_DISABLE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoEn          = PMIC_POWER_TPS6522X_LDO_DISABLE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoMode        = PMIC_POWER_TPS6522X_LDO_BYP_CONFIG_LDO_MODE;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoVoltage_mv  = 500;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoVmonThr     = PMIC_POWER_TPS6522X_LDO_VMON_THR_3_PCT;
        burtonPwrRsrcCfg->ldoPwrRsrcCfg[i].ldoRailGrpSel  = PMIC_POWER_TPS6522X_LDO_RAIL_SEL_NONE;
    }

    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.validParams = 0;

    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmonDeglitchSel =
        PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2En         = PMIC_POWER_TPS6522X_VMON2_DISABLE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1En         = PMIC_POWER_TPS6522X_VMON1_DISABLE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaVmonEn      = PMIC_POWER_TPS6522X_VCCA_VMON_DISABLE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaPgLevel     = PMIC_POWER_TPS6522X_VCCA_PG_LEVEL_3_3_V;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaVmonThr     = PMIC_POWER_TPS6522X_VCCA_VMON_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vccaRailGrpSel  = PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1Thr        = PMIC_POWER_TPS6522X_VMON1_THR_3_PCT_OR_30_MV;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1PgLevel_mv = 500;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon1RailGrpSel = PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2Thr        = PMIC_POWER_TPS6522X_VMON2_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2PgLevel_mv = 500;
    burtonPwrRsrcCfg->vccaVmonPwrRsrcCfg.vmon2RailGrpSel = PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_NONE;

    // clang-format on
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test error handling for when PMIC handle is NULL
 */
void test_power_getConfiguration_pmicHandle_null(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Pass NULL PMIC handle and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrResourceCfg(NULL, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test API error handing for when Power Resource CFG input parameter
 *                                               is NULL
 */
void test_power_getConfiguration_pwrRsrcCfg_null(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Pass NULL power resource CFG and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test API error handing for when there are no valid parameters
 *                                               within Power Resource CFG input param
 */
void test_power_getConfiguration_pwrRsrcCfg_noValidParam(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Set validParams of power resource CFG to zero
    pwrRsrcCfg.validParams = 0;

    // Pass power resource CFG and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                               within Buck Power Resource CFG
 */
void test_power_getConfiguration_buckPwrRsrcCfg_noValidParam(void)
{
    uint8_t                              i = 0;
    uint8_t                              validParams = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Save buck validParam
        validParams = pwrRsrcCfg.buckPwrRsrcCfg[i].validParams;

        // Set its validParam to zero
        pwrRsrcCfg.buckPwrRsrcCfg[i].validParams = 0;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Restore buck validParam
        pwrRsrcCfg.buckPwrRsrcCfg[i].validParams = validParams;
    }
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                               within LDO Power Resource CFG
 */
void test_power_getConfiguration_ldoPwrRsrcCfg_noValidParam(void)
{
    uint8_t                              i = 0;
    uint8_t                              validParams = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Save LDO validParam
        validParams = pwrRsrcCfg.ldoPwrRsrcCfg[i].validParams;

        // Set its validParam to zero
        pwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = 0;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Restore LDO validParam
        pwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = validParams;
    }
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                               within VCCA_VMON/VMONx Power Resource CFG
 */
void test_power_getConfiguration_vccaVmonPwrRsrcCfg_noValidParam(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Set validParams of VCCA_VMON/VMONx power resource CFG to zero
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = 0;

    // Pass power resource CFG and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrResourceCfg: Test API response for when all its input parameters are valid
 *                                               (no null parameters, acceptable power resource CFG validParams,
 *                                               etc.)
 */
void test_power_getConfiguration_validParameters(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Pass in the power resource CFG and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handling for when PMIC handle is NULL
 */
void test_power_setConfiguration_pmicHandle_null(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Pass in NULL PMIC handle and compare expected vs. actual return code
    status = Pmic_powerTps6522xSetPwrResourceCfg(NULL, pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when there are no valid parameters
 *                                               within Power Resource CFG input param
 */
void test_power_setConfiguration_pwrRsrcCfg_noValidParam(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // Pass in power resource CFG with no validParams and compare expected vs. actual return code
    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 * \brief   Pmic_powerTps6522xSetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                               within Buck Power Resource CFG
 */
void test_power_setConfiguration_buckPwrRsrcCfg_noValidParam(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Set buck validParam
        pwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT + i;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }
}

/**
 * \brief   Pmic_powerTps6522xSetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                               within LDO Power Resource CFG
 */
void test_power_setConfiguration_ldoPwrRsrcCfg_noValidParam(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Set LDO validParam
        pwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT + i;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }
}

/**
 * \brief   Pmic_powerTps6522xSetPwrResourceCfg: Test API error handling for when there are no valid parameters
 *                                               within VCCA_VMON/VMONx Power Resource CFG
 */
void test_power_setConfiguration_vccaVmonPwrRsrcCfg_noValidParam(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // For each VMON...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_VOLTAGE_MONITOR_NUM; i++)
    {
        // Set VCCA_VMON/VMONx validParam
        pwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT + i;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }
}

static void compareBuckPwrRsrcCfg(const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPwrRsrcCfg_1,
                                  const Pmic_powerTps6522xBuckPowerResourceCfg_t buckPwrRsrcCfg_2,
                                  const uint16_t                                 bitFieldValidParamShift_ignore)
{
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckPldn, buckPwrRsrcCfg_2.buckPldn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckVmonEn, buckPwrRsrcCfg_2.buckVmonEn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckPwmOption, buckPwrRsrcCfg_2.buckPwmOption);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckEn, buckPwrRsrcCfg_2.buckEn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckSlewRate, buckPwrRsrcCfg_2.buckSlewRate);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckVoltage_mv, buckPwrRsrcCfg_2.buckVoltage_mv);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckPwrRsrcCfg_1.buckRailGrpSel, buckPwrRsrcCfg_2.buckRailGrpSel);
    }
}

static void compareLdoPwrRsrcCfg(const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPwrRsrcCfg_1,
                                 const Pmic_powerTps6522xLdoPowerResourceCfg_t ldoPwrRsrcCfg_2,
                                 const uint16_t                                bitFieldValidParamShift_ignore)
{
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoDischargeEn, ldoPwrRsrcCfg_2.ldoDischargeEn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoVmonEn, ldoPwrRsrcCfg_2.ldoVmonEn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoEn, ldoPwrRsrcCfg_2.ldoEn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoMode, ldoPwrRsrcCfg_2.ldoMode);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoVoltage_mv, ldoPwrRsrcCfg_2.ldoVoltage_mv);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoVmonThr, ldoPwrRsrcCfg_2.ldoVmonThr);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoPwrRsrcCfg_1.ldoRailGrpSel, ldoPwrRsrcCfg_2.ldoRailGrpSel);
    }
}

static void compareVccaVmonPwrRsrcCfg(const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg_1,
                                      const Pmic_powerTps6522xVccaVmonPowerResourceCfg_t vccaVmonPwrRsrcCfg_2,
                                      const uint16_t                                     bitFieldValidParamShift_ignore)
{
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmonDeglitchSel, vccaVmonPwrRsrcCfg_2.vmonDeglitchSel);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon2En, vccaVmonPwrRsrcCfg_2.vmon2En);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon1En, vccaVmonPwrRsrcCfg_2.vmon1En);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vccaVmonEn, vccaVmonPwrRsrcCfg_2.vccaVmonEn);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vccaPgLevel, vccaVmonPwrRsrcCfg_2.vccaPgLevel);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vccaVmonThr, vccaVmonPwrRsrcCfg_2.vccaVmonThr);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vccaRailGrpSel, vccaVmonPwrRsrcCfg_2.vccaRailGrpSel);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon1Thr, vccaVmonPwrRsrcCfg_2.vmon1Thr);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon1PgLevel_mv, vccaVmonPwrRsrcCfg_2.vmon1PgLevel_mv);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon1RailGrpSel, vccaVmonPwrRsrcCfg_2.vmon1RailGrpSel);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon2Thr, vccaVmonPwrRsrcCfg_2.vmon2Thr);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon2PgLevel_mv, vccaVmonPwrRsrcCfg_2.vmon2PgLevel_mv);
    }
    if (bitFieldValidParamShift_ignore != PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonPwrRsrcCfg_1.vmon2RailGrpSel, vccaVmonPwrRsrcCfg_2.vmon2RailGrpSel);
    }
}

static void comparePwrRsrcCfg_ignoreBitField(const Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg_1,
                                             const Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg_2,
                                             const uint16_t                             pwrRsrcValidParamShift_ignore,
                                             const uint16_t                             bitFieldValidParamShift_ignore)
{
    uint8_t  i = 0;
    uint16_t pwrRsrcValidParamShift = 0;

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Calculate buck validParam
        pwrRsrcValidParamShift = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;

        // If there is a match in validParams...
        if (pwrRsrcValidParamShift == pwrRsrcValidParamShift_ignore)
        {
            // Target validParam found; compare all bit fields except for target bit field
            compareBuckPwrRsrcCfg(
                pwrRsrcCfg_1.buckPwrRsrcCfg[i], pwrRsrcCfg_2.buckPwrRsrcCfg[i], bitFieldValidParamShift_ignore);
        }
        else
        {
            // No target validParam for the power resource; compare all bit fields
            compareBuckPwrRsrcCfg(pwrRsrcCfg_1.buckPwrRsrcCfg[i], pwrRsrcCfg_2.buckPwrRsrcCfg[i], 0);
        }
    }

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Calculate LDO validParam
        pwrRsrcValidParamShift = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;

        // If there is a match in validParams...
        if (pwrRsrcValidParamShift == pwrRsrcValidParamShift_ignore)
        {
            // Target validParam found; compare all bit fields except for target bit field
            compareLdoPwrRsrcCfg(
                pwrRsrcCfg_1.ldoPwrRsrcCfg[i], pwrRsrcCfg_2.ldoPwrRsrcCfg[i], bitFieldValidParamShift_ignore);
        }
        else
        {
            // No target validParam for the power resource; compare all bit fields
            compareLdoPwrRsrcCfg(pwrRsrcCfg_1.ldoPwrRsrcCfg[i], pwrRsrcCfg_2.ldoPwrRsrcCfg[i], 0);
        }
    }

    // If there is a match in VCCA_VMON/VMONx validParams...
    if ((pwrRsrcValidParamShift_ignore == PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT) ||
        (pwrRsrcValidParamShift_ignore == PMIC_POWER_TPS6522X_CFG_VMON2_VALID_SHIFT) ||
        (pwrRsrcValidParamShift_ignore == PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT))
    {
        // Target power resource found; compare all bit fields except for target bit field
        compareVccaVmonPwrRsrcCfg(
            pwrRsrcCfg_1.vccaVmonPwrRsrcCfg, pwrRsrcCfg_2.vccaVmonPwrRsrcCfg, bitFieldValidParamShift_ignore);
    }
    else
    {
        // No target validParam for the power resource; compare all bit fields
        compareVccaVmonPwrRsrcCfg(pwrRsrcCfg_1.vccaVmonPwrRsrcCfg, pwrRsrcCfg_2.vccaVmonPwrRsrcCfg, 0);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable Buck Pull-down resistor
 */
void test_power_setConfiguration_buckPldnEnableDisable(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_PLDN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_PLDN_VALID_SHIFT;

        // Enable BUCK_PLDN
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPldn = PMIC_POWER_TPS6522X_BUCK_PLDN_ENABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_PLDN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPldn, actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckPldn);

        // Disable BUCK_PLDN
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPldn = PMIC_POWER_TPS6522X_BUCK_PLDN_DISABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_PLDN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPldn, actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckPldn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable Buck VMON
 */
void test_power_setConfiguration_buckVmonEnableDisable(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VMON_EN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_VMON_EN_VALID_SHIFT;

        // Enable buck VMON
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonEn = PMIC_POWER_TPS6522X_BUCK_VMON_ENABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_VMON_EN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonEn,
                          actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonEn);

        // Disable buck VMON
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonEn = PMIC_POWER_TPS6522X_BUCK_VMON_DISABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_VMON_EN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonEn,
                          actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure Buck to operate in AUTO mode
 *                                               or FPWM mode
 */
void test_power_setConfiguration_buckFPWM(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_FPWM validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_PWM_OPTION_VALID_SHIFT;

        // Set BUCK_FPWM to forced
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPwmOption = PMIC_POWER_TPS6522X_BUCK_PWM_FORCED;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_FPWM and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPwmOption,
                          actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckPwmOption);

        // Set BUCK_FPWM to auto
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPwmOption = PMIC_POWER_TPS6522X_BUCK_PWM_AUTO;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_VMON_EN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckPwmOption,
                          actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckPwmOption);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable Buck regulator
 */
void test_power_setConfiguration_buckEn(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_EN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_EN_VALID_SHIFT;

        // Enable buck
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckEn = PMIC_POWER_TPS6522X_BUCK_ENABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_EN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckEn, actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckEn);

        // Disable buck
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckEn = PMIC_POWER_TPS6522X_BUCK_DISABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_EN and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckEn, actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set Buck slew rate
 */
void test_power_setConfiguration_buckSlewRate(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xBuckSlewRate_t     buckSlewRate;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_SLEW_RATE validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_SLEW_RATE_VALID_SHIFT;

        buckSlewRate = PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US;
        do
        {
            // Set buck slew rate
            expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckSlewRate = buckSlewRate;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual buck slew rate and compare expected vs. actual value
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckSlewRate,
                              actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckSlewRate);
        }
        while ((buckSlewRate++) != PMIC_POWER_TPS6522X_BUCK_SLEW_RATE_1_25_MV_PER_US);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when Buck voltage is below range
 */
void test_power_setConfiguration_buckVout_voltageBelowRange(void)
{
    uint8_t                              i = 0;
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything less than 500 mV for a buck should result in an error
        for (voltage_mv = 499; voltage_mv != 0; voltage_mv--)
        {
            expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVoltage_mv = voltage_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when Buck voltage is above range
 */
void test_power_setConfiguration_buckVout_voltageAboveRange(void)
{
    uint8_t                              i = 0;
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything above 3300 mV for a buck should result in an error
        for (voltage_mv = 3301; voltage_mv != 4000; voltage_mv++)
        {
            expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVoltage_mv = voltage_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

static void setPwrConfig_buck1Vout_test(const uint16_t voltageRangeMin_mv,
                                        const uint16_t voltageRangeMax_mv,
                                        const uint8_t  voltageStep)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Set buck validParam and BUCK_VOLTAGE_MV validParam
    // for both expected and actual power resource CFGs
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT;
    expectedPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].validParams =
        PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT;
    actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
    actualPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].validParams =
        expectedPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].validParams;

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // for each voltage step...
    for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
    {
        // Set voltage
        expectedPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].buckVoltage_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual voltage and compare expected vs. actual voltage
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].buckVoltage_mv,
                          actualPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].buckVoltage_mv);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(
        initialPwrRsrcCfg,
        actualPwrRsrcCfg,
        expectedPwrRsrcCfg.validParams,
        expectedPwrRsrcCfg.buckPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_BUCK1].validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set Buck 1 output voltage
 */
void test_power_setConfiguration_buck1_buckVout(void)
{
    // Test setting BUCK1 VOUT within the range of 500 mV to 580 mV (20 mV steps)
    setPwrConfig_buck1Vout_test(PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MIN_VOLTAGE,
                                PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_1_MAX_VOLTAGE,
                                PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV);
    // Test setting BUCK1 VOUT within the range of 600 mV to 1095 mV (5 mV steps)
    setPwrConfig_buck1Vout_test(PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MIN_VOLTAGE,
                                PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_2_MAX_VOLTAGE,
                                PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV);
    // Test setting BUCK1 VOUT within the range of 1100 mV to 1650 mV (10 mV steps)
    setPwrConfig_buck1Vout_test(PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MIN_VOLTAGE,
                                PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_3_MAX_VOLTAGE,
                                PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV);
    // Test setting BUCK1 VOUT within the range of 1660 mV to 3300 mV (20 mV steps)
    setPwrConfig_buck1Vout_test(PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MIN_VOLTAGE,
                                PMIC_POWER_TPS6522X_BUCK1_VOLTAGE_RANGE_4_MAX_VOLTAGE,
                                PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV);
}

static void setPwrConfig_buck2_3_4_Vout_test(const uint16_t voltageRangeMin_mv,
                                             const uint16_t voltageRangeMax_mv,
                                             const uint8_t  voltageStep)
{
    uint8_t                              buckNum = 0;
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    for (buckNum = PMIC_POWER_TPS6522X_REGULATOR_BUCK2; buckNum <= PMIC_POWER_TPS6522X_REGULATOR_BUCK4; buckNum++)
    {
        // Set buck validParam and BUCK_VOLTAGE_MV validParam
        // for both expected and actual power resource CFGs
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << buckNum;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[buckNum].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_VOLTAGE_MV_VALID_SHIFT;
        actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
        actualPwrRsrcCfg.buckPwrRsrcCfg[buckNum].validParams = expectedPwrRsrcCfg.buckPwrRsrcCfg[buckNum].validParams;

        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // for each voltage step...
        for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
        {
            // Set voltage
            expectedPwrRsrcCfg.buckPwrRsrcCfg[buckNum].buckVoltage_mv = voltage_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual voltage and compare expected vs. actual voltage
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[buckNum].buckVoltage_mv,
                              actualPwrRsrcCfg.buckPwrRsrcCfg[buckNum].buckVoltage_mv);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[buckNum].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set Buck 2, Buck 3, Buck 4 output voltage
 */
void test_power_setConfiguration_buck2_3_4_buckVout(void)
{
    // Test setting BUCK2, BUCK3, BUCK4 VOUT within the range of 500 mV to 1150 mV (25 mV steps)
    setPwrConfig_buck2_3_4_Vout_test(PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MIN_VOLTAGE,
                                     PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_1_MAX_VOLTAGE,
                                     PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV);
    // Test setting BUCK2, BUCK3, BUCK4 VOUT within the range of 1200 mV to 3300 mV (50 mV steps)
    setPwrConfig_buck2_3_4_Vout_test(PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MIN_VOLTAGE,
                                     PMIC_POWER_TPS6522X_BUCK2_3_4_VOLTAGE_RANGE_2_MAX_VOLTAGE,
                                     PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set Buck VMON Threshold
 */
void test_power_setConfiguration_buckVmonThr(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xBuckVmonThr_t      buckVmonThr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VMON_THR validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_VMON_THR_VALID_SHIFT;

        buckVmonThr = PMIC_POWER_TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV;
        do
        {
            // Set buck VMON threshold
            expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonThr = buckVmonThr;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual buck VMON threshold and compare expected vs. actual value
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonThr,
                              actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckVmonThr);
        }
        while ((buckVmonThr++) != PMIC_POWER_TPS6522X_BUCK_VMON_THR_8_PCT_OR_80_MV);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure Buck Rail Group Selection
 */
void test_power_setConfiguration_buckRailGrpSel(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xBuckRailSel_t      buckRailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_RAIL_GRP_SEL validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_BUCK_RAIL_GRP_SEL_VALID_SHIFT;

        buckRailGrpSel = PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_NONE;
        do
        {
            // Set buck rail group
            expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckRailGrpSel = buckRailGrpSel;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual buck rail group and compare expected vs. actual value
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckPwrRsrcCfg[i].buckRailGrpSel,
                              actualPwrRsrcCfg.buckPwrRsrcCfg[i].buckRailGrpSel);
        }
        while ((buckRailGrpSel++) != PMIC_POWER_TPS6522X_BUCK_RAIL_SEL_OTHER);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable LDO discharge
 */
void test_power_setConfiguration_ldoDischargeEnableDisable(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_DISCHARGE_EN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_DISCHARGE_EN_VALID_SHIFT;

        // Enable LDO discharge
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoDischargeEn = PMIC_POWER_TPS6522X_LDO_DISCHARGE_ENABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO discharge enable and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoDischargeEn,
                          actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoDischargeEn);

        // Disable LDO discharge
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoDischargeEn = PMIC_POWER_TPS6522X_LDO_DISCHARGE_DISABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO discharge enable and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoDischargeEn,
                          actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoDischargeEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable LDO VMON
 */
void test_power_setConfiguration_ldoVmonEnableDisable(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VMON_EN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_VMON_EN_VALID_SHIFT;

        // Enable LDO VMON
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonEn = PMIC_POWER_TPS6522X_LDO_VMON_ENABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO VMON enable and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonEn, actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonEn);

        // Disable LDO VMON
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonEn = PMIC_POWER_TPS6522X_LDO_VMON_DISABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO VMON enable and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonEn, actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable LDO
 */
void test_power_setConfiguration_ldoEnableDisable(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_EN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_EN_VALID_SHIFT;

        // Enable LDO
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoEn = PMIC_POWER_TPS6522X_LDO_ENABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO enable and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoEn, actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoEn);

        // Disable LDO
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoEn = PMIC_POWER_TPS6522X_LDO_DISABLE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO enable and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoEn, actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure LDO to operate in
 *                                               Bypass Mode or LDO Mode
 */
void test_power_setConfiguration_ldoBypassConfig(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_BYP_CONFIG validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_MODE_VALID_SHIFT;

        // Set the LDO Bypass Config to LDO Mode
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoMode = PMIC_POWER_TPS6522X_LDO_BYP_CONFIG_LDO_MODE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO Bypass Config and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoMode, actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoMode);

        // Set the LDO Bypass Config to Bypass Mode
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoMode = PMIC_POWER_TPS6522X_LDO_BYP_CONFIG_BYPASS_MODE;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO Bypass Config and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoMode, actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoMode);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when LDO voltage is below range
 */
void test_power_setConfiguration_ldo1Vout_voltageBelowRange(void)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set LDO validParam and LDO_VOLTAGE_MV validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT;
    expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams =
        PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT;

    // Setting voltage_mv to anything less than 1200 mV for LDO1 should result in an error
    for (voltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VOLTAGE - 1; voltage_mv != 0; voltage_mv--)
    {
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when LDO1 voltage is above range
 */
void test_power_setConfiguration_ldo1Vout_voltageAboveRange(void)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set LDO validParam and LDO_VOLTAGE_MV validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT;
    expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams =
        PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT;

    // Setting voltage_mv to anything above 3300 mV for LDO1 should result in an error
    for (voltage_mv = PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VOLTAGE + 1;
         voltage_mv <= PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VOLTAGE + 100;
         voltage_mv++)
    {
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when LDO2, LDO3 voltage is below range
 */
void test_power_setConfiguration_ldo2_3_Vout_voltageBelowRange(void)
{
    uint8_t                              i = 0;
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For LDO2 and LDO3...
    for (i = PMIC_POWER_TPS6522X_REGULATOR_LDO2; i < PMIC_POWER_TPS6522X_REGULATOR_LDO3; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything less than 600 mV for LDO2, LDO3 should result in an error
        for (voltage_mv = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VOLTAGE - 1; voltage_mv != 0; voltage_mv--)
        {
            expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVoltage_mv = voltage_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when LDO2, LDO3 voltage is above range
 */
void test_power_setConfiguration_ldo2_3_Vout_voltageAboveRange(void)
{
    uint8_t                              i = 0;
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For LDO2 and LDO3...
    for (i = PMIC_POWER_TPS6522X_REGULATOR_LDO2; i < PMIC_POWER_TPS6522X_REGULATOR_LDO3; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything above 3400 mV for LDO2, LDO3 should result in an error
        for (voltage_mv = PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VOLTAGE + 1;
             voltage_mv <= PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VOLTAGE + 100;
             voltage_mv++)
        {
            expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVoltage_mv = voltage_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

static void setPwrConfig_ldo1Vout_test(const uint16_t voltageRangeMin_mv,
                                       const uint16_t voltageRangeMax_mv,
                                       const uint8_t  voltageStep)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Set LDO validParam and LDO_VOLTAGE_MV validParam
    // for both expected and actual power resource CFGs
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT;
    expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams =
        PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT;
    actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
    actualPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams =
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams;

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if (voltageStep == PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV)
    {
        // Set voltage
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltageRangeMin_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual voltage and compare expected vs. actual voltage
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv,
                          actualPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv);
    }
    else
    {
        for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
        {
            // Set voltage
            expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltage_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual voltage and compare expected vs. actual voltage
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv,
                              actualPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].ldoVoltage_mv);
        }
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.ldoPwrRsrcCfg[PMIC_POWER_TPS6522X_REGULATOR_LDO1].validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg:  Test whether API can set LDO1 output voltage
 */
void test_power_setConfiguration_ldo1_ldoVout(void)
{
    setPwrConfig_ldo1Vout_test(PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MIN_VOLTAGE,
                               PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_1_MAX_VOLTAGE,
                               PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV);
    setPwrConfig_ldo1Vout_test(PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MIN_VOLTAGE,
                               PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_2_MAX_VOLTAGE,
                               PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV);
    setPwrConfig_ldo1Vout_test(PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MIN_VOLTAGE,
                               PMIC_POWER_TPS6522X_LDO1_VOLTAGE_RANGE_3_MAX_VOLTAGE,
                               PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV);
}

static void setPwrConfig_ldo2_3_Vout_test(const uint16_t voltageRangeMin_mv,
                                          const uint16_t voltageRangeMax_mv,
                                          const uint8_t  voltageStep)
{
    uint8_t                              ldoNum = 0;
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For LDO2 and LDO3...
    for (ldoNum = PMIC_POWER_TPS6522X_REGULATOR_LDO2; ldoNum <= PMIC_POWER_TPS6522X_REGULATOR_LDO3; ldoNum++)
    {
        // Set LDO validParam and LDO_VOLTAGE_MV validParam
        // for both expected and actual power resource CFGs
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << ldoNum;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].validParams = PMIC_POWER_TPS6522X_CFG_LDO_VOLTAGE_MV_VALID_SHIFT;
        actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
        actualPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].validParams = expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].validParams;

        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if (voltageStep == PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV)
        {
            // Set voltage
            expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv = voltageRangeMin_mv;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual voltage and compare expected vs. actual voltage
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv,
                              actualPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv);
        }
        else
        {
            for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
            {
                // Set voltage
                expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv = voltage_mv;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Get actual voltage and compare expected vs. actual voltage
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv,
                                  actualPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].ldoVoltage_mv);
            }
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[ldoNum].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg:  Test whether API can set LDO1 output voltage
 */
void test_power_setConfiguration_ldo2_3_ldoVout(void)
{
    setPwrConfig_ldo2_3_Vout_test(PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MIN_VOLTAGE,
                                  PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_1_MAX_VOLTAGE,
                                  PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV);
    setPwrConfig_ldo2_3_Vout_test(PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MIN_VOLTAGE,
                                  PMIC_POWER_TPS6522X_LDO2_3_VOLTAGE_RANGE_2_MAX_VOLTAGE,
                                  PMIC_POWER_TPS6522X_VOLTAGE_STEP_0_MV);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set LDO VMON threshold
 */
void test_power_setConfiguration_ldoVmonThr(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xLdoVmonThr_t       ldoVmonThr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VMON_THR validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_VMON_THR_VALID_SHIFT;

        ldoVmonThr = PMIC_POWER_TPS6522X_LDO_VMON_THR_3_PCT;
        do
        {
            // Set LDO VMON threshold
            expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonThr = ldoVmonThr;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual LDO VMON threshold and compare expected vs. actual value
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonThr,
                              actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoVmonThr);
        }
        while ((ldoVmonThr++) != PMIC_POWER_TPS6522X_LDO_VMON_THR_8_PCT);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure LDO Rail Group Selection
 */
void test_power_setConfiguration_ldoRailGrpSel(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xLdoRailSel_t       ldoRailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0; i < PMIC_POWER_TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_RAIL_GRP_SEL validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams = PMIC_POWER_TPS6522X_CFG_LDO_RAIL_GRP_SEL_VALID_SHIFT;

        ldoRailGrpSel = PMIC_POWER_TPS6522X_LDO_RAIL_SEL_NONE;
        do
        {
            // Set LDO rail group
            expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoRailGrpSel = ldoRailGrpSel;
            status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual LDO rail group and compare expected vs. actual value
            status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoRailGrpSel,
                              actualPwrRsrcCfg.ldoPwrRsrcCfg[i].ldoRailGrpSel);
        }
        while ((ldoRailGrpSel++) != PMIC_POWER_TPS6522X_LDO_RAIL_SEL_OTHER);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoPwrRsrcCfg[i].validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure VMON Deglitch Selection
 */
void test_power_setConfiguration_vmonDeglitch(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xVmonDeglitchSel_t  vmonDeglitchSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VMON_DEGLITCH_SEL validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON_DEGLITCH_SEL_VALID_SHIFT;

    vmonDeglitchSel = PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US;
    do
    {
        // Set VMON deglitch select
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmonDeglitchSel = vmonDeglitchSel;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual VMON deglitch select and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmonDeglitchSel,
                          actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmonDeglitchSel);
    }
    while ((vmonDeglitchSel++) != PMIC_POWER_TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_20_US);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can enable/disable VMON1, VMON2, and VCCA_VMON
 */
void test_power_setConfiguration_VMON1_2_VCCA_VMON_EnableDisable(void)
{
    uint8_t                              vmonNum = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    vmonNum = PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VCCA_VMON;
    do
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set VMON validParam and VMON_EN validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT << vmonNum;
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID_SHIFT << vmonNum;

        switch (expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams)
        {
            case PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID_SHIFT:
                // Enable VMON 2
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En = PMIC_POWER_TPS6522X_VMON2_ENABLE;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En);

                // Disable VMON2
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En = PMIC_POWER_TPS6522X_VMON2_DISABLE;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En);

                break;
            case PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID_SHIFT:
                // Enable VMON 1
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En = PMIC_POWER_TPS6522X_VMON1_ENABLE;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En);

                // Disable VMON1
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En = PMIC_POWER_TPS6522X_VMON1_DISABLE;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En);

                break;
            case PMIC_POWER_TPS6522X_CFG_VCCA_VMON_EN_VALID_SHIFT:
                // Enable VCCA_VMON
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonEn = PMIC_POWER_TPS6522X_VCCA_VMON_ENABLE;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonEn,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonEn);

                // Disable VCCA_VMON
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonEn = PMIC_POWER_TPS6522X_VCCA_VMON_DISABLE;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonEn,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonEn);

                break;
        };

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
    }
    while ((vmonNum--) != PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set VCCA_VMON PG Level
 */
void test_power_setConfiguration_vccaPgLevel(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VCCA_PG_LEVEL validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_PG_LEVEL_VALID_SHIFT;

    // Set the VCCA PG level to 3.3V
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaPgLevel = PMIC_POWER_TPS6522X_VCCA_PG_LEVEL_3_3_V;
    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Read actual VCCA PG level and compare expected vs. actual value
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaPgLevel,
                      actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaPgLevel);

    // Set the VCCA PG level to 5.0V
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaPgLevel = PMIC_POWER_TPS6522X_VCCA_PG_LEVEL_5_0_V;
    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Read actual VCCA PG level and compare expected vs. actual value
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaPgLevel,
                      actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaPgLevel);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set VCCA_VMON Threshold
 */
void test_power_setConfiguration_vccaVmonThr(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xVccaVmonThr_t      vccaVmonThr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VCCA_VMON_THR validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_VMON_THR_VALID_SHIFT;

    vccaVmonThr = PMIC_POWER_TPS6522X_VCCA_VMON_THR_3_PCT;
    do
    {
        // Set VCCA_VMON powergood high/low threshold level
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonThr = vccaVmonThr;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual VCCA_VMON threshold and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonThr,
                          actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaVmonThr);
    }
    while ((vccaVmonThr++) != PMIC_POWER_TPS6522X_VCCA_VMON_THR_10_PCT);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure VCCA_VMON Rail Group Selection
 */
void test_power_setConfiguration_vccaRailGrpSel(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xVccaRailSel_t      vccaRailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VCCA_RAIL_GRP_SEL validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VCCA_RAIL_GRP_SEL_VALID_SHIFT;

    vccaRailGrpSel = PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_NONE;
    do
    {
        // Set VCCA rail group
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaRailGrpSel = vccaRailGrpSel;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual VCCA rail group and compare expected vs. actual value
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaRailGrpSel,
                          actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vccaRailGrpSel);
    }
    while ((vccaRailGrpSel++) != PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_OTHER);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when VMON1 voltage is below range
 *                                               and above range
 */
void test_power_setConfiguration_vmon1PgSet_voltageOutOfRange(void)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VMON1 validParam and VMON1_PG_LEVEL_MV validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID_SHIFT;

    // Setting voltage_mv to anything less than 500 mV for VMON1 should result in an error
    for (voltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VOLTAGE - 1; voltage_mv != 0; voltage_mv--)
    {
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1PgLevel_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Setting voltage_mv to anything above 3340 mV for VMON1 should result in an error
    for (voltage_mv = PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VOLTAGE + 1;
         voltage_mv <= PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VOLTAGE + 100;
         voltage_mv++)
    {
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1PgLevel_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test API error handing for when VMON2 voltage is below range
 *                                               and above range
 */
void test_power_setConfiguration_vmon2PgSet_voltageOutOfRange(void)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VMON2 validParam and VMON2_PG_LEVEL_MV validParam
    expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON2_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID_SHIFT;

    // Setting voltage_mv to anything less than 500 mV for VMON2 should result in an error
    for (voltage_mv = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VOLTAGE - 1; voltage_mv != 0; voltage_mv--)
    {
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2PgLevel_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Setting voltage_mv to anything above 3300 mV for VMON2 should result in an error
    for (voltage_mv = PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VOLTAGE + 1;
         voltage_mv <= PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VOLTAGE + 100;
         voltage_mv++)
    {
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2PgLevel_mv = voltage_mv;
        status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

static void setPwrConfig_vmon1_2_Vout_test(const uint8_t  vmonNum,
                                           const uint16_t voltageRangeMin_mv,
                                           const uint16_t voltageRangeMax_mv,
                                           const uint8_t  voltageStep)
{
    uint16_t                             voltage_mv = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Set VMONx validParam and VMONx_PG_LEVEL_MV validParam
    // for both expected and actual power resource CFGs
    expectedPwrRsrcCfg.validParams = (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                         PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT :
                                         PMIC_POWER_TPS6522X_CFG_VMON2_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = (vmonNum == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                                            PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID_SHIFT :
                                                            PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID_SHIFT;
    actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
    actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams;

    // Capture current configuration state of power resources for later comparison
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // for each voltage step...
    for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
    {
        switch (vmonNum)
        {
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
                // Set voltage
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1PgLevel_mv = voltage_mv;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Get actual voltage and compare expected vs. actual voltage
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1PgLevel_mv,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1PgLevel_mv);

                break;
            case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
                // Set voltage
                expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2PgLevel_mv = voltage_mv;
                status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Get actual voltage and compare expected vs. actual voltage
                status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2PgLevel_mv,
                                  actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2PgLevel_mv);

                break;
        }
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set VMON1 PG Level
 */
void test_power_setConfiguration_vmon1PgSet(void)
{
    const uint8_t vmonNum = PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1;

    // Test setting VMON1 PG_SET within the range of 500 mV to 580 mV (20 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MIN_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_1_MAX_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV);
    // Test setting VMON1 PG_SET within the range of 600 mV to 1095 mV (5 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MIN_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_2_MAX_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VOLTAGE_STEP_5_MV);
    // Test setting VMON1 PG_SET within the range of 1100 mV to 1650 mV (10 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MIN_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_3_MAX_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VOLTAGE_STEP_10_MV);
    // Test setting VMON1 PG_SET within the range of 1660 mV to 3340 mV (20 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MIN_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VMON1_VOLTAGE_RANGE_4_MAX_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VOLTAGE_STEP_20_MV);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set VMON2 PG Level
 */
void test_power_setConfiguration_vmon2PgSet(void)
{
    const uint8_t vmonNum = PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2;

    // Test setting VMON2 PG_SET within the range of 500 mV to 1150 mV (25 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum,
                                   PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MIN_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_1_MAX_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VOLTAGE_STEP_25_MV);
    // Test setting VMON2 PG_SET within the range of 1200 mV to 3300 mV (50 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum,
                                   PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MIN_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VMON2_VOLTAGE_RANGE_2_MAX_VOLTAGE,
                                   PMIC_POWER_TPS6522X_VOLTAGE_STEP_50_MV);
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can set VMON1 and VMON2 Threshold
 */
void test_power_setConfiguration_vmon1_2_Thr(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xVmon1Thr_t         vmon1Thr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    for (i = PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1; i <= PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set VMONx validParam and VMONx_THR validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = (i == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                                                PMIC_POWER_TPS6522X_CFG_VMON1_THR_VALID_SHIFT :
                                                                PMIC_POWER_TPS6522X_CFG_VMON2_THR_VALID_SHIFT;

        vmon1Thr = PMIC_POWER_TPS6522X_VMON1_THR_3_PCT_OR_30_MV;
        do
        {
            switch (i)
            {
                case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
                    // Set VMON1 Powergood Threshold
                    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1Thr = vmon1Thr;
                    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON1 Powergood Threshold and compare expected vs. actual value
                    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1Thr,
                                      actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1Thr);

                    break;
                case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
                    // Set VMON2 Powergood Threshold
                    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2Thr = (Pmic_powerTps6522xVmon2Thr_t)vmon1Thr;
                    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON2 Powergood Threshold and compare expected vs. actual value
                    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2Thr,
                                      actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2Thr);

                    break;
            }
        }
        while ((vmon1Thr++) != PMIC_POWER_TPS6522X_VMON1_THR_8_PCT_OR_80_MV);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xSetPwrResourceCfg: Test whether API can configure VMON1 and VMON2 Rail Group Selection
 */
void test_power_setConfiguration_vmon1_2_RailGrpSel(void)
{
    uint8_t                              i = 0;
    int32_t                              status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xPowerResourceCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    Pmic_powerTps6522xVmon1RailSel_t     vmon1RailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    for (i = PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1; i <= PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set VMONx validParam and VMONx_RAIL_GRP_SEL validParam
        expectedPwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams = (i == PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                                                PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID_SHIFT :
                                                                PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID_SHIFT;

        vmon1RailGrpSel = PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_NONE;
        do
        {
            switch (i)
            {
                case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON1:
                    // Set VMON1 rail group
                    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1RailGrpSel = vmon1RailGrpSel;
                    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON1 rail group and compare expected vs. actual value
                    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1RailGrpSel,
                                      actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1RailGrpSel);

                    break;
                case PMIC_POWER_TPS6522X_VOLTAGE_MONITOR_VMON2:
                    // Set VMON2 rail group
                    expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2RailGrpSel =
                        (Pmic_powerTps6522xVmon2RailSel_t)vmon1RailGrpSel;
                    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON2 rail group and compare expected vs. actual value
                    status = Pmic_powerTps6522xGetPwrResourceCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2RailGrpSel,
                                      actualPwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2RailGrpSel);

                    break;
            }
        }
        while ((vmon1RailGrpSel++) != PMIC_POWER_TPS6522X_VCCA_RAIL_SEL_OTHER);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams);
    }
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrRsrcStat: Test API error handling for when parameters are NULL
 */
void test_power_getPwrRsrcStat_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool    underOverVoltageStat = false;

    // Pass NULL handle into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrRsrcStat(NULL, PMIC_POWER_TPS6522X_BUCK1_UVOV_STAT, &underOverVoltageStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL variable into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetPwrRsrcStat(&pmicCoreHandle, PMIC_POWER_TPS6522X_BUCK1_UVOV_STAT, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrRsrcStat: Test whether API can read all Buck, LDO, and VCCA_VMON/VMONx status
 */
void test_power_getPwrRsrcStat_allPwrRsrc(void)
{
    int32_t             status = PMIC_ST_SUCCESS;
    pwrRsrcUVOVStatus_t pwrRsrcUVOVStat = PMIC_POWER_TPS6522X_BUCK1_UVOV_STAT;
    bool                underOverVoltageStat = false;

    do
    {
        // Get power resource UVOV status
        status = Pmic_powerTps6522xGetPwrRsrcStat(&pmicCoreHandle, pwrRsrcUVOVStat, &underOverVoltageStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Iterate until all power resource UVOV statuses are read
    }
    while ((pwrRsrcUVOVStat++) != PMIC_POWER_TPS6522X_VCCA_UVOV_STAT);
}

/**
 *  \brief   Pmic_powerTps6522xGetPwrRsrcStat: Test whether API can detect a UVOV status on VMON1 and VMON2.
 *
 *  \note    This test assumes that the VMONs are not connected to any voltage sources
 */
void test_power_getPwrRsrcStat_vmon1_2_UVOVStatDetection(void)
{
    int32_t                              status = PMIC_ST_SUCCESS;
    pwrRsrcUVOVStatus_t                  pwrRsrcUVOVStat = PMIC_POWER_TPS6522X_VMON1_UVOV_STAT;
    bool                                 underOverVoltageStat = false;
    Pmic_powerTps6522xPowerResourceCfg_t pwrRsrcCfg;

    // Initialize pwrRsrcCfg with VMON1 and VMON2 configuration
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);
    pwrRsrcCfg.validParams = PMIC_POWER_TPS6522X_CFG_VMON1_VALID_SHIFT | PMIC_POWER_TPS6522X_CFG_VMON2_VALID_SHIFT;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.validParams =
        PMIC_POWER_TPS6522X_CFG_VMON1_EN_VALID | PMIC_POWER_TPS6522X_CFG_VMON2_EN_VALID |
        PMIC_POWER_TPS6522X_CFG_VMON1_PG_LEVEL_MV_VALID | PMIC_POWER_TPS6522X_CFG_VMON1_RAIL_GRP_SEL_VALID |
        PMIC_POWER_TPS6522X_CFG_VMON2_PG_LEVEL_MV_VALID | PMIC_POWER_TPS6522X_CFG_VMON2_RAIL_GRP_SEL_VALID;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1En = PMIC_POWER_TPS6522X_VMON1_ENABLE;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2En = PMIC_POWER_TPS6522X_VMON2_ENABLE;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1PgLevel_mv = 3000;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2PgLevel_mv = 3000;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon1RailGrpSel = PMIC_POWER_TPS6522X_VMON1_RAIL_SEL_MCU;
    pwrRsrcCfg.vccaVmonPwrRsrcCfg.vmon2RailGrpSel = PMIC_POWER_TPS6522X_VMON2_RAIL_SEL_MCU;

    // Set power resource configuration
    status = Pmic_powerTps6522xSetPwrResourceCfg(&pmicCoreHandle, pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a short period of time so VMON1 and VMON2 could detect UV
    delayTimeInMs(&timerHandle, 500);

    do
    {
        // Get UVOV status
        status = Pmic_powerTps6522xGetPwrRsrcStat(&pmicCoreHandle, pwrRsrcUVOVStat, &underOverVoltageStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual UVOV status
        TEST_ASSERT_EQUAL(true, underOverVoltageStat);
    }
    while ((pwrRsrcUVOVStat++) != PMIC_POWER_TPS6522X_VMON2_UVOV_STAT);
}

static void resetThermalStat_withAllValidParams(Pmic_powerTps6522xThermalStat_t *thermalStat)
{
    thermalStat->validParams =
        PMIC_THERMAL_STAT_WARN_VALID | PMIC_THERMAL_STAT_ORD_SHTDWN_VALID | PMIC_THERMAL_STAT_IMM_SHTDWN_VALID;
    thermalStat->twarnStat = false;
    thermalStat->tsdOrdStat = false;
    thermalStat->tsdImmStat = false;
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrThermalStat: Test API error handling for when parameters are NULL
 */
void test_power_getPwrThermalStat_nullParam(void)
{
    int32_t                         status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalStat_t thermalStat;

    // Initialize thermalStat
    resetThermalStat_withAllValidParams(&thermalStat);

    // Pass NULL PMIC handle into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetThermalStat(NULL, &thermalStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL thermal status struct into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetThermalStat(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrThermalStat: Test API error handling for when there are no valid parameters
 */
void test_power_getPwrThermalStat_noValidParams(void)
{
    int32_t                         status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalStat_t thermalStat = {.validParams = 0};

    // Pass struct with validParams into API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetThermalStat(&pmicCoreHandle, &thermalStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetPwrThermalStat: Test whether API can get the thermal warning, orderly, and
 *                                               immediate thermal statuses
 */
void test_power_getPwrThermalStat_getAllStatus(void)
{
    int32_t                         status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalStat_t thermalStat;

    // Initialize thermalStat
    resetThermalStat_withAllValidParams(&thermalStat);

    // Get thermal status
    status = Pmic_powerTps6522xGetThermalStat(&pmicCoreHandle, &thermalStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

static void resetThermalCfg_withAllValidParams(Pmic_powerTps6522xThermalCfg_t *pThermalCfg)
{
    pThermalCfg->validParams =
        PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID_SHIFT | PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID_SHIFT;
    pThermalCfg->tsdOrdLvl = PMIC_POWER_TPS6522X_TSD_ORD_LVL_140_C;
    pThermalCfg->twarnLvl = PMIC_POWER_TPS6522X_TWARN_LVL_130C;
}

static void resetThermalCfg_withNoValidParams(Pmic_powerTps6522xThermalCfg_t *pThermalCfg)
{
    pThermalCfg->validParams = 0;
    pThermalCfg->tsdOrdLvl = PMIC_POWER_TPS6522X_TSD_ORD_LVL_140_C;
    pThermalCfg->twarnLvl = PMIC_POWER_TPS6522X_TWARN_LVL_130C;
}

/**
 *  \brief  Pmic_powerTps6522xGetThermalCfg: Test API error handling for when parameters are NULL
 */
void test_power_getThermalCfg_nullParam(void)
{
    int32_t                        status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg
    resetThermalCfg_withAllValidParams(&thermalCfg);

    // Pass NULL PMIC handle into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetThermalCfg(NULL, &thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL struct into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetThermalCfg(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xGetThermalCfg: Test API error handling for when there are no valid parameters
 */
void test_power_getThermalCfg_noValidParams(void)
{
    int32_t                        status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg with no valid Parameters
    resetThermalCfg_withNoValidParams(&thermalCfg);

    // Pass thermalCfg into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xSetThermalCfg: Test API error handling for when parameters are NULL
 */
void test_power_setThermalCfg_nullParam(void)
{
    int32_t                        status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg
    resetThermalCfg_withAllValidParams(&thermalCfg);

    // Pass NULL PMIC handle into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xSetThermalCfg(NULL, thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xSetThermalCfg: Test API error handling for when there are no valid parameters
 */
void test_power_setThermalCfg_noValidParams(void)
{
    int32_t                        status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg with no valid Parameters
    resetThermalCfg_withNoValidParams(&thermalCfg);

    // Pass thermalCfg into the API and compare expected vs. actual return code
    status = Pmic_powerTps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  Pmic_powerTps6522xSetThermalCfg: Test whether API can configure the orderly thermal shutdown level
 */
void test_power_setThermalCfg_TsdOrdLevel(void)
{
    int32_t                        status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalCfg_t thermalCfg_expected, thermalCfg_actual;

    // Initialize thermal CFGs
    resetThermalCfg_withNoValidParams(&thermalCfg_expected);
    resetThermalCfg_withAllValidParams(&thermalCfg_actual);
    thermalCfg_expected.validParams = PMIC_POWER_TPS6522X_TSD_ORD_LEVEL_VALID_SHIFT;

    // Set TSD_ORD_LEVEL to be 145C
    thermalCfg_expected.tsdOrdLvl = PMIC_POWER_TPS6522X_TSD_ORD_LVL_145_C;
    status = Pmic_powerTps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TSD_ORD_LEVEL
    status = Pmic_powerTps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.tsdOrdLvl, thermalCfg_actual.tsdOrdLvl);

    // Set TSD_ORD_LEVEL to be 140C
    thermalCfg_expected.tsdOrdLvl = PMIC_POWER_TPS6522X_TSD_ORD_LVL_140_C;
    status = Pmic_powerTps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TSD_ORD_LEVEL
    status = Pmic_powerTps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.tsdOrdLvl, thermalCfg_actual.tsdOrdLvl);
}

/**
 *  \brief  Pmic_powerTps6522xSetThermalCfg: Test whether API can configure the thermal warning level
 */
void test_power_setThermalCfg_TwarnLevel(void)
{
    int32_t                        status = PMIC_ST_SUCCESS;
    Pmic_powerTps6522xThermalCfg_t thermalCfg_expected, thermalCfg_actual;

    // Initialize thermal CFGs
    resetThermalCfg_withNoValidParams(&thermalCfg_expected);
    resetThermalCfg_withAllValidParams(&thermalCfg_actual);
    thermalCfg_expected.validParams = PMIC_POWER_TPS6522X_TWARN_LEVEL_VALID_SHIFT;

    // Set TWARN_LEVEL to be 140C
    thermalCfg_expected.twarnLvl = PMIC_POWER_TPS6522X_TWARN_LVL_140C;
    status = Pmic_powerTps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TWARN_LEVEL
    status = Pmic_powerTps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.twarnLvl, thermalCfg_actual.twarnLvl);

    // Set TWARN_LEVEL to be 130C
    thermalCfg_expected.twarnLvl = PMIC_POWER_TPS6522X_TWARN_LVL_130C;
    status = Pmic_powerTps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TWARN_LEVEL
    status = Pmic_powerTps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.twarnLvl, thermalCfg_actual.twarnLvl);
}

/**
 *  \brief  This function is called by Unity when it starts a test
 */
void setUp(void)
{
}

/**
 *  \brief  This function is called by Unity when it finishes a test
 */
void tearDown(void)
{
    disablePmicPowerResources();
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
}
