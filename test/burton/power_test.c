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
#include "tiva_testLib.h"

/* Test specific include */
#include "power_test.h"

/* Unity testing library */
#include "unity.h"

/* PMIC driver */
#include "pmic.h"

#define RUN_POWER_TESTS()   RUN_TEST(test_power_getConfiguration_pmicHandle_null);                      \
                            RUN_TEST(test_power_getConfiguration_pwrRsrcCfg_null);                      \
                            RUN_TEST(test_power_getConfiguration_pwrRsrcCfg_noValidParam);              \
                            RUN_TEST(test_power_getConfiguration_buckCfg_noValidParam);                 \
                            RUN_TEST(test_power_getConfiguration_ldoCfg_noValidParam);                  \
                            RUN_TEST(test_power_getConfiguration_vccaVmonCfg_noValidParam);             \
                            RUN_TEST(test_power_getConfiguration_validParameters);                      \
                            RUN_TEST(test_power_setConfiguration_pmicHandle_null);                      \
                            RUN_TEST(test_power_setConfiguration_pwrRsrcCfg_noValidParam);              \
                            RUN_TEST(test_power_setConfiguration_buckCfg_noValidParam);                 \
                            RUN_TEST(test_power_setConfiguration_ldoCfg_noValidParam);                  \
                            RUN_TEST(test_power_setConfiguration_vccaVmonCfg_noValidParam);             \
                            RUN_TEST(test_power_setConfiguration_buckPldnEnableDisable);                \
                            RUN_TEST(test_power_setConfiguration_buckVmonEnableDisable);                \
                            RUN_TEST(test_power_setConfiguration_buckFPWM);                             \
                            RUN_TEST(test_power_setConfiguration_buckEn);                               \
                            RUN_TEST(test_power_setConfiguration_buckSlewRate);                         \
                            RUN_TEST(test_power_setConfiguration_buckVout_voltageBelowRange);           \
                            RUN_TEST(test_power_setConfiguration_buckVout_voltageAboveRange);           \
                            RUN_TEST(test_power_setConfiguration_buck1_buckVout);                       \
                            RUN_TEST(test_power_setConfiguration_buck2_3_4_buckVout);                   \
                            RUN_TEST(test_power_setConfiguration_buckVmonThr);                          \
                            RUN_TEST(test_power_setConfiguration_buckRailGrpSel);                       \
                            RUN_TEST(test_power_setConfiguration_ldoDischargeEnableDisable);            \
                            RUN_TEST(test_power_setConfiguration_ldoVmonEnableDisable);                 \
                            RUN_TEST(test_power_setConfiguration_ldoEnableDisable);                     \
                            RUN_TEST(test_power_setConfiguration_ldoBypassConfig);                      \
                            RUN_TEST(test_power_setConfiguration_ldo1Vout_voltageBelowRange);           \
                            RUN_TEST(test_power_setConfiguration_ldo1Vout_voltageAboveRange);           \
                            RUN_TEST(test_power_setConfiguration_ldo2_3_Vout_voltageBelowRange);        \
                            RUN_TEST(test_power_setConfiguration_ldo2_3_Vout_voltageAboveRange);        \
                            RUN_TEST(test_power_setConfiguration_ldo1_ldoVout);                         \
                            RUN_TEST(test_power_setConfiguration_ldo2_3_ldoVout);                       \
                            RUN_TEST(test_power_setConfiguration_ldoVmonThr);                           \
                            RUN_TEST(test_power_setConfiguration_ldoRailGrpSel);                        \
                            RUN_TEST(test_power_setConfiguration_vmonDeglitch);                         \
                            RUN_TEST(test_power_setConfiguration_VMON1_2_VCCA_VMON_EnableDisable);      \
                            RUN_TEST(test_power_setConfiguration_vccaPgLevel);                          \
                            RUN_TEST(test_power_setConfiguration_vccaVmonThr);                          \
                            RUN_TEST(test_power_setConfiguration_vccaRailGrpSel);                       \
                            RUN_TEST(test_power_setConfiguration_vmon1PgSet_voltageOutOfRange);         \
                            RUN_TEST(test_power_setConfiguration_vmon2PgSet_voltageOutOfRange);         \
                            RUN_TEST(test_power_setConfiguration_vmon1PgSet);                           \
                            RUN_TEST(test_power_setConfiguration_vmon2PgSet);                           \
                            RUN_TEST(test_power_setConfiguration_vmon1_2_RailGrpSel);                   \
                            RUN_TEST(test_power_setConfiguration_vmon1_2_Thr);                          \
                            RUN_TEST(test_power_getPwrRsrcStat_nullParam);                              \
                            RUN_TEST(test_power_getPwrRsrcStat_allPwrRsrc);                             \
                            RUN_TEST(test_power_getPwrRsrcStat_vmon1_2_UVOVStatDetection);              \
                            RUN_TEST(test_power_getPwrThermalStat_nullParam);                           \
                            RUN_TEST(test_power_getPwrThermalStat_noValidParams);                       \
                            RUN_TEST(test_power_getPwrThermalStat_getAllStatus);                        \
                            RUN_TEST(test_power_getThermalCfg_nullParam);                               \
                            RUN_TEST(test_power_getThermalCfg_noValidParams);                           \
                            RUN_TEST(test_power_setThermalCfg_nullParam);                               \
                            RUN_TEST(test_power_setThermalCfg_noValidParams);                           \
                            RUN_TEST(test_power_setThermalCfg_TsdOrdLevel);                             \
                            RUN_TEST(test_power_setThermalCfg_TwarnLevel)                               \

timerHandle_t tHandle;
Pmic_CoreHandle_t pmicCoreHandle;

int main(void)
{
    /*** Variable declaration/initialization ***/
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    Pmic_CoreCfg_t pmicConfigData = {
        .validParams =
            (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT  | PMIC_CFG_COMM_MODE_VALID_SHIFT    | 
             PMIC_CFG_SLAVEADDR_VALID_SHIFT    | PMIC_CFG_QASLAVEADDR_VALID_SHIFT  | 
             PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT | PMIC_CFG_COMM_HANDLE_VALID_SHIFT  | 
             PMIC_CFG_COMM_IO_RD_VALID_SHIFT   | PMIC_CFG_COMM_IO_WR_VALID_SHIFT   |
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
    initializeTimerHandle(&tHandle);
    initializeTimer(&tHandle);

    /*** Clear the console before printing anything ***/
    clearConsole(&vcpHandle);

    /*** Put PMIC power resources into a known state for testing ***/
    disablePmicPowerResources(pmicCoreHandle);
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);

    /*** Ensure changes are propagated by waiting a certain period of time ***/
    delayTimeInMs(&tHandle, 1000);

    /*** Print welcome message ***/
    UARTStrPut(&vcpHandle, "Running all PMIC Power tests...\r\n\r\n");

    /*** Begin unity testing ***/
    UNITY_BEGIN();

    RUN_POWER_TESTS();

    /*** Finish unity testing ***/
    return UNITY_END();
}

static void resetBurtonPwrCfg_withAllValidParams(tps6522xPwrRsrcCfg_t *burtonPwrRsrcCfg)
{
    uint8_t i = 0U;

    burtonPwrRsrcCfg->validParams =
        TPS6522X_BUCK1_VALID_SHIFT   | TPS6522X_BUCK2_VALID_SHIFT |
        TPS6522X_BUCK3_VALID_SHIFT   | TPS6522X_BUCK4_VALID_SHIFT |
        TPS6522X_LDO1_VALID_SHIFT    | TPS6522X_LDO2_VALID_SHIFT  |
        TPS6522X_LDO3_VALID_SHIFT    | TPS6522X_VMON1_VALID_SHIFT |
        TPS6522X_VMON2_VALID_SHIFT   | TPS6522X_VCCA_VALID_SHIFT;

    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        burtonPwrRsrcCfg->buckCfg[i].validParams =
            TPS6522X_BUCK_PLDN_VALID_SHIFT       | TPS6522X_BUCK_VMON_EN_VALID_SHIFT |
            TPS6522X_BUCK_PWM_OPTION_VALID_SHIFT | TPS6522X_BUCK_EN_VALID_SHIFT |
            TPS6522X_BUCK_SLEW_RATE_VALID_SHIFT  | TPS6522X_BUCK_VOLTAGE_MV_VALID_SHIFT
            | TPS6522X_BUCK_VMON_THR_VALID_SHIFT   |
            TPS6522X_BUCK_RAIL_GRP_SEL_VALID_SHIFT;

        burtonPwrRsrcCfg->buckCfg[i].buckPldn       = TPS6522X_BUCK_PLDN_DISABLE;
        burtonPwrRsrcCfg->buckCfg[i].buckVmonEn     = TPS6522X_BUCK_VMON_DISABLE;
        burtonPwrRsrcCfg->buckCfg[i].buckPwmOption  = TPS6522X_BUCK_PWM_AUTO;
        burtonPwrRsrcCfg->buckCfg[i].buckEn         = TPS6522X_BUCK_DISABLE;
        burtonPwrRsrcCfg->buckCfg[i].buckSlewRate   = TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US;
        burtonPwrRsrcCfg->buckCfg[i].buckVoltage_mv = 500U;
        burtonPwrRsrcCfg->buckCfg[i].buckVmonThr    = TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV;
        burtonPwrRsrcCfg->buckCfg[i].buckRailGrpSel = TPS6522X_BUCK_RAIL_SEL_NONE;
    }

    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        burtonPwrRsrcCfg->ldoCfg[i].validParams =
            TPS6522X_LDO_DISCHARGE_EN_VALID_SHIFT | TPS6522X_LDO_VMON_EN_VALID_SHIFT  |
            TPS6522X_LDO_EN_VALID_SHIFT           | TPS6522X_LDO_MODE_VALID_SHIFT     |
            TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT   | TPS6522X_LDO_VMON_THR_VALID_SHIFT |
            TPS6522X_LDO_RAIL_GRP_SEL_VALID_SHIFT;

        burtonPwrRsrcCfg->ldoCfg[i].ldoDischargeEn = TPS6522X_LDO_DISCHARGE_DISABLE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoVmonEn      = TPS6522X_LDO_VMON_DISABLE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoEn          = TPS6522X_LDO_DISABLE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoMode        = TPS6522X_LDO_BYP_CONFIG_LDO_MODE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoVoltage_mv  = 500U;
        burtonPwrRsrcCfg->ldoCfg[i].ldoVmonThr     = TPS6522X_LDO_VMON_THR_3_PCT;
        burtonPwrRsrcCfg->ldoCfg[i].ldoRailGrpSel  = TPS6522X_LDO_RAIL_SEL_NONE;
    }

    burtonPwrRsrcCfg->vccaVmonCfg.validParams =
        TPS6522X_VMON_DEGLITCH_SEL_VALID_SHIFT  | TPS6522X_VMON2_EN_VALID_SHIFT |
        TPS6522X_VMON1_EN_VALID_SHIFT           | TPS6522X_VCCA_VMON_EN_VALID_SHIFT |
        TPS6522X_VCCA_PG_LEVEL_VALID_SHIFT      | TPS6522X_VCCA_VMON_THR_VALID_SHIFT |
        TPS6522X_VCCA_RAIL_GRP_SEL_VALID_SHIFT  | TPS6522X_VMON1_THR_VALID_SHIFT |
        TPS6522X_VMON1_PG_LEVEL_MV_VALID_SHIFT  |
        TPS6522X_VMON1_RAIL_GRP_SEL_VALID_SHIFT | TPS6522X_VMON2_THR_VALID_SHIFT |
        TPS6522X_VMON2_PG_LEVEL_MV_VALID_SHIFT  |
        TPS6522X_VMON2_RAIL_GRP_SEL_VALID_SHIFT;

    burtonPwrRsrcCfg->vccaVmonCfg.vmonDeglitchSel =
        TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2En         = TPS6522X_VMON2_DISABLE;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1En         = TPS6522X_VMON1_DISABLE;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaVmonEn      = TPS6522X_VCCA_VMON_DISABLE;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaPgLevel     = TPS6522X_VCCA_PG_LEVEL_3_3_V;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaVmonThr     = TPS6522X_VCCA_VMON_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaRailGrpSel  = TPS6522X_VCCA_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1Thr        = TPS6522X_VMON1_THR_3_PCT_OR_30_MV;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1PgLevel_mv = 500U;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1RailGrpSel = TPS6522X_VMON1_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2Thr        = TPS6522X_VMON2_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2PgLevel_mv = 500U;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2RailGrpSel = TPS6522X_VMON2_RAIL_SEL_NONE;
}

static void resetBurtonPwrCfg_withNoValidParams(tps6522xPwrRsrcCfg_t *burtonPwrRsrcCfg)
{
    uint8_t i = 0U;

    burtonPwrRsrcCfg->validParams = 0U;

    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        burtonPwrRsrcCfg->buckCfg[i].validParams = 0U;

        burtonPwrRsrcCfg->buckCfg[i].buckPldn       = TPS6522X_BUCK_PLDN_DISABLE;
        burtonPwrRsrcCfg->buckCfg[i].buckVmonEn     = TPS6522X_BUCK_VMON_DISABLE;
        burtonPwrRsrcCfg->buckCfg[i].buckPwmOption  = TPS6522X_BUCK_PWM_AUTO;
        burtonPwrRsrcCfg->buckCfg[i].buckEn         = TPS6522X_BUCK_DISABLE;
        burtonPwrRsrcCfg->buckCfg[i].buckSlewRate   = TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US;
        burtonPwrRsrcCfg->buckCfg[i].buckVoltage_mv = 500U;
        burtonPwrRsrcCfg->buckCfg[i].buckVmonThr    = TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV;
        burtonPwrRsrcCfg->buckCfg[i].buckRailGrpSel = TPS6522X_BUCK_RAIL_SEL_NONE;
    }

    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        burtonPwrRsrcCfg->ldoCfg[i].validParams = 0U;

        burtonPwrRsrcCfg->ldoCfg[i].ldoDischargeEn = TPS6522X_LDO_DISCHARGE_DISABLE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoVmonEn      = TPS6522X_LDO_VMON_DISABLE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoEn          = TPS6522X_LDO_DISABLE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoMode        = TPS6522X_LDO_BYP_CONFIG_LDO_MODE;
        burtonPwrRsrcCfg->ldoCfg[i].ldoVoltage_mv  = 500U;
        burtonPwrRsrcCfg->ldoCfg[i].ldoVmonThr     = TPS6522X_LDO_VMON_THR_3_PCT;
        burtonPwrRsrcCfg->ldoCfg[i].ldoRailGrpSel  = TPS6522X_LDO_RAIL_SEL_NONE;
    }

    burtonPwrRsrcCfg->vccaVmonCfg.validParams = 0U;

    burtonPwrRsrcCfg->vccaVmonCfg.vmonDeglitchSel =
        TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2En         = TPS6522X_VMON2_DISABLE;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1En         = TPS6522X_VMON1_DISABLE;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaVmonEn      = TPS6522X_VCCA_VMON_DISABLE;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaPgLevel     = TPS6522X_VCCA_PG_LEVEL_3_3_V;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaVmonThr     = TPS6522X_VCCA_VMON_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonCfg.vccaRailGrpSel  = TPS6522X_VCCA_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1Thr        = TPS6522X_VMON1_THR_3_PCT_OR_30_MV;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1PgLevel_mv = 500U;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon1RailGrpSel = TPS6522X_VMON1_RAIL_SEL_NONE;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2Thr        = TPS6522X_VMON2_THR_3_PCT;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2PgLevel_mv = 500U;
    burtonPwrRsrcCfg->vccaVmonCfg.vmon2RailGrpSel = TPS6522X_VMON2_RAIL_SEL_NONE;
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test error handling for when PMIC handle is NULL
 */
void test_power_getConfiguration_pmicHandle_null(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Pass NULL PMIC handle and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcCfg(NULL, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test API error handing for when Power Resource CFG input parameter
 *                                               is NULL
 */
void test_power_getConfiguration_pwrRsrcCfg_null(void)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Pass NULL power resource CFG and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test API error handing for when there are no valid parameters
 *                                               within Power Resource CFG input param
 */
void test_power_getConfiguration_pwrRsrcCfg_noValidParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Set validParams of power resource CFG to zero
    pwrRsrcCfg.validParams = 0U;

    // Pass power resource CFG and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                               within Buck Power Resource CFG
 */
void test_power_getConfiguration_buckCfg_noValidParam(void)
{
    uint8_t i = 0U;
    uint8_t validParams = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Save buck validParam
        validParams = pwrRsrcCfg.buckCfg[i].validParams;

        // Set its validParam to zero
        pwrRsrcCfg.buckCfg[i].validParams = 0U;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Restore buck validParam
        pwrRsrcCfg.buckCfg[i].validParams = validParams;
    }
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                               within LDO Power Resource CFG
 */
void test_power_getConfiguration_ldoCfg_noValidParam(void)
{
    uint8_t i = 0U;
    uint8_t validParams = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Save LDO validParam
        validParams = pwrRsrcCfg.ldoCfg[i].validParams;

        // Set its validParam to zero
        pwrRsrcCfg.ldoCfg[i].validParams = 0U;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

        // Restore LDO validParam
        pwrRsrcCfg.ldoCfg[i].validParams = validParams;
    }
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                               within VCCA_VMON/VMONx Power Resource CFG
 */
void test_power_getConfiguration_vccaVmonCfg_noValidParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Set validParams of VCCA_VMON/VMONx power resource CFG to zero
    pwrRsrcCfg.vccaVmonCfg.validParams = 0U;

    // Pass power resource CFG and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrRsrcCfg: Test API response for when all its input parameters are valid
 *                                               (no null parameters, acceptable power resource CFG validParams,
 *                                               etc.)
 */
void test_power_getConfiguration_validParameters(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Pass in the power resource CFG and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handling for when PMIC handle is NULL
 */
void test_power_setConfiguration_pmicHandle_null(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withAllValidParams(&pwrRsrcCfg);

    // Pass in NULL PMIC handle and compare expected vs. actual return code
    status = tps6522xSetPwrRsrcCfg(NULL, pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when there are no valid parameters
 *                                               within Power Resource CFG input param
 */
void test_power_setConfiguration_pwrRsrcCfg_noValidParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // Pass in power resource CFG with no validParams and compare expected vs. actual return code
    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 * \brief   tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                               within Buck Power Resource CFG
 */
void test_power_setConfiguration_buckCfg_noValidParam(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Set buck validParam
        pwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT + i;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }
}

/**
 * \brief   tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                               within LDO Power Resource CFG
 */
void test_power_setConfiguration_ldoCfg_noValidParam(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Set LDO validParam
        pwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT + i;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }
}

/**
 * \brief   tps6522xSetPwrRsrcCfg: Test API error handling for when there are no valid parameters
 *                                               within VCCA_VMON/VMONx Power Resource CFG
 */
void test_power_setConfiguration_vccaVmonCfg_noValidParam(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);

    // For each VMON...
    for (i = 0U; i < TPS6522X_MAX_VOLTAGE_MONITOR_NUM; i++)
    {
        // Set VCCA_VMON/VMONx validParam
        pwrRsrcCfg.validParams = TPS6522X_VMON1_VALID_SHIFT + i;

        // Pass in power resource CFG and compare expected vs. actual return code
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, pwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }
}

static void compareBuckPwrRsrcCfg(const tps6522xBuckCfg_t buckCfg_1,
                                  const tps6522xBuckCfg_t buckCfg_2,
                                  const uint16_t bitFieldValidParamShift_ignore)
{
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_PLDN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckPldn, buckCfg_2.buckPldn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_VMON_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckVmonEn, buckCfg_2.buckVmonEn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_PWM_OPTION_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckPwmOption, buckCfg_2.buckPwmOption);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckEn, buckCfg_2.buckEn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_SLEW_RATE_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckSlewRate, buckCfg_2.buckSlewRate);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_VOLTAGE_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckVoltage_mv, buckCfg_2.buckVoltage_mv);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_BUCK_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(buckCfg_1.buckRailGrpSel, buckCfg_2.buckRailGrpSel);
    }
}

static void compareLdoPwrRsrcCfg(const tps6522xLdoCfg_t ldoCfg_1,
                                 const tps6522xLdoCfg_t ldoCfg_2,
                                 const uint16_t bitFieldValidParamShift_ignore)
{
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_DISCHARGE_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoDischargeEn, ldoCfg_2.ldoDischargeEn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_VMON_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoVmonEn, ldoCfg_2.ldoVmonEn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoEn, ldoCfg_2.ldoEn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_MODE_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoMode, ldoCfg_2.ldoMode);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoVoltage_mv, ldoCfg_2.ldoVoltage_mv);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_VMON_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoVmonThr, ldoCfg_2.ldoVmonThr);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_LDO_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(ldoCfg_1.ldoRailGrpSel, ldoCfg_2.ldoRailGrpSel);
    }
}

static void compareVccaVmonPwrRsrcCfg(const tps6522xVccaVmonCfg_t vccaVmonCfg_1,
                                      const tps6522xVccaVmonCfg_t vccaVmonCfg_2,
                                      const uint16_t bitFieldValidParamShift_ignore)
{
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON_DEGLITCH_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmonDeglitchSel, vccaVmonCfg_2.vmonDeglitchSel);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON2_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon2En, vccaVmonCfg_2.vmon2En);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON1_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon1En, vccaVmonCfg_2.vmon1En);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VCCA_VMON_EN_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vccaVmonEn, vccaVmonCfg_2.vccaVmonEn);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VCCA_PG_LEVEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vccaPgLevel, vccaVmonCfg_2.vccaPgLevel);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VCCA_VMON_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vccaVmonThr, vccaVmonCfg_2.vccaVmonThr);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VCCA_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vccaRailGrpSel, vccaVmonCfg_2.vccaRailGrpSel);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON1_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon1Thr, vccaVmonCfg_2.vmon1Thr);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON1_PG_LEVEL_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon1PgLevel_mv, vccaVmonCfg_2.vmon1PgLevel_mv);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON1_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon1RailGrpSel, vccaVmonCfg_2.vmon1RailGrpSel);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON2_THR_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon2Thr, vccaVmonCfg_2.vmon2Thr);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON2_PG_LEVEL_MV_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon2PgLevel_mv, vccaVmonCfg_2.vmon2PgLevel_mv);
    }
    if (bitFieldValidParamShift_ignore != TPS6522X_VMON2_RAIL_GRP_SEL_VALID_SHIFT)
    {
        TEST_ASSERT_EQUAL(vccaVmonCfg_1.vmon2RailGrpSel, vccaVmonCfg_2.vmon2RailGrpSel);
    }
}

static void comparePwrRsrcCfg_ignoreBitField(const tps6522xPwrRsrcCfg_t pwrRsrcCfg_1,
                                             const tps6522xPwrRsrcCfg_t pwrRsrcCfg_2,
                                             const uint16_t                             pwrRsrcValidParamShift_ignore,
                                             const uint16_t                             bitFieldValidParamShift_ignore)
{
    uint8_t  i = 0U;
    uint16_t pwrRsrcValidParamShift = 0U;

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Calculate buck validParam
        pwrRsrcValidParamShift = TPS6522X_BUCK1_VALID_SHIFT << i;

        // If there is a match in validParams...
        if (pwrRsrcValidParamShift == pwrRsrcValidParamShift_ignore)
        {
            // Target validParam found; compare all bit fields except for target bit field
            compareBuckPwrRsrcCfg(
                pwrRsrcCfg_1.buckCfg[i], pwrRsrcCfg_2.buckCfg[i], bitFieldValidParamShift_ignore);
        }
        else
        {
            // No target validParam for the power resource; compare all bit fields
            compareBuckPwrRsrcCfg(pwrRsrcCfg_1.buckCfg[i], pwrRsrcCfg_2.buckCfg[i], 0);
        }
    }

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Calculate LDO validParam
        pwrRsrcValidParamShift = TPS6522X_LDO1_VALID_SHIFT << i;

        // If there is a match in validParams...
        if (pwrRsrcValidParamShift == pwrRsrcValidParamShift_ignore)
        {
            // Target validParam found; compare all bit fields except for target bit field
            compareLdoPwrRsrcCfg(
                pwrRsrcCfg_1.ldoCfg[i], pwrRsrcCfg_2.ldoCfg[i], bitFieldValidParamShift_ignore);
        }
        else
        {
            // No target validParam for the power resource; compare all bit fields
            compareLdoPwrRsrcCfg(pwrRsrcCfg_1.ldoCfg[i], pwrRsrcCfg_2.ldoCfg[i], 0);
        }
    }

    // If there is a match in VCCA_VMON/VMONx validParams...
    if ((pwrRsrcValidParamShift_ignore == TPS6522X_VMON1_VALID_SHIFT) ||
        (pwrRsrcValidParamShift_ignore == TPS6522X_VMON2_VALID_SHIFT) ||
        (pwrRsrcValidParamShift_ignore == TPS6522X_VCCA_VALID_SHIFT))
    {
        // Target power resource found; compare all bit fields except for target bit field
        compareVccaVmonPwrRsrcCfg(
            pwrRsrcCfg_1.vccaVmonCfg, pwrRsrcCfg_2.vccaVmonCfg, bitFieldValidParamShift_ignore);
    }
    else
    {
        // No target validParam for the power resource; compare all bit fields
        compareVccaVmonPwrRsrcCfg(pwrRsrcCfg_1.vccaVmonCfg, pwrRsrcCfg_2.vccaVmonCfg, 0);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable Buck Pull-down resistor
 */
void test_power_setConfiguration_buckPldnEnableDisable(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_PLDN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_PLDN_VALID_SHIFT;

        // Enable BUCK_PLDN
        expectedPwrRsrcCfg.buckCfg[i].buckPldn = TPS6522X_BUCK_PLDN_ENABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_PLDN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckPldn, actualPwrRsrcCfg.buckCfg[i].buckPldn);

        // Disable BUCK_PLDN
        expectedPwrRsrcCfg.buckCfg[i].buckPldn = TPS6522X_BUCK_PLDN_DISABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_PLDN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckPldn, actualPwrRsrcCfg.buckCfg[i].buckPldn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable Buck VMON
 */
void test_power_setConfiguration_buckVmonEnableDisable(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VMON_EN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_VMON_EN_VALID_SHIFT;

        // Enable buck VMON
        expectedPwrRsrcCfg.buckCfg[i].buckVmonEn = TPS6522X_BUCK_VMON_ENABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_VMON_EN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckVmonEn,
                          actualPwrRsrcCfg.buckCfg[i].buckVmonEn);

        // Disable buck VMON
        expectedPwrRsrcCfg.buckCfg[i].buckVmonEn = TPS6522X_BUCK_VMON_DISABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_VMON_EN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckVmonEn,
                          actualPwrRsrcCfg.buckCfg[i].buckVmonEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure Buck to operate in AUTO mode
 *                                               or FPWM mode
 */
void test_power_setConfiguration_buckFPWM(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_FPWM validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_PWM_OPTION_VALID_SHIFT;

        // Set BUCK_FPWM to forced
        expectedPwrRsrcCfg.buckCfg[i].buckPwmOption = TPS6522X_BUCK_PWM_FORCED;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_FPWM and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckPwmOption,
                          actualPwrRsrcCfg.buckCfg[i].buckPwmOption);

        // Set BUCK_FPWM to auto
        expectedPwrRsrcCfg.buckCfg[i].buckPwmOption = TPS6522X_BUCK_PWM_AUTO;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_VMON_EN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckPwmOption,
                          actualPwrRsrcCfg.buckCfg[i].buckPwmOption);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable Buck regulator
 */
void test_power_setConfiguration_buckEn(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_EN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_EN_VALID_SHIFT;

        // Enable buck
        expectedPwrRsrcCfg.buckCfg[i].buckEn = TPS6522X_BUCK_ENABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_EN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckEn, actualPwrRsrcCfg.buckCfg[i].buckEn);

        // Disable buck
        expectedPwrRsrcCfg.buckCfg[i].buckEn = TPS6522X_BUCK_DISABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual BUCK_EN and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckEn, actualPwrRsrcCfg.buckCfg[i].buckEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck slew rate
 */
void test_power_setConfiguration_buckSlewRate(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t     buckSlewRate;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_SLEW_RATE validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_SLEW_RATE_VALID_SHIFT;

        buckSlewRate = TPS6522X_BUCK_SLEW_RATE_10_MV_PER_US;
        do
        {
            // Set buck slew rate
            expectedPwrRsrcCfg.buckCfg[i].buckSlewRate = buckSlewRate;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual buck slew rate and compare expected vs. actual value
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckSlewRate,
                              actualPwrRsrcCfg.buckCfg[i].buckSlewRate);
        }
        while ((buckSlewRate++) != TPS6522X_BUCK_SLEW_RATE_1_25_MV_PER_US);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when Buck voltage is below range
 */
void test_power_setConfiguration_buckVout_voltageBelowRange(void)
{
    uint8_t i = 0U;
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything less than 500 mV for a buck should result in an error
        for (voltage_mv = 499; voltage_mv != 0; voltage_mv--)
        {
            expectedPwrRsrcCfg.buckCfg[i].buckVoltage_mv = voltage_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when Buck voltage is above range
 */
void test_power_setConfiguration_buckVout_voltageAboveRange(void)
{
    uint8_t i = 0U;
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything above 3300 mV for a buck should result in an error
        for (voltage_mv = 3301; voltage_mv != 4000; voltage_mv++)
        {
            expectedPwrRsrcCfg.buckCfg[i].buckVoltage_mv = voltage_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

static void setPwrConfig_buck1Vout_test(const uint16_t voltageRangeMin_mv,
                                        const uint16_t voltageRangeMax_mv,
                                        const uint8_t  voltageStep)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Set buck validParam and BUCK_VOLTAGE_MV validParam
    // for both expected and actual power resource CFGs
    expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT;
    expectedPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].validParams =
        TPS6522X_BUCK_VOLTAGE_MV_VALID_SHIFT;
    actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
    actualPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].validParams =
        expectedPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].validParams;

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // for each voltage step...
    for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
    {
        // Set voltage
        expectedPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].buckVoltage_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual voltage and compare expected vs. actual voltage
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].buckVoltage_mv,
                          actualPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].buckVoltage_mv);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(
        initialPwrRsrcCfg,
        actualPwrRsrcCfg,
        expectedPwrRsrcCfg.validParams,
        expectedPwrRsrcCfg.buckCfg[TPS6522X_REGULATOR_BUCK1].validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck 1 output voltage
 */
void test_power_setConfiguration_buck1_buckVout(void)
{
    // Test setting BUCK1 VOUT within the range of 500 mV to 580 mV (20 mV steps)
    setPwrConfig_buck1Vout_test(500U, 580U, 20U);
    // Test setting BUCK1 VOUT within the range of 600 mV to 1095 mV (5 mV steps)
    setPwrConfig_buck1Vout_test(600U, 1095U, 5U);
    // Test setting BUCK1 VOUT within the range of 1100 mV to 1650 mV (10 mV steps)
    setPwrConfig_buck1Vout_test(1100U, 1650U, 10U);
    // Test setting BUCK1 VOUT within the range of 1660 mV to 3300 mV (20 mV steps)
    setPwrConfig_buck1Vout_test(1660U, 3300, 20U);
}

static void setPwrConfig_buck2_3_4_Vout_test(const uint16_t voltageRangeMin_mv,
                                             const uint16_t voltageRangeMax_mv,
                                             const uint8_t  voltageStep)
{
    uint8_t buckNum = 0U;
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    for (buckNum = TPS6522X_REGULATOR_BUCK2; buckNum <= TPS6522X_REGULATOR_BUCK4; buckNum++)
    {
        // Set buck validParam and BUCK_VOLTAGE_MV validParam
        // for both expected and actual power resource CFGs
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << buckNum;
        expectedPwrRsrcCfg.buckCfg[buckNum].validParams = TPS6522X_BUCK_VOLTAGE_MV_VALID_SHIFT;
        actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
        actualPwrRsrcCfg.buckCfg[buckNum].validParams = expectedPwrRsrcCfg.buckCfg[buckNum].validParams;

        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // for each voltage step...
        for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
        {
            // Set voltage
            expectedPwrRsrcCfg.buckCfg[buckNum].buckVoltage_mv = voltage_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual voltage and compare expected vs. actual voltage
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[buckNum].buckVoltage_mv,
                              actualPwrRsrcCfg.buckCfg[buckNum].buckVoltage_mv);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[buckNum].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck 2, Buck 3, Buck 4 output voltage
 */
void test_power_setConfiguration_buck2_3_4_buckVout(void)
{
    // Test setting BUCK2, BUCK3, BUCK4 VOUT within the range of 500 mV to 1150 mV (25 mV steps)
    setPwrConfig_buck2_3_4_Vout_test(500U, 1150U, 25U);
    // Test setting BUCK2, BUCK3, BUCK4 VOUT within the range of 1200 mV to 3300 mV (50 mV steps)
    setPwrConfig_buck2_3_4_Vout_test(1200U, 3300U, 50U);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set Buck VMON Threshold
 */
void test_power_setConfiguration_buckVmonThr(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t buckVmonThr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_VMON_THR validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_VMON_THR_VALID_SHIFT;

        buckVmonThr = TPS6522X_BUCK_VMON_THR_3_PCT_OR_30_MV;
        do
        {
            // Set buck VMON threshold
            expectedPwrRsrcCfg.buckCfg[i].buckVmonThr = buckVmonThr;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual buck VMON threshold and compare expected vs. actual value
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckVmonThr,
                              actualPwrRsrcCfg.buckCfg[i].buckVmonThr);
        }
        while ((buckVmonThr++) != TPS6522X_BUCK_VMON_THR_8_PCT_OR_80_MV);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure Buck Rail Group Selection
 */
void test_power_setConfiguration_buckRailGrpSel(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t      buckRailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each buck...
    for (i = 0U; i < TPS6522X_MAX_BUCK_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set buck validParam and BUCK_RAIL_GRP_SEL validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_BUCK1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.buckCfg[i].validParams = TPS6522X_BUCK_RAIL_GRP_SEL_VALID_SHIFT;

        buckRailGrpSel = TPS6522X_BUCK_RAIL_SEL_NONE;
        do
        {
            // Set buck rail group
            expectedPwrRsrcCfg.buckCfg[i].buckRailGrpSel = buckRailGrpSel;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual buck rail group and compare expected vs. actual value
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.buckCfg[i].buckRailGrpSel,
                              actualPwrRsrcCfg.buckCfg[i].buckRailGrpSel);
        }
        while ((buckRailGrpSel++) != TPS6522X_BUCK_RAIL_SEL_OTHER);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.buckCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable LDO discharge
 */
void test_power_setConfiguration_ldoDischargeEnableDisable(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_DISCHARGE_EN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_DISCHARGE_EN_VALID_SHIFT;

        // Enable LDO discharge
        expectedPwrRsrcCfg.ldoCfg[i].ldoDischargeEn = TPS6522X_LDO_DISCHARGE_ENABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO discharge enable and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoDischargeEn,
                          actualPwrRsrcCfg.ldoCfg[i].ldoDischargeEn);

        // Disable LDO discharge
        expectedPwrRsrcCfg.ldoCfg[i].ldoDischargeEn = TPS6522X_LDO_DISCHARGE_DISABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO discharge enable and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoDischargeEn,
                          actualPwrRsrcCfg.ldoCfg[i].ldoDischargeEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable LDO VMON
 */
void test_power_setConfiguration_ldoVmonEnableDisable(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VMON_EN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_VMON_EN_VALID_SHIFT;

        // Enable LDO VMON
        expectedPwrRsrcCfg.ldoCfg[i].ldoVmonEn = TPS6522X_LDO_VMON_ENABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO VMON enable and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoVmonEn, actualPwrRsrcCfg.ldoCfg[i].ldoVmonEn);

        // Disable LDO VMON
        expectedPwrRsrcCfg.ldoCfg[i].ldoVmonEn = TPS6522X_LDO_VMON_DISABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO VMON enable and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoVmonEn, actualPwrRsrcCfg.ldoCfg[i].ldoVmonEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable LDO
 */
void test_power_setConfiguration_ldoEnableDisable(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_EN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_EN_VALID_SHIFT;

        // Enable LDO
        expectedPwrRsrcCfg.ldoCfg[i].ldoEn = TPS6522X_LDO_ENABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO enable and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoEn, actualPwrRsrcCfg.ldoCfg[i].ldoEn);

        // Disable LDO
        expectedPwrRsrcCfg.ldoCfg[i].ldoEn = TPS6522X_LDO_DISABLE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO enable and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoEn, actualPwrRsrcCfg.ldoCfg[i].ldoEn);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure LDO to operate in
 *                                           Bypass Mode or LDO Mode
 */
void test_power_setConfiguration_ldoBypassConfig(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_BYP_CONFIG validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_MODE_VALID_SHIFT;

        // Set the LDO Bypass Config to LDO Mode
        expectedPwrRsrcCfg.ldoCfg[i].ldoMode = TPS6522X_LDO_BYP_CONFIG_LDO_MODE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO Bypass Config and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoMode, actualPwrRsrcCfg.ldoCfg[i].ldoMode);

        // Set the LDO Bypass Config to Bypass Mode
        expectedPwrRsrcCfg.ldoCfg[i].ldoMode = TPS6522X_LDO_BYP_CONFIG_BYPASS_MODE;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual LDO Bypass Config and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoMode, actualPwrRsrcCfg.ldoCfg[i].ldoMode);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO voltage is below range
 */
void test_power_setConfiguration_ldo1Vout_voltageBelowRange(void)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set LDO validParam and LDO_VOLTAGE_MV validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT;
    expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams =
        TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT;

    // Setting voltage_mv to anything less than 1200 mV for LDO1 should result in an error
    for (voltage_mv = 1199U; voltage_mv != 0; voltage_mv--)
    {
        expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO1 voltage is above range
 */
void test_power_setConfiguration_ldo1Vout_voltageAboveRange(void)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set LDO validParam and LDO_VOLTAGE_MV validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT;
    expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams =
        TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT;

    // Setting voltage_mv to anything above 3300 mV for LDO1 should result in an error
    for (voltage_mv = 3301U; voltage_mv <= 3500U; voltage_mv++)
    {
        expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO2, LDO3 voltage is below range
 */
void test_power_setConfiguration_ldo2_3_Vout_voltageBelowRange(void)
{
    uint8_t i = 0U;
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For LDO2 and LDO3...
    for (i = TPS6522X_REGULATOR_LDO2; i < TPS6522X_REGULATOR_LDO3; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything less than 600 mV for LDO2, LDO3 should result in an error
        for (voltage_mv = 599U; voltage_mv != 0U; voltage_mv--)
        {
            expectedPwrRsrcCfg.ldoCfg[i].ldoVoltage_mv = voltage_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when LDO2, LDO3 voltage is above range
 */
void test_power_setConfiguration_ldo2_3_Vout_voltageAboveRange(void)
{
    uint8_t i = 0U;
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For LDO2 and LDO3...
    for (i = TPS6522X_REGULATOR_LDO2; i < TPS6522X_REGULATOR_LDO3; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VOLTAGE_MV validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT;

        // Setting voltage_mv to anything above 3400 mV for LDO2, LDO3 should result in an error
        for (voltage_mv = 3401U;
             voltage_mv <= 3501U;
             voltage_mv++)
        {
            expectedPwrRsrcCfg.ldoCfg[i].ldoVoltage_mv = voltage_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

static void setPwrConfig_ldo1Vout_test(const uint16_t voltageRangeMin_mv,
                                       const uint16_t voltageRangeMax_mv,
                                       const uint8_t  voltageStep)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Set LDO validParam and LDO_VOLTAGE_MV validParam
    // for both expected and actual power resource CFGs
    expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT;
    expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams =
        TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT;
    actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
    actualPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams =
        expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams;

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if (voltageStep == 0U)
    {
        // Set voltage
        expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltageRangeMin_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual voltage and compare expected vs. actual voltage
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv,
                          actualPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv);
    }
    else
    {
        for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
        {
            // Set voltage
            expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv = voltage_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual voltage and compare expected vs. actual voltage
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv,
                              actualPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].ldoVoltage_mv);
        }
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.ldoCfg[TPS6522X_REGULATOR_LDO1].validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg:  Test whether API can set LDO1 output voltage
 */
void test_power_setConfiguration_ldo1_ldoVout(void)
{
    setPwrConfig_ldo1Vout_test(1200U, 1200U, 0U);
    setPwrConfig_ldo1Vout_test(1250U, 3250U, 50U);
    setPwrConfig_ldo1Vout_test(3300U, 3300U, 0U);
}

static void setPwrConfig_ldo2_3_Vout_test(const uint16_t voltageRangeMin_mv,
                                          const uint16_t voltageRangeMax_mv,
                                          const uint8_t  voltageStep)
{
    uint8_t ldoNum = 0;
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For LDO2 and LDO3...
    for (ldoNum = TPS6522X_REGULATOR_LDO2; ldoNum <= TPS6522X_REGULATOR_LDO3; ldoNum++)
    {
        // Set LDO validParam and LDO_VOLTAGE_MV validParam
        // for both expected and actual power resource CFGs
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << ldoNum;
        expectedPwrRsrcCfg.ldoCfg[ldoNum].validParams = TPS6522X_LDO_VOLTAGE_MV_VALID_SHIFT;
        actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
        actualPwrRsrcCfg.ldoCfg[ldoNum].validParams = expectedPwrRsrcCfg.ldoCfg[ldoNum].validParams;

        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if (voltageStep == 0U)
        {
            // Set voltage
            expectedPwrRsrcCfg.ldoCfg[ldoNum].ldoVoltage_mv = voltageRangeMin_mv;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Get actual voltage and compare expected vs. actual voltage
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[ldoNum].ldoVoltage_mv,
                              actualPwrRsrcCfg.ldoCfg[ldoNum].ldoVoltage_mv);
        }
        else
        {
            for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
            {
                // Set voltage
                expectedPwrRsrcCfg.ldoCfg[ldoNum].ldoVoltage_mv = voltage_mv;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Get actual voltage and compare expected vs. actual voltage
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[ldoNum].ldoVoltage_mv,
                                  actualPwrRsrcCfg.ldoCfg[ldoNum].ldoVoltage_mv);
            }
        }

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[ldoNum].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg:  Test whether API can set LDO1 output voltage
 */
void test_power_setConfiguration_ldo2_3_ldoVout(void)
{
    setPwrConfig_ldo2_3_Vout_test(600U, 3350U, 50U);
    setPwrConfig_ldo2_3_Vout_test(3400U, 3400U, 0U);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set LDO VMON threshold
 */
void test_power_setConfiguration_ldoVmonThr(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t ldoVmonThr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_VMON_THR validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_VMON_THR_VALID_SHIFT;

        ldoVmonThr = TPS6522X_LDO_VMON_THR_3_PCT;
        do
        {
            // Set LDO VMON threshold
            expectedPwrRsrcCfg.ldoCfg[i].ldoVmonThr = ldoVmonThr;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual LDO VMON threshold and compare expected vs. actual value
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoVmonThr,
                              actualPwrRsrcCfg.ldoCfg[i].ldoVmonThr);
        }
        while ((ldoVmonThr++) != TPS6522X_LDO_VMON_THR_8_PCT);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure LDO Rail Group Selection
 */
void test_power_setConfiguration_ldoRailGrpSel(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t ldoRailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // For each LDO...
    for (i = 0U; i < TPS6522X_MAX_LDO_NUM; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set LDO validParam and LDO_RAIL_GRP_SEL validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_LDO1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.ldoCfg[i].validParams = TPS6522X_LDO_RAIL_GRP_SEL_VALID_SHIFT;

        ldoRailGrpSel = TPS6522X_LDO_RAIL_SEL_NONE;
        do
        {
            // Set LDO rail group
            expectedPwrRsrcCfg.ldoCfg[i].ldoRailGrpSel = ldoRailGrpSel;
            status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

            // Read actual LDO rail group and compare expected vs. actual value
            status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
            TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.ldoCfg[i].ldoRailGrpSel,
                              actualPwrRsrcCfg.ldoCfg[i].ldoRailGrpSel);
        }
        while ((ldoRailGrpSel++) != TPS6522X_LDO_RAIL_SEL_OTHER);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.ldoCfg[i].validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VMON Deglitch Selection
 */
void test_power_setConfiguration_vmonDeglitch(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t vmonDeglitchSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VMON_DEGLITCH_SEL validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VMON_DEGLITCH_SEL_VALID_SHIFT;

    vmonDeglitchSel = TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_4_US;
    do
    {
        // Set VMON deglitch select
        expectedPwrRsrcCfg.vccaVmonCfg.vmonDeglitchSel = vmonDeglitchSel;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual VMON deglitch select and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmonDeglitchSel,
                          actualPwrRsrcCfg.vccaVmonCfg.vmonDeglitchSel);
    }
    while ((vmonDeglitchSel++) != TPS6522X_VMON_DEGLITCH_SEL_BUCK_LDO_VMON_4_US_VCCA_20_US);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can enable/disable VMON1, VMON2, and VCCA_VMON
 */
void test_power_setConfiguration_VMON1_2_VCCA_VMON_EnableDisable(void)
{
    uint8_t vmonNum = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    vmonNum = TPS6522X_VOLTAGE_MONITOR_VCCA_VMON;
    do
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set VMON validParam and VMON_EN validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_VMON1_VALID_SHIFT << vmonNum;
        expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VMON2_EN_VALID_SHIFT << vmonNum;

        switch (expectedPwrRsrcCfg.vccaVmonCfg.validParams)
        {
            case TPS6522X_VMON2_EN_VALID_SHIFT:
                // Enable VMON 2
                expectedPwrRsrcCfg.vccaVmonCfg.vmon2En = TPS6522X_VMON2_ENABLE;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon2En,
                                  actualPwrRsrcCfg.vccaVmonCfg.vmon2En);

                // Disable VMON2
                expectedPwrRsrcCfg.vccaVmonCfg.vmon2En = TPS6522X_VMON2_DISABLE;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon2En,
                                  actualPwrRsrcCfg.vccaVmonCfg.vmon2En);

                break;
            case TPS6522X_VMON1_EN_VALID_SHIFT:
                // Enable VMON 1
                expectedPwrRsrcCfg.vccaVmonCfg.vmon1En = TPS6522X_VMON1_ENABLE;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon1En,
                                  actualPwrRsrcCfg.vccaVmonCfg.vmon1En);

                // Disable VMON1
                expectedPwrRsrcCfg.vccaVmonCfg.vmon1En = TPS6522X_VMON1_DISABLE;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon1En,
                                  actualPwrRsrcCfg.vccaVmonCfg.vmon1En);

                break;
            case TPS6522X_VCCA_VMON_EN_VALID_SHIFT:
                // Enable VCCA_VMON
                expectedPwrRsrcCfg.vccaVmonCfg.vccaVmonEn = TPS6522X_VCCA_VMON_ENABLE;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vccaVmonEn,
                                  actualPwrRsrcCfg.vccaVmonCfg.vccaVmonEn);

                // Disable VCCA_VMON
                expectedPwrRsrcCfg.vccaVmonCfg.vccaVmonEn = TPS6522X_VCCA_VMON_DISABLE;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Read actual VMON enable and compare expected vs. actual value
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vccaVmonEn,
                                  actualPwrRsrcCfg.vccaVmonCfg.vccaVmonEn);

                break;
        };

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.vccaVmonCfg.validParams);
    }
    while ((vmonNum--) != TPS6522X_VOLTAGE_MONITOR_VMON1);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VCCA_VMON PG Level
 */
void test_power_setConfiguration_vccaPgLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VCCA_PG_LEVEL validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VCCA_PG_LEVEL_VALID_SHIFT;

    // Set the VCCA PG level to 3.3V
    expectedPwrRsrcCfg.vccaVmonCfg.vccaPgLevel = TPS6522X_VCCA_PG_LEVEL_3_3_V;
    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Read actual VCCA PG level and compare expected vs. actual value
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vccaPgLevel,
                      actualPwrRsrcCfg.vccaVmonCfg.vccaPgLevel);

    // Set the VCCA PG level to 5.0V
    expectedPwrRsrcCfg.vccaVmonCfg.vccaPgLevel = TPS6522X_VCCA_PG_LEVEL_5_0_V;
    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Read actual VCCA PG level and compare expected vs. actual value
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vccaPgLevel,
                      actualPwrRsrcCfg.vccaVmonCfg.vccaPgLevel);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VCCA_VMON Threshold
 */
void test_power_setConfiguration_vccaVmonThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t vccaVmonThr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VCCA_VMON_THR validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VCCA_VMON_THR_VALID_SHIFT;

    vccaVmonThr = TPS6522X_VCCA_VMON_THR_3_PCT;
    do
    {
        // Set VCCA_VMON powergood high/low threshold level
        expectedPwrRsrcCfg.vccaVmonCfg.vccaVmonThr = vccaVmonThr;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual VCCA_VMON threshold and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vccaVmonThr,
                          actualPwrRsrcCfg.vccaVmonCfg.vccaVmonThr);
    }
    while ((vccaVmonThr++) != TPS6522X_VCCA_VMON_THR_10_PCT);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VCCA_VMON Rail Group Selection
 */
void test_power_setConfiguration_vccaRailGrpSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t vccaRailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VCCA_VMON validParam and VCCA_RAIL_GRP_SEL validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_VCCA_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VCCA_RAIL_GRP_SEL_VALID_SHIFT;

    vccaRailGrpSel = TPS6522X_VCCA_RAIL_SEL_NONE;
    do
    {
        // Set VCCA rail group
        expectedPwrRsrcCfg.vccaVmonCfg.vccaRailGrpSel = vccaRailGrpSel;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Read actual VCCA rail group and compare expected vs. actual value
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vccaRailGrpSel,
                          actualPwrRsrcCfg.vccaVmonCfg.vccaRailGrpSel);
    }
    while ((vccaRailGrpSel++) != TPS6522X_VCCA_RAIL_SEL_OTHER);

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when VMON1 voltage is below range
 *                                           and above range
 */
void test_power_setConfiguration_vmon1PgSet_voltageOutOfRange(void)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VMON1 validParam and VMON1_PG_LEVEL_MV validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_VMON1_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VMON1_PG_LEVEL_MV_VALID_SHIFT;

    // Setting voltage_mv to anything less than 500 mV for VMON1 should result in an error
    for (voltage_mv = 499U; voltage_mv != 0U; voltage_mv--)
    {
        expectedPwrRsrcCfg.vccaVmonCfg.vmon1PgLevel_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Setting voltage_mv to anything above 3340 mV for VMON1 should result in an error
    for (voltage_mv = 3341U; voltage_mv <= 3441U; voltage_mv++)
    {
        expectedPwrRsrcCfg.vccaVmonCfg.vmon1PgLevel_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test API error handing for when VMON2 voltage is below range
 *                                           and above range
 */
void test_power_setConfiguration_vmon2PgSet_voltageOutOfRange(void)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Set VMON2 validParam and VMON2_PG_LEVEL_MV validParam
    expectedPwrRsrcCfg.validParams = TPS6522X_VMON2_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = TPS6522X_VMON2_PG_LEVEL_MV_VALID_SHIFT;

    // Setting voltage_mv to anything less than 500 mV for VMON2 should result in an error
    for (voltage_mv = 499U; voltage_mv != 0; voltage_mv--)
    {
        expectedPwrRsrcCfg.vccaVmonCfg.vmon2PgLevel_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Setting voltage_mv to anything above 3300 mV for VMON2 should result in an error
    for (voltage_mv = 3301U;
         voltage_mv <= 3401U;
         voltage_mv++)
    {
        expectedPwrRsrcCfg.vccaVmonCfg.vmon2PgLevel_mv = voltage_mv;
        status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_VOLTAGE, status);
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

static void setPwrConfig_vmon1_2_Vout_test(const uint8_t  vmonNum,
                                           const uint16_t voltageRangeMin_mv,
                                           const uint16_t voltageRangeMax_mv,
                                           const uint8_t  voltageStep)
{
    uint16_t voltage_mv = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;

    // Initialize power resource CFG
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withNoValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    // Set VMONx validParam and VMONx_PG_LEVEL_MV validParam
    // for both expected and actual power resource CFGs
    expectedPwrRsrcCfg.validParams = (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                         TPS6522X_VMON1_VALID_SHIFT :
                                         TPS6522X_VMON2_VALID_SHIFT;
    expectedPwrRsrcCfg.vccaVmonCfg.validParams = (vmonNum == TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                                            TPS6522X_VMON1_PG_LEVEL_MV_VALID_SHIFT :
                                                            TPS6522X_VMON2_PG_LEVEL_MV_VALID_SHIFT;
    actualPwrRsrcCfg.validParams = expectedPwrRsrcCfg.validParams;
    actualPwrRsrcCfg.vccaVmonCfg.validParams = expectedPwrRsrcCfg.vccaVmonCfg.validParams;

    // Capture current configuration state of power resources for later comparison
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // for each voltage step...
    for (voltage_mv = voltageRangeMin_mv; voltage_mv <= voltageRangeMax_mv; voltage_mv += voltageStep)
    {
        switch (vmonNum)
        {
            case TPS6522X_VOLTAGE_MONITOR_VMON1:
                // Set voltage
                expectedPwrRsrcCfg.vccaVmonCfg.vmon1PgLevel_mv = voltage_mv;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Get actual voltage and compare expected vs. actual voltage
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon1PgLevel_mv,
                                  actualPwrRsrcCfg.vccaVmonCfg.vmon1PgLevel_mv);

                break;
            case TPS6522X_VOLTAGE_MONITOR_VMON2:
                // Set voltage
                expectedPwrRsrcCfg.vccaVmonCfg.vmon2PgLevel_mv = voltage_mv;
                status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                // Get actual voltage and compare expected vs. actual voltage
                status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon2PgLevel_mv,
                                  actualPwrRsrcCfg.vccaVmonCfg.vmon2PgLevel_mv);

                break;
        }
    }

    // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                     actualPwrRsrcCfg,
                                     expectedPwrRsrcCfg.validParams,
                                     expectedPwrRsrcCfg.vccaVmonCfg.validParams);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON1 PG Level
 */
void test_power_setConfiguration_vmon1PgSet(void)
{
    const uint8_t vmonNum = TPS6522X_VOLTAGE_MONITOR_VMON1;

    // Test setting VMON1 PG_SET within the range of 500 mV to 580 mV (20 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum, 500U, 580U, 20U);
    // Test setting VMON1 PG_SET within the range of 600 mV to 1095 mV (5 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum, 600U, 1095U, 5U);
    // Test setting VMON1 PG_SET within the range of 1100 mV to 1650 mV (10 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum, 1100U, 1650U, 10U);
    // Test setting VMON1 PG_SET within the range of 1660 mV to 3340 mV (20 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum, 1660U, 3340U, 20U);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON2 PG Level
 */
void test_power_setConfiguration_vmon2PgSet(void)
{
    const uint8_t vmonNum = TPS6522X_VOLTAGE_MONITOR_VMON2;

    // Test setting VMON2 PG_SET within the range of 500 mV to 1150 mV (25 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum, 500U, 1150U, 25U);
    // Test setting VMON2 PG_SET within the range of 1200 mV to 3300 mV (50 mV steps)
    setPwrConfig_vmon1_2_Vout_test(vmonNum, 1200U, 3300U, 50U);
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can set VMON1 and VMON2 Threshold
 */
void test_power_setConfiguration_vmon1_2_Thr(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t         vmon1Thr;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    for (i = TPS6522X_VOLTAGE_MONITOR_VMON1; i <= TPS6522X_VOLTAGE_MONITOR_VMON2; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set VMONx validParam and VMONx_THR validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_VMON1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.vccaVmonCfg.validParams = (i == TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                                                TPS6522X_VMON1_THR_VALID_SHIFT :
                                                                TPS6522X_VMON2_THR_VALID_SHIFT;

        vmon1Thr = TPS6522X_VMON1_THR_3_PCT_OR_30_MV;
        do
        {
            switch (i)
            {
                case TPS6522X_VOLTAGE_MONITOR_VMON1:
                    // Set VMON1 Powergood Threshold
                    expectedPwrRsrcCfg.vccaVmonCfg.vmon1Thr = vmon1Thr;
                    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON1 Powergood Threshold and compare expected vs. actual value
                    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon1Thr,
                                      actualPwrRsrcCfg.vccaVmonCfg.vmon1Thr);

                    break;
                case TPS6522X_VOLTAGE_MONITOR_VMON2:
                    // Set VMON2 Powergood Threshold
                    expectedPwrRsrcCfg.vccaVmonCfg.vmon2Thr = (uint8_t)vmon1Thr;
                    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON2 Powergood Threshold and compare expected vs. actual value
                    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon2Thr,
                                      actualPwrRsrcCfg.vccaVmonCfg.vmon2Thr);

                    break;
            }
        }
        while ((vmon1Thr++) != TPS6522X_VMON1_THR_8_PCT_OR_80_MV);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.vccaVmonCfg.validParams);
    }
}

/**
 *  \brief  tps6522xSetPwrRsrcCfg: Test whether API can configure VMON1 and VMON2 Rail Group Selection
 */
void test_power_setConfiguration_vmon1_2_RailGrpSel(void)
{
    uint8_t i = 0U;
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xPwrRsrcCfg_t expectedPwrRsrcCfg, actualPwrRsrcCfg, initialPwrRsrcCfg;
    uint8_t vmon1RailGrpSel;

    // Initialize power resource CFGs
    resetBurtonPwrCfg_withNoValidParams(&expectedPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&actualPwrRsrcCfg);
    resetBurtonPwrCfg_withAllValidParams(&initialPwrRsrcCfg);

    for (i = TPS6522X_VOLTAGE_MONITOR_VMON1; i <= TPS6522X_VOLTAGE_MONITOR_VMON2; i++)
    {
        // Capture current configuration state of power resources for later comparison
        status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &initialPwrRsrcCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Set VMONx validParam and VMONx_RAIL_GRP_SEL validParam
        expectedPwrRsrcCfg.validParams = TPS6522X_VMON1_VALID_SHIFT << i;
        expectedPwrRsrcCfg.vccaVmonCfg.validParams = (i == TPS6522X_VOLTAGE_MONITOR_VMON1) ?
                                                                TPS6522X_VMON1_RAIL_GRP_SEL_VALID_SHIFT :
                                                                TPS6522X_VMON2_RAIL_GRP_SEL_VALID_SHIFT;

        vmon1RailGrpSel = TPS6522X_VMON1_RAIL_SEL_NONE;
        do
        {
            switch (i)
            {
                case TPS6522X_VOLTAGE_MONITOR_VMON1:
                    // Set VMON1 rail group
                    expectedPwrRsrcCfg.vccaVmonCfg.vmon1RailGrpSel = vmon1RailGrpSel;
                    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON1 rail group and compare expected vs. actual value
                    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon1RailGrpSel,
                                      actualPwrRsrcCfg.vccaVmonCfg.vmon1RailGrpSel);

                    break;
                case TPS6522X_VOLTAGE_MONITOR_VMON2:
                    // Set VMON2 rail group
                    expectedPwrRsrcCfg.vccaVmonCfg.vmon2RailGrpSel =
                        (uint8_t)vmon1RailGrpSel;
                    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, expectedPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                    // Read actual VMON2 rail group and compare expected vs. actual value
                    status = tps6522xGetPwrRsrcCfg(&pmicCoreHandle, &actualPwrRsrcCfg);
                    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                    TEST_ASSERT_EQUAL(expectedPwrRsrcCfg.vccaVmonCfg.vmon2RailGrpSel,
                                      actualPwrRsrcCfg.vccaVmonCfg.vmon2RailGrpSel);

                    break;
            }
        }
        while ((vmon1RailGrpSel++) != TPS6522X_VCCA_RAIL_SEL_OTHER);

        // Ensure that only the target bit field has been changed and no other registers/bit fields are modified
        comparePwrRsrcCfg_ignoreBitField(initialPwrRsrcCfg,
                                         actualPwrRsrcCfg,
                                         expectedPwrRsrcCfg.validParams,
                                         expectedPwrRsrcCfg.vccaVmonCfg.validParams);
    }
}

/**
 *  \brief  tps6522xGetPwrRsrcStat: Test API error handling for when parameters are NULL
 */
void test_power_getPwrRsrcStat_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool underOverVoltageStat = false;

    // Pass NULL handle into the API and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcStat(NULL, TPS6522X_BUCK1_UVOV_STAT, &underOverVoltageStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL variable into the API and compare expected vs. actual return code
    status = tps6522xGetPwrRsrcStat(&pmicCoreHandle, TPS6522X_BUCK1_UVOV_STAT, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrRsrcStat: Test whether API can read all Buck, LDO, and VCCA_VMON/VMONx status
 */
void test_power_getPwrRsrcStat_allPwrRsrc(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcUVOVStat = TPS6522X_BUCK1_UVOV_STAT;
    bool underOverVoltageStat = false;

    do
    {
        // Get power resource UVOV status
        status = tps6522xGetPwrRsrcStat(&pmicCoreHandle, pwrRsrcUVOVStat, &underOverVoltageStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Iterate until all power resource UVOV statuses are read
    }
    while ((pwrRsrcUVOVStat++) != TPS6522X_VCCA_UVOV_STAT);
}

/**
 *  \brief   tps6522xGetPwrRsrcStat: Test whether API can detect a UVOV status on VMON1 and VMON2.
 *
 *  \note    This test assumes that the VMONs are not connected to any voltage sources
 */
void test_power_getPwrRsrcStat_vmon1_2_UVOVStatDetection(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pwrRsrcUVOVStat = TPS6522X_VMON1_UVOV_STAT;
    bool underOverVoltageStat = false;
    tps6522xPwrRsrcCfg_t pwrRsrcCfg;

    // Initialize pwrRsrcCfg with VMON1 and VMON2 configuration
    resetBurtonPwrCfg_withNoValidParams(&pwrRsrcCfg);
    pwrRsrcCfg.validParams = TPS6522X_VMON1_VALID_SHIFT | TPS6522X_VMON2_VALID_SHIFT;
    pwrRsrcCfg.vccaVmonCfg.validParams =
        TPS6522X_VMON1_EN_VALID | TPS6522X_VMON2_EN_VALID |
        TPS6522X_VMON1_PG_LEVEL_MV_VALID | TPS6522X_VMON1_RAIL_GRP_SEL_VALID |
        TPS6522X_VMON2_PG_LEVEL_MV_VALID | TPS6522X_VMON2_RAIL_GRP_SEL_VALID;
    pwrRsrcCfg.vccaVmonCfg.vmon1En = TPS6522X_VMON1_ENABLE;
    pwrRsrcCfg.vccaVmonCfg.vmon2En = TPS6522X_VMON2_ENABLE;
    pwrRsrcCfg.vccaVmonCfg.vmon1PgLevel_mv = 3000;
    pwrRsrcCfg.vccaVmonCfg.vmon2PgLevel_mv = 3000;
    pwrRsrcCfg.vccaVmonCfg.vmon1RailGrpSel = TPS6522X_VMON1_RAIL_SEL_MCU;
    pwrRsrcCfg.vccaVmonCfg.vmon2RailGrpSel = TPS6522X_VMON2_RAIL_SEL_MCU;

    // Set power resource configuration
    status = tps6522xSetPwrRsrcCfg(&pmicCoreHandle, pwrRsrcCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Wait a short period of time so VMON1 and VMON2 could detect UV
    delayTimeInMs(&tHandle, 500);

    do
    {
        // Get UVOV status
        status = tps6522xGetPwrRsrcStat(&pmicCoreHandle, pwrRsrcUVOVStat, &underOverVoltageStat);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual UVOV status
        TEST_ASSERT_EQUAL(true, underOverVoltageStat);
    }
    while ((pwrRsrcUVOVStat++) != TPS6522X_VMON2_UVOV_STAT);
}

static void resetThermalStat_withAllValidParams(tps6522xThermalStat_t *thermalStat)
{
    thermalStat->validParams =
        PMIC_THERMAL_STAT_WARN_VALID | PMIC_THERMAL_STAT_ORD_SHTDWN_VALID | PMIC_THERMAL_STAT_IMM_SHTDWN_VALID;
    thermalStat->twarnStat = false;
    thermalStat->tsdOrdStat = false;
    thermalStat->tsdImmStat = false;
}

/**
 *  \brief  tps6522xGetPwrThermalStat: Test API error handling for when parameters are NULL
 */
void test_power_getPwrThermalStat_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalStat_t thermalStat;

    // Initialize thermalStat
    resetThermalStat_withAllValidParams(&thermalStat);

    // Pass NULL PMIC handle into the API and compare expected vs. actual return code
    status = tps6522xGetThermalStat(NULL, &thermalStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL thermal status struct into the API and compare expected vs. actual return code
    status = tps6522xGetThermalStat(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrThermalStat: Test API error handling for when there are no valid parameters
 */
void test_power_getPwrThermalStat_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalStat_t thermalStat = {.validParams = 0U};

    // Pass struct with validParams into API and compare expected vs. actual return code
    status = tps6522xGetThermalStat(&pmicCoreHandle, &thermalStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  tps6522xGetPwrThermalStat: Test whether API can get the thermal warning, orderly, and
 *                                               immediate thermal statuses
 */
void test_power_getPwrThermalStat_getAllStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalStat_t thermalStat;

    // Initialize thermalStat
    resetThermalStat_withAllValidParams(&thermalStat);

    // Get thermal status
    status = tps6522xGetThermalStat(&pmicCoreHandle, &thermalStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

static void resetThermalCfg_withAllValidParams(tps6522xThermalCfg_t *pThermalCfg)
{
    pThermalCfg->validParams =
        TPS6522X_TWARN_LEVEL_VALID_SHIFT | TPS6522X_TSD_ORD_LEVEL_VALID_SHIFT;
    pThermalCfg->tsdOrdLvl = TPS6522X_TSD_ORD_LVL_140C;
    pThermalCfg->twarnLvl = TPS6522X_TWARN_LVL_130C;
}

static void resetThermalCfg_withNoValidParams(tps6522xThermalCfg_t *pThermalCfg)
{
    pThermalCfg->validParams = 0U;
    pThermalCfg->tsdOrdLvl = TPS6522X_TSD_ORD_LVL_140C;
    pThermalCfg->twarnLvl = TPS6522X_TWARN_LVL_130C;
}

/**
 *  \brief  tps6522xGetThermalCfg: Test API error handling for when parameters are NULL
 */
void test_power_getThermalCfg_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg
    resetThermalCfg_withAllValidParams(&thermalCfg);

    // Pass NULL PMIC handle into the API and compare expected vs. actual return code
    status = tps6522xGetThermalCfg(NULL, &thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    // Pass NULL struct into the API and compare expected vs. actual return code
    status = tps6522xGetThermalCfg(&pmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xGetThermalCfg: Test API error handling for when there are no valid parameters
 */
void test_power_getThermalCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg with no valid Parameters
    resetThermalCfg_withNoValidParams(&thermalCfg);

    // Pass thermalCfg into the API and compare expected vs. actual return code
    status = tps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  tps6522xSetThermalCfg: Test API error handling for when parameters are NULL
 */
void test_power_setThermalCfg_nullParam(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg
    resetThermalCfg_withAllValidParams(&thermalCfg);

    // Pass NULL PMIC handle into the API and compare expected vs. actual return code
    status = tps6522xSetThermalCfg(NULL, thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);
}

/**
 *  \brief  tps6522xSetThermalCfg: Test API error handling for when there are no valid parameters
 */
void test_power_setThermalCfg_noValidParams(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalCfg_t thermalCfg;

    // Initialize thermalCfg with no valid Parameters
    resetThermalCfg_withNoValidParams(&thermalCfg);

    // Pass thermalCfg into the API and compare expected vs. actual return code
    status = tps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
}

/**
 *  \brief  tps6522xSetThermalCfg: Test whether API can configure the orderly thermal shutdown level
 */
void test_power_setThermalCfg_TsdOrdLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalCfg_t thermalCfg_expected, thermalCfg_actual;

    // Initialize thermal CFGs
    resetThermalCfg_withNoValidParams(&thermalCfg_expected);
    resetThermalCfg_withAllValidParams(&thermalCfg_actual);
    thermalCfg_expected.validParams = TPS6522X_TSD_ORD_LEVEL_VALID_SHIFT;

    // Set TSD_ORD_LEVEL to be 145C
    thermalCfg_expected.tsdOrdLvl = TPS6522X_TSD_ORD_LVL_145C;
    status = tps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TSD_ORD_LEVEL
    status = tps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.tsdOrdLvl, thermalCfg_actual.tsdOrdLvl);

    // Set TSD_ORD_LEVEL to be 140C
    thermalCfg_expected.tsdOrdLvl = TPS6522X_TSD_ORD_LVL_140C;
    status = tps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TSD_ORD_LEVEL
    status = tps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.tsdOrdLvl, thermalCfg_actual.tsdOrdLvl);
}

/**
 *  \brief  tps6522xSetThermalCfg: Test whether API can configure the thermal warning level
 */
void test_power_setThermalCfg_TwarnLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    tps6522xThermalCfg_t thermalCfg_expected, thermalCfg_actual;

    // Initialize thermal CFGs
    resetThermalCfg_withNoValidParams(&thermalCfg_expected);
    resetThermalCfg_withAllValidParams(&thermalCfg_actual);
    thermalCfg_expected.validParams = TPS6522X_TWARN_LEVEL_VALID_SHIFT;

    // Set TWARN_LEVEL to be 140C
    thermalCfg_expected.twarnLvl = TPS6522X_TWARN_LVL_140C;
    status = tps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TWARN_LEVEL
    status = tps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(thermalCfg_expected.twarnLvl, thermalCfg_actual.twarnLvl);

    // Set TWARN_LEVEL to be 130C
    thermalCfg_expected.twarnLvl = TPS6522X_TWARN_LVL_130C;
    status = tps6522xSetThermalCfg(&pmicCoreHandle, thermalCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual thermal configuration and compare expected vs. actual TWARN_LEVEL
    status = tps6522xGetThermalCfg(&pmicCoreHandle, &thermalCfg_actual);
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
    disablePmicPowerResources(pmicCoreHandle);
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
}
