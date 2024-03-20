/**
 * \file core_test.c
 * \author John Bui (j-bui1@ti.com)
 * \brief Source file for PMIC Core Unity testing, containing source code for Core tests
 * \version 1.0
 * \date 2023-12-29
 *
 * \copyright Copyright (c) 2023
 *
 */
// Standard include
#include <stdio.h>

// PMIC driver
#include "pmic.h"

// Test specific include
#include "core_test.h"

// Unit testing
#include "unity.h"

#define RUN_CORE_TESTS()    RUN_TEST(test_core_setRecoveryCntCfg_thrVal);                   \
                            RUN_TEST(test_core_setRecoveryCntCfg_clrCnt);                   \
                            RUN_TEST(test_core_setScratchPadValue_allScratchPads);          \
                            RUN_TEST(test_core_setUserSpareValue_allUserSpares);            \
                            RUN_TEST(test_core_setCommonCtrlCfg_spreadSpectrumEn);          \
                            RUN_TEST(test_core_setCommonCtrlCfg_skipEepromDefaultLoadEn);   \
                            RUN_TEST(test_core_setCommonCtrlCfg_eepromDefaultLoad);         \
                            RUN_TEST(test_core_setCommonCtrlCfg_enDrv);                     \
                            RUN_TEST(test_core_setCommonCtrlCfg_regLock);                   \
                            RUN_TEST(test_core_setCommonCtrlCfg_spreadSpectrumDepth);       \
                            RUN_TEST(test_core_setMiscCtrlCfg_amuxOutRefOutEn);             \
                            RUN_TEST(test_core_setMiscCtrlCfg_clkMonEn);                    \
                            RUN_TEST(test_core_setMiscCtrlCfg_syncClkOutFreqSel);           \
                            RUN_TEST(test_core_setMiscCtrlCfg_extClkSel);                   \
                            RUN_TEST(test_core_setMiscCtrlCfg_syncClkInFreq);               \
                            RUN_TEST(test_core_setMiscCtrlCfg_nRstOutSocSignal);            \
                            RUN_TEST(test_core_setMiscCtrlCfg_nRstOutSignal);               \
                            RUN_TEST(test_core_setBatteryCtrlCfg_invalidDevice);            \
                            RUN_TEST(test_core_getBatteryCtrlCfg_invalidDevice);            \
                            RUN_TEST(test_core_getCommonCtrlStat_spmiLpmStat);              \
                            RUN_TEST(test_core_getCommonCtrlStat_forceEnDrvLowStat);        \
                            RUN_TEST(test_core_getCommonCtrlStat_bbEndOfChargeIndication);  \
                            RUN_TEST(test_core_getCommonCtrlStat_regLockStat);              \
                            RUN_TEST(test_core_getCommonCtrlStat_extClkValidity);           \
                            RUN_TEST(test_core_getCommonCtrlStat_startupPin);               \
                            RUN_TEST(test_core_getCommonCtrlStat_enDrvPin);                 \
                            RUN_TEST(test_core_getCommonCtrlStat_nRstOutSocPin);            \
                            RUN_TEST(test_core_getCommonCtrlStat_nRstOutPin);               \
                            RUN_TEST(test_core_getCommonCtrlStat_nIntPin);                  \
                            RUN_TEST(test_core_getDeviceInfo)

Pmic_CoreHandle_t pmicCoreHandle;

int main(void)
{
    uartHandle_t vcpHandle;
    i2cHandle_t I2C1Handle;
    const Pmic_CoreCfg_t pmicCoreCfg = {
        .validParams = (PMIC_CFG_DEVICE_TYPE_VALID_SHIFT    | PMIC_CFG_COMM_MODE_VALID_SHIFT      |
                        PMIC_CFG_SLAVEADDR_VALID_SHIFT      | PMIC_CFG_QASLAVEADDR_VALID_SHIFT    |
                        PMIC_CFG_COMM_HANDLE_VALID_SHIFT    | PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT  |
                        PMIC_CFG_COMM_IO_RD_VALID_SHIFT     | PMIC_CFG_COMM_IO_WR_VALID_SHIFT     |
                        PMIC_CFG_CRITSEC_START_VALID_SHIFT  | PMIC_CFG_CRITSEC_STOP_VALID_SHIFT   |
                        PMIC_CFG_I2C1_SPEED_VALID_SHIFT     | PMIC_CFG_I2C2_SPEED_VALID_SHIFT),
        .pmicDeviceType = PMIC_DEV_BURTON_TPS6522X,
        .commMode = PMIC_INTF_SINGLE_I2C,
        .slaveAddr = BURTON_I2C_USER_PAGE_ADDRESS,
        .qaSlaveAddr = BURTON_I2C_WDG_PAGE_ADDRESS,
        .pCommHandle = &I2C1Handle,
        .pQACommHandle = &I2C1Handle,
        .pFnPmicCommIoRead = &pmicI2CRead,
        .pFnPmicCommIoWrite = &pmicI2CWrite,
        .pFnPmicCritSecStart = &pmicCritSecStart,
        .pFnPmicCritSecStop = &pmicCritSecStop,
        .i2c1Speed = PMIC_I2C_STANDARD_MODE
    };
    int32_t status = PMIC_ST_SUCCESS;

    // Initialize system clock
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 400 / 2 / 4 = 50 MHz clock rate

    // Enable printing to console
    initializeVCPHandle(&vcpHandle);
    initializeVCP(&vcpHandle);

    // Initialize I2C for communicating with PMIC
    initializeI2C1Handle(&I2C1Handle);
    initializeI2C(&I2C1Handle);

    // Initialize PMIC handle
    initializePmicCoreHandle(&pmicCoreHandle);
    status = Pmic_init(&pmicCoreCfg, &pmicCoreHandle);

    // Clear console of any previous messages
    clearConsole(&vcpHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        // Disable all power resources and clear all interrupts
        disablePmicPowerResources(pmicCoreHandle);
        status = Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);

        // Conduct unit testing
        if (status == PMIC_ST_SUCCESS)
        {
            UARTStrPut(&vcpHandle, "Running all PMIC Core tests...\r\n\r\n");

            UNITY_BEGIN();
            RUN_CORE_TESTS();
            UNITY_END();
        }
        // Error message if IRQs were not able to be cleared
        else
        {
            UARTStrPut(&vcpHandle, "Error in clearing all PMIC IRQs. Error code: ");
            UARTInt32Put(&vcpHandle, status);
            UARTStrPut(&vcpHandle, "\r\n\r\n");
        }
    }
    // Error message if PMIC handle not initialized correctly
    else
    {
        UARTStrPut(&vcpHandle, "Error in initializing PMIC handle. Error code: ");
        UARTInt32Put(&vcpHandle, status);
        UARTStrPut(&vcpHandle, "\r\n\r\n");
    }

    return status;
}

/**
 *  \brief  Pmic_getRecoveryCntCfg: Test whether API can set recovery count threshold value
 */
void test_core_setRecoveryCntCfg_thrVal(void)
{
    int32_t                  status = PMIC_ST_SUCCESS;
    const Pmic_RecovCntCfg_t recovCntCfg_expected = {.validParams = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT,
                                                     .thrVal = 15};
    Pmic_RecovCntCfg_t       recovCntCfg_actual = {.validParams = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT};

    // Set recovery count threshold value
    status = Pmic_setRecoveryCntCfg(&pmicCoreHandle, recovCntCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual recovery count threshold value
    status = Pmic_getRecoveryCntCfg(&pmicCoreHandle, &recovCntCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(15, recovCntCfg_actual.thrVal);
}

/**
 *  \brief  Pmic_getRecoveryCntCfg: Test whether API can clear the recovery counter
 */
void test_core_setRecoveryCntCfg_clrCnt(void)
{
    int32_t                  status = PMIC_ST_SUCCESS;
    uint8_t                  recovCnt = 0;
    const Pmic_RecovCntCfg_t recovCntCfg_expected = {.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID, .clrCnt = true};

    // Clear recovery count
    status = Pmic_setRecoveryCntCfg(&pmicCoreHandle, recovCntCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual recovery count
    status = Pmic_getRecoveryCnt(&pmicCoreHandle, &recovCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(0, recovCnt);
}

/**
 *  \brief  Pmic_setScratchPadValue: Test whether API can set the value of all scratchpad registers
 */
void test_core_setScratchPadValue_allScratchPads(void)
{
    uint8_t       iter = 0;
    uint8_t       data_expected = 0;
    uint8_t       data_actual = 0;
    const uint8_t increment = 25;
    int32_t       status = PMIC_ST_SUCCESS;

    // For each scratchpad register...
    for (iter = PMIC_SCRATCH_PAD_REG_1; iter <= PMIC_SCRATCH_PAD_REG_4; iter++)
    {
        // Set scratchpad value to be an increment of 25 (arbitrary increment)
        data_expected = increment * (iter + 1);
        status = Pmic_setScratchPadValue(&pmicCoreHandle, iter, data_expected);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual scratchpad value
        status = Pmic_getScratchPadValue(&pmicCoreHandle, iter, &data_actual);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual value
        TEST_ASSERT_EQUAL(data_expected, data_actual);
    }
}

/**
 *  \brief  Pmic_setUserSpareValue: Test whether API can set the value of all valid user spares and
 *                                  be able to handle errors for invalid user spares
 */
void test_core_setUserSpareValue_allUserSpares(void)
{
    uint8_t iter = 0;
    uint8_t actualData = 0;
    int32_t status = PMIC_ST_SUCCESS;

    // For each user spare...
    for (iter = PMIC_USER_SPARE_REG_1; iter <= PMIC_USER_SPARE_REG_4; iter++)
    {
        // set the user spare value
        status = Pmic_setUserSpareValue(&pmicCoreHandle, iter, PMIC_USER_SPARE_REG_VAL_1);

        // Depending on user spare value, check for error or check for expected vs. actual value
        switch (iter)
        {
            case PMIC_USER_SPARE_REG_1:
            case PMIC_USER_SPARE_REG_4:
                TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
                status = Pmic_getUserSpareValue(&pmicCoreHandle, iter, &actualData);
                TEST_ASSERT_EQUAL(PMIC_USER_SPARE_REG_VAL_1, actualData);

                break;
            case PMIC_USER_SPARE_REG_2:
            case PMIC_USER_SPARE_REG_3:
                TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

                break;
        }
    }
}

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can enable/disable spread spectrum
 */
void test_core_setCommonCtrlCfg_spreadSpectrumEn(void)
{
    int32_t              status = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_expected = {.validParams = PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT,
                                                   .spreadSpectrumEn = PMIC_SPREAD_SPECTRUM_CFG_ENABLE};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_actual = {.validParams = PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT};

    // Enable spread spectrum
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual spread spectrum enable/disable status
    status = Pmic_getCommonCtrlConfig(&pmicCoreHandle, &commonCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(commonCtrlCfg_expected.spreadSpectrumEn, commonCtrlCfg_actual.spreadSpectrumEn);

    // Disable spread spectrum
    commonCtrlCfg_expected.spreadSpectrumEn = PMIC_SPREAD_SPECTRUM_CFG_DISABLE;
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual spread spectrum enable/disable status
    status = Pmic_getCommonCtrlConfig(&pmicCoreHandle, &commonCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(commonCtrlCfg_expected.spreadSpectrumEn, commonCtrlCfg_actual.spreadSpectrumEn);
}

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test error handling for when user wants to configure the Skip
 *                                    EEPROM Default Load Enable (TPS6522x does not support this feature)
 */
void test_core_setCommonCtrlCfg_skipEepromDefaultLoadEn(void)
{
    int32_t              status = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg = {.validParams = PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT,
                                          .skipEepromDefaultLoadEn =
                                              PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_ENABLED};

    // Enable skip EEPROM default load and compare expected vs. actual return code
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test error handling for when user wants to configure the EEPROM
 *                                    Default Load (TPS6522x does not support this feature)
 */
void test_core_setCommonCtrlCfg_eepromDefaultLoad(void)
{
    int32_t              status = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg = {.validParams = PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
                                          .eepromDefaultLoad =
                                              PMIC_TPS6594X_EEPROM_DEFAULTS_NOT_LOADED_TO_RTC_DOMAIN_BITS};

    // Set EEPROM default load and compare expected vs. actual return code
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can configure EN_DRV
 */
void test_core_setCommonCtrlCfg_enDrv(void)
{
    int32_t              status = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_expected = {.validParams = PMIC_CFG_ENABLE_DRV_VALID_SHIFT,
                                                   .enDrv = PMIC_PIN_SIGNAL_LEVEL_LOW};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_actual = {.validParams = PMIC_CFG_ENABLE_DRV_VALID_SHIFT};

    // Set EN_DRV signal to low
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual EN_DRV signal level
    status = Pmic_getCommonCtrlConfig(&pmicCoreHandle, &commonCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(commonCtrlCfg_expected.enDrv, commonCtrlCfg_actual.enDrv);

    // Set EN_DRV signal to high
    commonCtrlCfg_expected.spreadSpectrumEn = PMIC_PIN_SIGNAL_LEVEL_HIGH;
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual EN_DRV signal level
    status = Pmic_getCommonCtrlConfig(&pmicCoreHandle, &commonCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(commonCtrlCfg_expected.enDrv, commonCtrlCfg_actual.enDrv);
}

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can lock/unlock registers
 */
void test_core_setCommonCtrlCfg_regLock(void)
{
    int32_t               status = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t  commonCtrlCfg = {.validParams = PMIC_CFG_REG_LOCK_VALID_SHIFT, .regLock = PMIC_REGISTER_LOCK};
    Pmic_CommonCtrlStat_t commonCtrlStat = {.validParams = PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT};

    // Lock registers
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get register lock status
    status = Pmic_getCommonCtrlStat(&pmicCoreHandle, &commonCtrlStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value (1 means locked)
    TEST_ASSERT_EQUAL(1, commonCtrlStat.regLockStat);

    // Unlock registers
    commonCtrlCfg.regLock = PMIC_REGISTER_UNLOCK;
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get register lock status
    status = Pmic_getCommonCtrlStat(&pmicCoreHandle, &commonCtrlStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value (0 means unlocked)
    TEST_ASSERT_EQUAL(0, commonCtrlStat.regLockStat);
}

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can configure the spread spectrum depth
 */
void test_core_setCommonCtrlCfg_spreadSpectrumDepth(void)
{
    int32_t              status = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_expected = {.validParams = PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT,
                                                   .spreadSpectrumDepth =
                                                       PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_7_PERCENT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_actual = {.validParams = PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT};

    // Set spread spectrum modulation depth to be 7%
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual spread spectrum modulation depth
    status = Pmic_getCommonCtrlConfig(&pmicCoreHandle, &commonCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(commonCtrlCfg_expected.spreadSpectrumDepth, commonCtrlCfg_actual.spreadSpectrumDepth);

    // Set spread spectrum modulation depth to be 4%
    commonCtrlCfg_expected.spreadSpectrumDepth = PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_4_PERCENT;
    status = Pmic_setCommonCtrlConfig(&pmicCoreHandle, commonCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual spread spectrum modulation depth
    status = Pmic_getCommonCtrlConfig(&pmicCoreHandle, &commonCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(commonCtrlCfg_expected.spreadSpectrumDepth, commonCtrlCfg_actual.spreadSpectrumDepth);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure AMUXOUT_EN
 *                                  (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_amuxOutRefOutEn(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg = {.validParams = PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT,
                                      .amuxOutRefOutEn = PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE};

    // Configure AMUXOUT_EN and compare expected vs. actual return code
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure CLKMON_EN
 *                                  (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_clkMonEn(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg = {.validParams = PMIC_CFG_CLK_MON_EN_VALID_SHIFT,
                                      .clkMonEn = PMIC_INTERNAL_CLK_MONITORING_CFG_ENABLE};

    // Configure CLKMON_EN and compare expected vs. actual return code
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure
 *                                  SYNCCLKOUT_FREQ_SEL (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_syncClkOutFreqSel(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg = {.validParams = PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT,
                                      .clkMonEn = PMIC_SYNCCLKOUT_4_4_MHZ};

    // Configure SYNCCLKOUT_FREQ_SEL and compare expected vs. actual return code
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test whether API can configure the external clock mode (SEL_EXT_CLK)
 */
void test_core_setMiscCtrlCfg_extClkSel(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_expected = {.validParams = PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT,
                                               .extClkSel = PMIC_AUTOMATIC_EXT_CLK};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_actual = {.validParams = PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT};

    // Configure external clock to be automatic
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual external clock configuration
    status = Pmic_getMiscCtrlConfig(&pmicCoreHandle, &miscCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(miscCtrlCfg_expected.extClkSel, miscCtrlCfg_actual.extClkSel);

    // Configure external clock to be forced to internal RC osc.
    miscCtrlCfg_expected.extClkSel = PMIC_INTERNAL_RC_OSC;
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual external clock configuration
    status = Pmic_getMiscCtrlConfig(&pmicCoreHandle, &miscCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(miscCtrlCfg_expected.extClkSel, miscCtrlCfg_actual.extClkSel);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test whether API can configure external clock frequency
 */
void test_core_setMiscCtrlCfg_syncClkInFreq(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_expected = {.validParams = PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_actual = {.validParams = PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT};

    // For each possible external clock frequency...
    for (miscCtrlCfg_expected.syncClkInFreq = PMIC_TPS6522X_SYNCCLKIN_1_1_MHZ;
         miscCtrlCfg_expected.syncClkInFreq <= PMIC_TPS6522X_SYNCCLKIN_8_8_MHZ;
         miscCtrlCfg_expected.syncClkInFreq++)
    {
        // Set frequency configuration
        status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg_expected);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Get actual frequency configuration
        status = Pmic_getMiscCtrlConfig(&pmicCoreHandle, &miscCtrlCfg_actual);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        // Compare expected vs. actual value
        TEST_ASSERT_EQUAL(miscCtrlCfg_expected.syncClkInFreq, miscCtrlCfg_actual.syncClkInFreq);
    }

    // Set external clock frequency back to its TRM reset value
    miscCtrlCfg_expected.syncClkInFreq = PMIC_TPS6522X_SYNCCLKIN_2_2_MHZ;
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure
 *                                  NRSTOUT_SOC signal (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_nRstOutSocSignal(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg = {.validParams = PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,
                                      .nRstOutSocSignal = PMIC_PIN_SIGNAL_LEVEL_HIGH};

    // Configure NRSTOUT_SOC signal and compare expected vs. actual return code
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test whether API can configure NRSTOUT signal
 */
void test_core_setMiscCtrlCfg_nRstOutSignal(void)
{
    int32_t            status = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_expected = {.validParams = PMIC_CFG_NRSTOUT_VALID_SHIFT,
                                               .nRstOutSignal = PMIC_PIN_SIGNAL_LEVEL_LOW};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_actual = {.validParams = PMIC_CFG_NRSTOUT_VALID_SHIFT};

    // Configure NRSTOUT signal level to be low
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual NRSTOUT signal level
    status = Pmic_getMiscCtrlConfig(&pmicCoreHandle, &miscCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(miscCtrlCfg_expected.nRstOutSignal, miscCtrlCfg_actual.nRstOutSignal);

    // Configure NRSTOUT signal level to be high
    miscCtrlCfg_expected.nRstOutSignal = PMIC_PIN_SIGNAL_LEVEL_HIGH;
    status = Pmic_setMiscCtrlConfig(&pmicCoreHandle, miscCtrlCfg_expected);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Get actual NRSTOUT signal level
    status = Pmic_getMiscCtrlConfig(&pmicCoreHandle, &miscCtrlCfg_actual);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    // Compare expected vs. actual value
    TEST_ASSERT_EQUAL(miscCtrlCfg_expected.nRstOutSignal, miscCtrlCfg_actual.nRstOutSignal);
}

/**
 *  \brief  Pmic_setBatteryCtrlConfig: Test API error handling for when user wants to set the
 *                                     battery configurations (TPS6522x does not support this
 *                                     feature)
 */
void test_core_setBatteryCtrlCfg_invalidDevice(void)
{
    
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg = {
        .validParams = (PMIC_CFG_CHARGING_EN_VALID_SHIFT            |
                        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT  |
                        PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT),
        .chargingEn         = true,
        .endOfChargeVoltage = PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_3_3_V,
        .chargeCurrent      = PMIC_TPS6594X_BB_CHARGING_CURRENT_500
    };
    

    // Set battery CTRL CFG and compare expected vs. actual return code
    status = Pmic_setBatteryCtrlConfig(&pmicCoreHandle, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);
}

/**
 *  \brief  Pmic_getBatteryCtrlConfig: Test API error handling for when user wants to get the
 *                                     battery configurations (TPS6522x does not support this
 *                                     feature)
 */
void test_core_getBatteryCtrlCfg_invalidDevice(void)
{
    
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg = {
        .validParams = (PMIC_CFG_CHARGING_EN_VALID_SHIFT            |
                        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT  |
                        PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT)
    };
    

    // Get battery CTRL CFG and compare expected vs. actual return code
    status = Pmic_getBatteryCtrlConfig(&pmicCoreHandle, &batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, status);
}

/**
 *  \brief      This private helper function is used to test the Pmic_getCommonCtrlStat API.
 *              Given a validParam, it outputs a return code - the return code will be tested
 *              against an expected value.
 *
 *  \param      validParam      [IN]    The return code is generated based on this parameter
 *
 *  \return     Success code if status of the parameter was obtained, error code otherwise
 */
static int32_t getCommonCtrlStatTest(uint32_t validParam)
{
    Pmic_CommonCtrlStat_t commonCtrlStat = {.validParams = validParam};

    return Pmic_getCommonCtrlStat(&pmicCoreHandle, &commonCtrlStat);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test API error handling for when user wants to set the
 *                                  SPMI low power mode control (TPS6522x does not support this
 *                                  feature)
 */
void test_core_getCommonCtrlStat_spmiLpmStat(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the FORCE_EN_DRV_LOW status
 */
void test_core_getCommonCtrlStat_forceEnDrvLowStat(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test API error handling for when user wants to get the
 *                                  backup battery end of charge indication (TPS6522x does
 *                                  not support this feature)
 */
void test_core_getCommonCtrlStat_bbEndOfChargeIndication(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_BB_EOC_INDICATION_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the register lock/unlock status
 */
void test_core_getCommonCtrlStat_regLockStat(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the external clock validity status
 */
void test_core_getCommonCtrlStat_extClkValidity(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the startup pin status
 */
void test_core_getCommonCtrlStat_startupPin(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_STARTUP_PIN_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the EN_DRV pin
 *                                  status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_enDrvPin(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_EN_DRV_PIN_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the NRSTOUT_SOC
 *                                  pin status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_nRstOutSocPin(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the NRSTOUT pin
 *                                  status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_nRstOutPin(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_NRSTOUT_PIN_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the nINT pin
 *                                  status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_nIntPin(void)
{
    int32_t status = getCommonCtrlStatTest(PMIC_CFG_NINT_PIN_STAT_VALID_SHIFT);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);
}

/**
 *  \brief  Pmic_getDeviceInfo: Test whether API can get the PMIC device information
 */
void test_core_getDeviceInfo(void)
{
    int32_t           status = PMIC_ST_SUCCESS;
    Pmic_DeviceInfo_t deviceInfo;

    status = Pmic_getDeviceInfo(&pmicCoreHandle, &deviceInfo);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
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
    (void)Pmic_irqClrErrStatus(&pmicCoreHandle, PMIC_IRQ_ALL);
}
