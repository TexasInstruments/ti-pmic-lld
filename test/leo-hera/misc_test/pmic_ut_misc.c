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
 *  \file   pmic_ut_misc.c
 *
 *  \brief  PMIC Unit Test for testing PMIC MISC APIs
 *
 */

#include <pmic_ut_misc.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;
uint32_t g_skip_eeprom_test_flag = 0;

extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;
static int32_t test_pmic_leo_pmicA_misc_testApp(void);

extern int8_t gRecoveryCntCfgTestFlag;
extern int8_t gEnableDrvPinCfgTestFlag;
extern int8_t gI2c1CrcEnableTestFlag;
extern int8_t gI2c2CrcEnableTestFlag;
extern int8_t gSpmiLpmStatTestFlag;

/*!
 * \brief   PMIC MISC Test Cases
 */
static Pmic_Ut_Tests_t pmic_misc_tests[] =
{
    /*! testID
     *  testDesc
     */
    {
        7628,
        "Pmic_setRecoveryCntCfg : Test Set Recovery Counter Threshold."
    },
    {
        7629,
        "Pmic_setRecoveryCntCfg : Parameter validation for thrVal."
    },
    {
        7630,
        "Pmic_setRecoveryCntCfg : Test Set/Get Clear Recovery Counter."
    },
    {
        7631,
        "Pmic_setRecoveryCntCfg : Parameter validation for clrCnt."
    },
    {
        7632,
        "Pmic_setRecoveryCntCfg : Parameter validation for handle."
    },
    {
        7633,
        "Pmic_getRecoveryCntCfg : Parameter validation for handle."
    },
    {
        7634,
        "Pmic_getRecoveryCntCfg : Parameter validation for recovCntCfg."
    },
    {
        7635,
        "Pmic_getRecoveryCnt : Parameter validation for handle."
    },
    {
        7636,
        "Pmic_getRecoveryCnt : Parameter validation for recovCntVal."
    },
    {
        7637,
        "Pmic_getRecoveryCnt : Read Recovery Count Value."
    },
    {
        7715,
        "Pmic_irqGetErrStatus : Test BIST_PASS_INT interrupt."
    },
    {
        0xAB1F,
        "Pmic_irqGetErrStatus : Test FSD_INT interrupt."
    },
    {
        0xAB2D,
        "Pmic_irqGetErrStatus : Test NPWRON_INT interrupt."
    },
    {
        0xAB20,
        "Pmic_irqGetErrStatus : Test NPWRON_LONG_INT interrupt."
    },
    {
        0xAB2C,
        "Pmic_commFrmError : Test COMM_FRM_ERROR interrupt"
    },
    {
        7768,
        "Pmic_irqGetErrStatus : Test ENABLE_INT interrupt."
    },
    {
        8321,
        "Pmic_setScratchPadValue : Parameter validation for handle."
    },
    {
        8322,
        "Pmic_setScratchPadValue : Parameter validation for scratchPadRegId."
    },
    {
        8323,
        "Pmic_setScratchPadValue : Test Set/Get Scratchpad value."
    },
    {
        8324,
        "Pmic_getScratchPadValue : Parameter validation for handle."
    },
    {
        8325,
        "Pmic_getScratchPadValue : Parameter validation for scratchPadRegId."
    },
    {
        8326,
        "Pmic_getScratchPadValue : Parameter validation for data."
    },
    {
        9915,
        "Pmic_setUserSpareValue : Parameter validation for handle."
    },
    {
        9916,
        "Pmic_setUserSpareValue : Parameter validation for userSpareRegNum."
    },
    {
        9917,
        "Pmic_setUserSpareValue : Parameter validation for data."
    },
    {
        9918,
        "Pmic_setUserSpareValue : Test Set/Get userSpare Reg value."
    },
    {
        9919,
        "Pmic_getUserSpareValue : Parameter validation for handle."
    },
    {
        9920,
        "Pmic_getUserSpareValue : Parameter validation for userSpareRegNum."
    },
    {
        9921,
        "Pmic_getUserSpareValue : Parameter validation for data."
    },
    {
        9922,
        "Pmic_setCommonCtrlConfig : Enable/Disable Spread Spectrum"
    },
    {
        9923,
        "Pmic_setCommonCtrlConfig : Lock and UnLock Registers"
    },
    {
        9924,
        "Pmic_setCommonCtrlConfig : Configure Spread Spectrum Depth"
    },
    {
        9925,
        "Pmic_getCommonCtrlConfig : Parameter validation for handle."
    },
    {
        9926,
        "Pmic_getCommonCtrlConfig : Parameter validation for pCommonCtrlCfg."
    },
    {
        9927,
        "Pmic_setCommonCtrlConfig : Parameter validation for eepromDefaultLoad."
    },
    {
        9928,
        "Pmic_setCommonCtrlConfig : Parameter validation for regLock."
    },
    {
        9929,
        "Pmic_setCommonCtrlConfig : Parameter validation for spreadSpectrumDepth."
    },
    {
        9930,
        "Pmic_setCommonCtrlConfig : Parameter validation for handle."
    },
    {
        9931,
        "Pmic_setMiscCtrlConfig : Enable/Disable Band gap Voltage to AMUX OUT/RFF OUT Pin"
    },
    {
        9932,
        "Pmic_setMiscCtrlConfig : Enable or Disable internal Clock Monitoring"
    },
    {
        9933,
        "Pmic_setMiscCtrlConfig : Selects SYNCCLKOUT Frequency"
    },
    {
        9934,
        "Pmic_setMiscCtrlConfig : External clock Selection"
    },
    {
        9935,
        "Pmic_setMiscCtrlConfig : Selects External clock Frequency"
    },
    {
        9936,
        "Pmic_setMiscCtrlConfig : Parameter validation for syncClkOutFreqSel"
    },
    {
        9937,
        "Pmic_setMiscCtrlConfig : Parameter validation for extClkSel"
    },
    {
        9938,
        "Pmic_setMiscCtrlConfig : Parameter validation for syncClkInFreq"
    },
    {
        9939,
        "Pmic_setMiscCtrlConfig : Parameter validation for handle"
    },
    {
        9940,
        "Pmic_getMiscCtrlConfig : Parameter validation for handle"
    },
    {
        9941,
        "Pmic_getMiscCtrlConfig : Parameter validation for pMiscCtrlCfg"
    },
    {
        9942,
        "Pmic_setBatteryCtrlConfig : Enable/Disable Backup Battery Charging"
    },
    {
        9943,
        "Pmic_setBatteryCtrlConfig : Configure Backup Battery configuration for End of charge Voltage"
    },
    {
        9944,
        "Pmic_setBatteryCtrlConfig : Configure Backup Battery charging current value"
    },
    {
        9945,
        "Pmic_setBatteryCtrlConfig : Parameter validation for endOfChargeVoltage"
    },
    {
        9946,
        "Pmic_setBatteryCtrlConfig : Parameter validation for chargeCurrent"
    },
    {
        9948,
        "Pmic_setBatteryCtrlConfig : Parameter validation for handle"
    },
    {
        9949,
        "Pmic_getBatteryCtrlConfig : Parameter validation for handle"
    },
    {
        9950,
        "Pmic_getBatteryCtrlConfig : Parameter validation for pBatteryCtrlCfg"
    },
    {
        9951,
        "Pmic_getCommonCtrlStat : Get the status of nIntPin"
    },
    {
        9952,
        "Pmic_getCommonCtrlStat : Get the status of spmiLpmStat"
    },
    {
        9953,
        "Pmic_getCommonCtrlStat : Get the status of forceEnDrvLowStat"
    },
    {
        9954,
        "Pmic_getCommonCtrlStat : Get the status of bbEndOfChargeIndication"
    },
    {
        9955,
        "Pmic_getCommonCtrlStat : Get the status of regLockStat"
    },
    {
        9956,
        "Pmic_getCommonCtrlStat : Get the status of extClkValidity"
    },
    {
        9957,
        "Pmic_getCommonCtrlStat : Get the status of startupPin"
    },
    {
        9958,
        "Pmic_getCommonCtrlStat : Get the status of enDrvPin"
    },
    {
        9959,
        "Pmic_getCommonCtrlStat : Get the status of nRstOutSocPin"
    },
    {
        9960,
        "Pmic_getCommonCtrlStat : Get the status of nRstOutPin"
    },
    {
        9961,
        "Pmic_getCommonCtrlStat : Parameter validation for handle"
    },
    {
        9962,
        "Pmic_getCommonCtrlStat : Parameter validation for pCommonCtrlStat"
    },
    {
        9963,
        "Pmic_getI2CSpeed :  Parameter validation for handle"
    },
    {
        9964,
        "Pmic_getI2CSpeed :  Parameter validation for pI2C1Speed"
    },
    {
        9965,
        "Pmic_getI2CSpeed :  Parameter validation for pI2C2Speed"
    },
    {
        9966,
        "Pmic_getI2CSpeed :  Get I2C1 or I2C2 Speed value"
    },
    {
        9967,
        "Pmic_getDeviceInfo :  Get the Device Information"
    },
    {
        9968,
        "Pmic_getDeviceInfo :  Parameter validation for handle"
    },
    {
        9969,
        "Pmic_getDeviceInfo :  Parameter validation for pDeviceInfo"
    },
    {
        9970,
        "Pmic_getPinValue : Get the status of EN_DRV Pin Value "
    },
    {
        9971,
        "Pmic_getPinValue : Get the status of NRSTOUT_SOC Pin Value "
    },
    {
        9972,
        "Pmic_getPinValue : Get the status of NRSTOUT Pin Value "
    },
    {
        9973,
        "Pmic_getPinValue : Parameter validation for pinType "
    },
    {
        9974,
        "Pmic_getPinValue : Parameter validation for handle "
    },
    {
        9975,
        "Pmic_getPinValue : Parameter validation for pPinValue "
    },
    {
        9978,
        "Pmic_setMiscCtrlConfig : Configure NRSTOUT_SOC Signal"
    },
    {
        9979,
        "Pmic_setMiscCtrlConfig : Configure NRSTOUT Signal"
    },
    {
        9980,
        "Pmic_setMiscCtrlConfig : Parameter validation for NRSTOUT_SOC Signal"
    },
    {
        9981,
        "Pmic_setMiscCtrlConfig : Parameter validation for NRSTOUT Signal"
    },
    {
        9982,
        "Pmic_getMiscCtrlConfig : Get the status of NRSTOUT_SOC Signal"
    },
    {
        9983,
        "Pmic_getMiscCtrlConfig : Get the status of NRSTOUT Signal"
    },
    {
        9984,
        "Pmic_setCommonCtrlConfig : Configure EN_DRV Signal"
    },
    {
        9985,
        "Pmic_setCommonCtrlConfig : Parameter validation for EN_DRV Signal"
    },
    {
        9988,
        "Pmic_setCommonCtrlConfig : Load from EEPROM defaults on RTC domain/conf Registers"
    },
    {
        9989,
        "Pmic_setCommonCtrlConfig : Not Loaded from EEPROM defaults on RTC domain/conf Registers"
    },
    {
        9990,
        "Pmic_setCommonCtrlConfig : Enable to skip EEPROM defaults load on conf registers with FIRST_STARTUP_DONE as '0'"
    },
    {
        9991,
        "Pmic_setCommonCtrlConfig : Disable to skip EEPROM defaults load on conf registers with FIRST_STARTUP_DONE as '0'"
    },
    {
        9993,
        "Pmic_setCommonCtrlConfig : Disable to skip EEPROM defaults load on conf registers with FIRST_STARTUP_DONE as '1'"
    },
    {
        9994,
        "Test Pmic Write Protection when Register is Lock"
    },
    {
        9995,
        "Test Pmic Write Protection when Register is UnLock"
    },
    {
        10573,
        "Pmic_getRecoveryCnt : Test to read recovery counter value"
    },
    {
        10574,
        "Pmic_getRecoveryCnt : Parameter validation for pRecovCntVal"
    },
    {
        10575,
        "Pmic_init : Parameter validation for pPmicCoreHandle"
    },
    {
        10577,
        "Pmic_init : Parameter validation for pPmicConfigData"
    },
    {
        10578,
        "Pmic_init : Parameter validation for pmicDeviceType"
    },
    {
        10579,
        "Pmic_init : Parameter validation for Interface mode"
    },
    {
        10580,
        "Pmic_init : Parameter validation for I2C1/SPI Main Interface handle"
    },
    {
        10581,
        "Pmic_init : Parameter validation for I2C2-QA Interface handle"
    },
    {
        10582,
        "Pmic_init : Parameter validation for I2C/SPI Comm LLD Read Function"
    },
    {
        10583,
        "Pmic_init : Parameter validation for I2C/SPI Comm LLD Write Function"
    },
    {
        10584,
        "Pmic_init : Parameter validation for Critical Section Start Function"
    },
    {
        10585,
        "Pmic_init : Parameter validation for Critical Section Stop Function"
    },
    {
        10586,
        "Pmic_deinit : Parameter validation for pPmicCoreHandle"
    },
    {
        10587,
        "Pmic_setScratchPadValue : Test to write data to scratchpad reg1, reg2 and reg3"
    },
    {
        10588,
        "Pmic_init : Parameter validation for I2C1 Speed"
    },
    {
        10589,
        "Pmic_init : Parameter validation for I2C2 Speed"
    },
    {
        10590,
        "Pmic_setCommonCtrlConfig : Enable Skip EEPROM Default Load to CONF and Other registers"
    },
    {
        10591,
        "Pmic_setI2CSpeedCfg : Parameter validation for handle"
    },
    {
        10592,
        "Pmic_getCrcStatus : Parameter validation for handle"
    },
    {
        10593,
        "Pmic_getCrcStatus : Parameter validation for pI2c2CrcStatus"
    },
    {
        10594,
        "Pmic_getCrcStatus : Parameter validation for pI2c1CrcStatus"
    },
    {
        10595,
        "Pmic_getCrcStatus : Parameter validation for both pI2c1CrcStatus and pI2c2CrcStatus"
    },
    {
        10596,
        "Pmic_enableCRC : Parameter validation for handle"
    },
    {
        10597,
        "Pmic_setBatteryCtrlConfig : Negative test for LP8764x Hera PMIC device"
    },
    {
        10598,
        "Pmic_getBatteryCtrlConfig : Negative test for LP8764x Hera PMIC device"
    },
    {
        10599,
        "Pmic_setCommonCtrlConfig : Disable Load from EEPROM defaults on RTC domain/conf Registers"
    },
    {
        10600,
        "Pmic_setCommonCtrlConfig : Negative test for TPS6594x Leo PMIC device"
    },
    {
        10601,
        "Pmic_getCommonCtrlConfig : Negative test for TPS6594x Leo PMIC device"
    },
    {
        10602,
        "Pmic_setMiscCtrlConfig : Configure NRSTOUT Signal for high signal"
    },
    {
        10603,
        "Pmic_setMiscCtrlConfig : Configure NRSTOUT_SOC Signal for high signal"
    },
    {
        10604,
        "Pmic_init : Config Device Type as PMIC_DEV_HERA_LP8764X"
    },
    {
        10751,
        "Pmic_setCommonCtrlConfig : Enable Skip EEPROM Default Load to CONF and Other registers"
    },
    {
        10752,
        "Test Pmic Write Protection when Register is Lock - code coverage"
    },
    {
        10786,
        "Pmic_setCommonCtrlConfig : Configure EN_DRV Signal - code coverage"
    },
    {
        8933,
        "Pmic_miscTests : Fault Injection and Coverage Gaps."
    },
};

/*!
 * \brief   Pmic_irqGetErrStatus : Test ENABLE_INT interrupt.
 */
static void test_pmic_interruptEnableInt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool intrEnable           = PMIC_IRQ_UNMASK;

    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(7768,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(7768,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(startup_type == PMIC_ENABLE_STARTUP_TYPE)
    {
        pmic_log("Enable-pin Start-up is detected\n");
    }
    else
    {
        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                          PMIC_TPS6594X_ENABLE_INT,
                                          intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                          PMIC_LP8764X_ENABLE_INT,
                                          intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        }

        /* Enable WDG Timer */
        pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Set QA parameters */
        pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        /* Waiting for longwindow time interval expiring */
        Osal_delay(wdgCfg.longWinDuration_ms);
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7768,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setRecoveryCntCfg : Test Set Recovery Counter Threshold.
 */
static void test_pmic_setRecoveryCntCfg_threshold(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;
    Pmic_RecovCntCfg_t recovCntCfg_rd;

    recovCntCfg.validParams    = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg_rd.validParams = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg.thrVal         = PMIC_RECOV_CNT_THR_MAX;

    test_pmic_print_unity_testcase_info(7628,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, &recovCntCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    TEST_ASSERT_EQUAL(recovCntCfg.thrVal, recovCntCfg_rd.thrVal);

    pmic_testResultUpdate_pass(7628,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setRecoveryCntCfg : Parameter validation for thrVal.
 */
static void test_pmic_setRecoveryCntCfgPrmValTest_thrVal(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg.thrVal      = PMIC_RECOV_CNT_THR_MAX + 1U;

    test_pmic_print_unity_testcase_info(7629,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7629,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setRecoveryCntCfg : Test Set/Get Clear Recovery Counter.
 */
static void test_pmic_setRecoveryCntCfg_clrCnt(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;
    Pmic_RecovCntCfg_t recovCntCfg_rd;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
    recovCntCfg.clrCnt      = 1U;

    test_pmic_print_unity_testcase_info(7630,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    recovCntCfg_rd.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
    status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, &recovCntCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(0U, recovCntCfg_rd.clrCnt);

    pmic_testResultUpdate_pass(7630,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setRecoveryCntCfg : Parameter validation for clrCnt.
 */
static void test_pmic_setRecoveryCntCfgPrmValTest_clrCnt(void)
{
    int32_t status             = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
    recovCntCfg.clrCnt      = 0U;

    test_pmic_print_unity_testcase_info(7631,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(7631,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setRecoveryCntCfg : Parameter validation for handle.
 */
static void test_pmic_setRecoveryCntCfgPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    recovCntCfg.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;

    test_pmic_print_unity_testcase_info(7632,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setRecoveryCntCfg(NULL, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7632,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getRecoveryCntCfg : Parameter validation for handle.
 */
static void test_pmic_getRecoveryCntCfgPrmValTest_handle(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;

    test_pmic_print_unity_testcase_info(7633,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCntCfg(NULL, &recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7633,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getRecoveryCntCfg : Parameter validation for recovCntCfg.
 */
static void test_pmic_getRecoveryCntCfgPrmValTest_recovCntCfg(void)
{
    int32_t status             = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7634,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(7634,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getRecoveryCnt : Parameter validation for handle.
 */
static void test_Pmic_getRecoveryCntPrmValTest_handle(void)
{
    int32_t status      = PMIC_ST_SUCCESS;
    uint8_t recovCntVal = 0U;

    test_pmic_print_unity_testcase_info(7635,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCnt(NULL, &recovCntVal);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(7635,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getRecoveryCnt : Parameter validation for recovCntVal.
 */
static void test_Pmic_getRecoveryCntPrmValTest_recovCntVal(void)
{
    int32_t status             = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(7636,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCnt(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(7636,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * To test the following manual test-case,
 * the test-case should be executed twice continuously.
 */
/*!
 * \brief   Pmic_getRecoveryCnt : Read Recovery Count Value.
 */
static void test_pmic_getRecoveryCnt_read_recovCntVal(void)
{
    int32_t status          = PMIC_ST_SUCCESS;
    uint8_t recovCntVal     = 0U;
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(7637,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(7637,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    status = Pmic_getRecoveryCnt(pPmicCoreHandle, &recovCntVal);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_log("Recovery count: %d\n", recovCntVal);
    /* Enable WDG Timer */
    status = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Set QA parameters */
    status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(7637,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetErrStatus : Test BIST_PASS_INT interrupt.
 */
static void test_Pmic_getBistPassInterrupt(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    bool nsleepType = 0U;
    bool maskEnable = 0U;
    int8_t timeout = 10U;

    test_pmic_print_unity_testcase_info(7715,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(7715,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_BIST_PASS_INT,
                                      PMIC_IRQ_UNMASK);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_LP8764X_BIST_PASS_INT,
                                      PMIC_IRQ_UNMASK);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    /*
     * On J721E, Need to Mask NSLEEP1 and NSLEEP2 signal, becuase
     * NSLEEPn signals are having high priority than RUNTIME BIST PASS.
     */
    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {

        nsleepType = PMIC_NSLEEP1_SIGNAL;
        maskEnable = PMIC_NSLEEPX_MASK;

        pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                                 nsleepType,
                                                 maskEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        nsleepType = PMIC_NSLEEP2_SIGNAL;
        maskEnable = PMIC_NSLEEPX_MASK;

        pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                                 nsleepType,
                                                 maskEnable);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmRequestRuntimeBist(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_TPS6594X_BIST_PASS_INT/32U] &
                 (1U << (PMIC_TPS6594X_BIST_PASS_INT % 32U))) != 0U))
            {
                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    /* clear the interrupt */
                    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                      PMIC_TPS6594X_BIST_PASS_INT);
                    break;
                }
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_LP8764X_BIST_PASS_INT/32U] &
                 (1U << (PMIC_LP8764X_BIST_PASS_INT % 32U))) != 0U))
            {
                if(PMIC_ST_SUCCESS == pmicStatus)
                {
                    /* clear the interrupt */
                    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                      PMIC_LP8764X_BIST_PASS_INT);
                    break;
                }
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    pmic_testResultUpdate_pass(7715,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * \brief   Pmic_irqGetErrStatus : Test FSD_INT interrupt.
 *          To test this testcase, user needs to follow below steps
 *          1. After POR reset, run below test case by setting 'startup_type' variable to
 *             PMIC_FSD_STARTUP_TYPE.
 */
static void test_Pmic_FSD_interrupt(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    bool intrEnable       = PMIC_IRQ_UNMASK;

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NONE,
        PMIC_GPIO_HIGH
    };

    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(0xAB1F,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    /* Ignored because, it needs NVM setup */
    pmic_testResultUpdate_ignore(0xAB1F,
                                 pmic_misc_tests,
                                 PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB1F,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(startup_type == PMIC_FSD_STARTUP_TYPE)
    {
        pmic_log("FSD Start-up is detected\n");
    }
    else
    {
        status = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                              gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle, PMIC_TPS6594X_FSD_INT, intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }
        if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle, PMIC_LP8764X_FSD_INT, intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Set QA parameters */
        status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Waiting for longwindow time interval expiring */
        Osal_delay(wdgCfg.longWinDuration_ms);
    }

    pmic_testResultUpdate_pass(0xAB1F,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}
/*!
 * \brief   Pmic_irqGetErrStatus : Test NPWRON_INT interrupt Manual TestCase.
 *          To test this testcase, user needs to follow below steps
 *          1. After POR reset, run below test case by setting 'startup_type' variable to
 *             PMIC_NPWRON_STARTUP_TYPE.
 */
static void test_Pmic_NPWRON_interrupt(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    bool intrEnable       = PMIC_IRQ_UNMASK;

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON,
        PMIC_GPIO_HIGH
    };

    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        3000U,
        6150U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_7,
        PMIC_WDG_RESET_THRESHOLD_COUNT_7,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_DISABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_0,
        PMIC_WDG_QA_LFSR_VALUE_0,
        PMIC_WDG_QA_QUES_SEED_VALUE_10,
    };

    test_pmic_print_unity_testcase_info(0xAB2D,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB2D,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB2D,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(startup_type == PMIC_NPWRON_STARTUP_TYPE)
    {
        pmic_log("NPWRON Start-up is detected\n");
    }
    else
    {
        status = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                              gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON,
                                      intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

        /* Enable WDG Timer */
        status = Pmic_wdgEnable(pPmicCoreHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Set QA parameters */
        status = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        /* Waiting for longwindow time interval expiring */
        Osal_delay(wdgCfg.longWinDuration_ms);
    }

    pmic_testResultUpdate_pass(0xAB2D,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_irqGetErrStatus : Test NPWRON_LONG_INT interrupt Manual TestCase.
 *          To test this testcase, user needs to follow below steps
 *          1. User has to hold the NPWRON button or Keep low NPWON pin some duration.
 *          2. Pmic will be reset after first step.
 *          3. After reset, run below test case by setting 'startup_type' variable to
 *             PMIC_NPWRON_STARTUP_TYPE.
 */
static void test_Pmic_NPWRON_long_interrupt(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    bool intrEnable       = PMIC_IRQ_UNMASK;

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_NPWRON_PINFUNC_NPWRON,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB20,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(0xAB20,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(0xAB20,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(startup_type == PMIC_NPWRON_STARTUP_TYPE)
    {
        pmic_log("NPWRON Start-up is detected\n");
    }
    else
    {
        status = Pmic_gpioSetNPwronEnablePinConfiguration(pPmicCoreHandle,
                                                              gpioCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
        {
            status = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_NPWRON_LONG_INT,
                                      intrEnable);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        }

    }

    pmic_testResultUpdate_pass(0xAB20,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_commFrmError : Test COMM_FRM_ERROR interrupt Manual TestCase.
 *          To test this testcase, user needs to follow below steps
 *          1. PMIC SPI must be enabled by NVM.
 *          2. Run below test case to get an SPI FRAME Error
 */
static void test_Pmic_commFrmErrorIntr(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ = false;
    uint8_t buffLength = 1U;
    uint8_t txBuf[PMIC_IO_BUF_SIZE] = {0};
    uint8_t txData = 0x0FU;
    uint16_t regAddr = PMIC_SCRATCH_PAD_REG_4_REGADDR;
    uint16_t pmicRegAddr = regAddr;
    uint8_t instType = PMIC_MAIN_INST;
    int8_t  timeout      = 10U;

    test_pmic_print_unity_testcase_info(0xAB2C,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if((NULL == pPmicCoreHandle) && (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        pmic_testResultUpdate_ignore(0xAB2C,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        /*
         * Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         * explained in PMIC TRM
         */

        buffLength = 0;
        /* Set ADDR to txbuf[0], Bits 1-8: ADDR[7:0] */
        txBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);
        buffLength++;

        /* Writing wrong value to Set PAGE to txBuf[1] 2:0 bits */
        txBuf[buffLength] = (uint8_t)((pmicRegAddr >> 8U) & 0x7U);

        /* Set R/W in txBuf[1] as bit-3, Bit 12: 0 for Write Request */
        txBuf[buffLength] &= (uint8_t)(~PMIC_IO_REQ_RW);
        buffLength++;

        if(((bool)true) == pPmicCoreHandle->crcEnable)
        {
            /* Set CRC data to txBuf[3], Bits 25-32 CRC */
            txBuf[buffLength] = Pmic_getCRC8Val(txBuf, buffLength);
            /* Increment 1 more byte to store CRC8 */
            buffLength++;
        }

    }

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_TPS6594X_COMM_FRM_ERR_INT,
                                      PMIC_IRQ_UNMASK);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_irqMaskIntr(pPmicCoreHandle,
                                      PMIC_LP8764X_COMM_FRM_ERR_INT,
                                      PMIC_IRQ_UNMASK);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    pmicStatus = test_pmic_regWrite(pPmicCoreHandle,
                                    instType,
                                    pmicRegAddr,
                                    txBuf,
                                    buffLength);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

            if(((errStat.intStatus[PMIC_TPS6594X_COMM_FRM_ERR_INT/32U] &
               (1U << (PMIC_TPS6594X_COMM_FRM_ERR_INT % 32U))) != 0U))
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_COMM_FRM_ERR_INT);
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        while(timeout--)
        {
            /* Delay added to avoid timeout */
            Osal_delay(1000);

            pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
            TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
            if((PMIC_ST_SUCCESS == pmicStatus) &&
               ((errStat.intStatus[PMIC_LP8764X_COMM_FRM_ERR_INT/32U] &
                 (1U << (PMIC_LP8764X_COMM_FRM_ERR_INT % 32U))) != 0U))
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_LP8764X_COMM_FRM_ERR_INT);
                break;
            }
        }

        if(0 > timeout)
        {
            pmicStatus = PMIC_ST_ERR_FAIL;
        }

        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    }

    pmic_testResultUpdate_pass(0xAB2C,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

#endif

/*!
 * \brief   Pmic_setScratchPadValue : Parameter validation for handle.
 */
static void test_pmic_setScratchPadValuePrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId,data;

    data = 0x10;
    scratchPadRegId = PMIC_SCRATCH_PAD_REG_4;

    test_pmic_print_unity_testcase_info(8321,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setScratchPadValue(NULL, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(8321,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setScratchPadValue : Parameter validation for scratchPadRegId.
 */
static void test_pmic_setScratchPadValuePrmValTest_scratchPadRegId(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId,data;

    data = 0x10;
    scratchPadRegId = PMIC_SCRATCH_PAD_REG_4 + 1;

    test_pmic_print_unity_testcase_info(8322,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(8322,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setScratchPadValue : Test Set/Get Scratchpad value.
 */
static void test_pmic_setScratchPadValue_setget(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId,data, data_rd, data_def;

    data = 0x10;
    scratchPadRegId = PMIC_SCRATCH_PAD_REG_4;


    test_pmic_print_unity_testcase_info(8323,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    for(scratchPadRegId = PMIC_SCRATCH_PAD_REG_1;
        scratchPadRegId <= PMIC_SCRATCH_PAD_REG_4; scratchPadRegId++)
    {
        status = Pmic_getScratchPadValue(pPmicCoreHandle,
                                         scratchPadRegId,
                                         &data_def);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_setScratchPadValue(pPmicCoreHandle,
                                         scratchPadRegId,
                                         data);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_getScratchPadValue(pPmicCoreHandle,
                                         scratchPadRegId,
                                         &data_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(data, data_rd);

        status = Pmic_setScratchPadValue(pPmicCoreHandle,
                                         scratchPadRegId,
                                         data_def);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_getScratchPadValue(pPmicCoreHandle,
                                         scratchPadRegId,
                                         &data_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(data_def, data_rd);
    }
    pmic_testResultUpdate_pass(8323,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getScratchPadValue : Parameter validation for handle.
 */
static void test_pmic_getScratchPadValuePrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId,data;

    scratchPadRegId = PMIC_SCRATCH_PAD_REG_4;

    test_pmic_print_unity_testcase_info(8324,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getScratchPadValue(NULL, scratchPadRegId, &data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(8324,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getScratchPadValue : Parameter validation for scratchPadRegId.
 */
static void test_pmic_getScratchPadValuePrmValTest_scratchPadRegId(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId,data;

    scratchPadRegId = PMIC_SCRATCH_PAD_REG_4 + 1;

    test_pmic_print_unity_testcase_info(8325,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getScratchPadValue(pPmicCoreHandle, scratchPadRegId, &data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(8325,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getScratchPadValue : Parameter validation for data.
 */
static void test_pmic_getScratchPadValuePrmValTest_data(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId;

    scratchPadRegId = PMIC_SCRATCH_PAD_REG_4 ;

    test_pmic_print_unity_testcase_info(8326,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getScratchPadValue(pPmicCoreHandle, scratchPadRegId, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(8326,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setUserSpareValue : Parameter validation for handle.
 */
static void test_pmic_setUserSpareValuePrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum, data;

    data = PMIC_USER_SPARE_REG_VAL_1;
    userSpareRegNum = PMIC_USER_SPARE_REG_4;

    test_pmic_print_unity_testcase_info(9915,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setUserSpareValue(NULL, userSpareRegNum, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(9915,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setUserSpareValue : Parameter validation for userSpareRegNum.
 */
static void test_pmic_setUserSpareValuePrmValTest_userSpareRegNum(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum, data;

    data = PMIC_USER_SPARE_REG_VAL_1;
    userSpareRegNum = PMIC_USER_SPARE_REG_4 + 1;

    test_pmic_print_unity_testcase_info(9916,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setUserSpareValue(pPmicCoreHandle, userSpareRegNum, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(9916,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setUserSpareValue : Parameter validation for userSpareRegNum.
 */
static void test_pmic_setUserSpareValuePrmValTest_data(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum, data;

    data = 0x2;
    userSpareRegNum = PMIC_USER_SPARE_REG_4;

    test_pmic_print_unity_testcase_info(9917,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setUserSpareValue(pPmicCoreHandle, userSpareRegNum, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(9917,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setUserSpareValue : Test Set/Get userSpare Reg value.
 */
static void test_pmic_setUserSpareValue(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum, data, data_rd;

    data = PMIC_USER_SPARE_REG_VAL_1;
    userSpareRegNum = PMIC_USER_SPARE_REG_4;

    test_pmic_print_unity_testcase_info(9918,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    for(userSpareRegNum = PMIC_USER_SPARE_REG_1;
        userSpareRegNum <= PMIC_USER_SPARE_REG_4; userSpareRegNum++)
    {
        status = Pmic_setUserSpareValue(pPmicCoreHandle,
                                        userSpareRegNum,
                                        data);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

        status = Pmic_getUserSpareValue(pPmicCoreHandle,
                                         userSpareRegNum,
                                         &data_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        TEST_ASSERT_EQUAL(data, data_rd);
    }
    pmic_testResultUpdate_pass(9918,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getUserSpareValue : Parameter validation for handle.
 */
static void test_pmic_getUserSpareValuePrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum, data;

    userSpareRegNum = PMIC_USER_SPARE_REG_4;

    test_pmic_print_unity_testcase_info(9919,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getUserSpareValue(NULL, userSpareRegNum, &data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(9919,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getUserSpareValue : Parameter validation for userSpareRegNum.
 */
static void test_pmic_getUserSpareValuePrmValTest_userSpareRegNum(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum, data;

    userSpareRegNum = PMIC_USER_SPARE_REG_4 + 1;

    test_pmic_print_unity_testcase_info(9920,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getUserSpareValue(pPmicCoreHandle, userSpareRegNum, &data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(9920,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getUserSpareValue : Parameter validation for data.
 */
static void test_pmic_getUserSpareValuePrmValTest_data(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t userSpareRegNum;

    userSpareRegNum = PMIC_USER_SPARE_REG_4 ;

    test_pmic_print_unity_testcase_info(9921,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getUserSpareValue(pPmicCoreHandle, userSpareRegNum, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(9921,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Enable/Disable Spread Spectrum
 */
static void test_pmic_setCommonCtrlCfg_sreadSpectrumEnable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_def = {PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(9922,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.sreadSpectrumEn,
                      commonCtrlCfg_rd.sreadSpectrumEn);

    commonCtrlCfg.sreadSpectrumEn = PMIC_SPREAD_SPECTRUM_CFG_DISABLE;
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.sreadSpectrumEn,
                      commonCtrlCfg_rd.sreadSpectrumEn);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg_def.sreadSpectrumEn,
                      commonCtrlCfg_rd.sreadSpectrumEn);

    pmic_testResultUpdate_pass(9922,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Lock and UnLock Registers
 */
static void test_pmic_setCommonCtrlCfg_RegisterLock(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_REG_LOCK_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_LOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT};

    test_pmic_print_unity_testcase_info(9923,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);


    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_LOCK,
                      commonCtrlStat_rd.regLockStat);

    commonCtrlCfg.regLock = PMIC_REGISTER_UNLOCK;
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_UNLOCK,
                      commonCtrlStat_rd.regLockStat);

    pmic_testResultUpdate_pass(9923,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Configure Spread Spectrum Depth
 */
static void test_pmic_setCommonCtrlCfg_sreadSpectrumDepth(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_def = {PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    uint8_t ssDepthVal;

    test_pmic_print_unity_testcase_info(9924,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(ssDepthVal = PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE;
        ssDepthVal <= PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_8_4_PERCENT;
        ssDepthVal++)
    {
        commonCtrlCfg.spreadSpectrumDepth = ssDepthVal;

        pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(commonCtrlCfg.spreadSpectrumDepth,
                          commonCtrlCfg_rd.spreadSpectrumDepth);
    }

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg_def.spreadSpectrumDepth,
                      commonCtrlCfg_rd.spreadSpectrumDepth);

    pmic_testResultUpdate_pass(9924,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlConfig : Parameter validation for handle
 */
static void test_pmic_getCommonCtrlCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9925,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlConfig(NULL, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9925,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlConfig : Parameter validation for pCommonCtrlCfg
 */
static void test_pmic_getCommonCtrlCfgPrmValTest_pCommonCtrlCfg(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9926,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9926,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Parameter validation for eepromDefaultLoad
 */
static void test_pmic_setCommonCtrlCfgPrmValTest_eepromDefaultLoad(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        2U,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(9927,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9927,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Parameter validation for regLock
 */
static void test_pmic_setCommonCtrlCfgPrmValTest_regLock(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_REG_LOCK_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        0x20U,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(9928,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9928,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Parameter validation for spreadSpectrumDepth
 */
static void test_pmic_setCommonCtrlCfgPrmValTest_spreadSpectrumDepth(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        3U
    };

    test_pmic_print_unity_testcase_info(9929,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9929,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Parameter validation for handle
 */
static void test_pmic_setCommonCtrlCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(9930,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setCommonCtrlConfig(NULL, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9930,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Enable/Disable Band gap Voltage to AMUX OUT/RFF OUT Pin
 */
static void test_pmic_setMiscCtrlCfg_amuxOutRefOutEn(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_def = {PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9931,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        miscCtrlCfg.amuxOutRefOutEn = PMIC_LP8764X_REF_OUT_PIN_CFG_DISABLE;
    }

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.amuxOutRefOutEn,
                      miscCtrlCfg_rd.amuxOutRefOutEn);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        miscCtrlCfg.amuxOutRefOutEn = PMIC_TPS6594X_AMUX_OUT_PIN_CFG_ENABLE;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        miscCtrlCfg.amuxOutRefOutEn = PMIC_LP8764X_REF_OUT_PIN_CFG_ENABLE;
    }

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.amuxOutRefOutEn,
                      miscCtrlCfg_rd.amuxOutRefOutEn);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg_def.amuxOutRefOutEn,
                      miscCtrlCfg_rd.amuxOutRefOutEn);

    pmic_testResultUpdate_pass(9931,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Enable/Disable internal Clock Monitoring
 */
static void test_pmic_setMiscCtrlCfg_clkMonEn(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_CLK_MON_EN_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_def = {PMIC_CFG_CLK_MON_EN_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_CLK_MON_EN_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9932,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.clkMonEn,
                      miscCtrlCfg_rd.clkMonEn);

    miscCtrlCfg.clkMonEn = PMIC_INTERNAL_CLK_MONITORING_CFG_ENABLE;

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.clkMonEn,
                      miscCtrlCfg_rd.clkMonEn);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg_def.clkMonEn,
                      miscCtrlCfg_rd.clkMonEn);

    pmic_testResultUpdate_pass(9932,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Selects SYNCCLKOUT Frequency
 */
static void test_pmic_setMiscCtrlCfg_syncClkOutFreqSel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_def = {PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };
    uint8_t    freqSel;

    test_pmic_print_unity_testcase_info(9933,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(freqSel = PMIC_SYNCCLKOUT_DISABLE;
        freqSel <= PMIC_SYNCCLKOUT_4_4_MHZ;
        freqSel++)
    {
        miscCtrlCfg.syncClkOutFreqSel = freqSel;

        pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(miscCtrlCfg.syncClkOutFreqSel,
                          miscCtrlCfg_rd.syncClkOutFreqSel);
    }

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg_def.syncClkOutFreqSel,
                      miscCtrlCfg_rd.syncClkOutFreqSel);

    pmic_testResultUpdate_pass(9933,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : External clock Selection
 */
static void test_pmic_setMiscCtrlCfg_extClkSel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_def = {PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9934,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.extClkSel,
                      miscCtrlCfg_rd.extClkSel);

    miscCtrlCfg.extClkSel = PMIC_AUTOMATIC_EXT_CLK;

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.extClkSel,
                      miscCtrlCfg_rd.extClkSel);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg_def.extClkSel,
                      miscCtrlCfg_rd.extClkSel);

    pmic_testResultUpdate_pass(9934,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Selects External clock Frequency
 */
static void test_pmic_setMiscCtrlCfg_syncClkInFreq(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg_def = {PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };
    uint8_t syncClkInFreq, maxVal;

    test_pmic_print_unity_testcase_info(9935,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        syncClkInFreq = PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ;
        maxVal = PMIC_TPS6594X_SYNCCLKIN_4_4_MHZ;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        syncClkInFreq = PMIC_LP8764X_SYNCCLKIN_1_1_MHZ;
        maxVal = PMIC_LP8764X_SYNCCLKIN_8_8_MHZ;
    }

    for(;syncClkInFreq <= maxVal;syncClkInFreq++)
    {
        pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(miscCtrlCfg.syncClkInFreq,
                          miscCtrlCfg_rd.syncClkInFreq);
    }

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg_def.syncClkInFreq,
                      miscCtrlCfg_rd.syncClkInFreq);

    pmic_testResultUpdate_pass(9935,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Parameter validation for syncClkOutFreqSel
 */
static void test_pmic_setMiscCtrlCfgPrmValTest_syncClkOutFreqSel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        4U,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9936,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9936,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Parameter validation for extClkSel
 */
static void test_pmic_setMiscCtrlCfgPrmValTest_extClkSel(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        2U,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9937,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9937,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Parameter validation for syncClkInFreq
 */
static void test_pmic_setMiscCtrlCfgPrmValTest_syncClkInFreq(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        3U,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        miscCtrlCfg.syncClkInFreq = 4U;
    }
    test_pmic_print_unity_testcase_info(9938,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9938,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Parameter validation for handle
 */
static void test_pmic_setMiscCtrlCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9939,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(NULL, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9939,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getMiscCtrlConfig : Parameter validation for handle
 */
static void test_pmic_getMiscCtrlCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9940,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(NULL, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9940,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getMiscCtrlConfig : Parameter validation for pMiscCtrlCfg
 */
static void test_pmic_getMiscCtrlCfgPrmValTest_pMiscCtrlCfg(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9941,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9941,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Enable/Disable Backup Battery Charging
 */
static void test_pmic_setBatteryCtrlCfg_chargingEn(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_rd = {PMIC_CFG_CHARGING_EN_VALID_SHIFT,};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_def = {PMIC_CFG_CHARGING_EN_VALID_SHIFT,};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_CHARGING_EN_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };

    test_pmic_print_unity_testcase_info(9942,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9942,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(batteryCtrlCfg.chargingEn,
                      batteryCtrlCfg_rd.chargingEn);


    batteryCtrlCfg.chargingEn = PMIC_TPS6594X_BB_CHARGINGING_CFG_ENABLE;

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(batteryCtrlCfg.chargingEn,
                      batteryCtrlCfg_rd.chargingEn);

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(batteryCtrlCfg_def.chargingEn,
                      batteryCtrlCfg_rd.chargingEn);

    pmic_testResultUpdate_pass(9942,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Configure Backup Battery configuration
 *          for End of charge Voltage
 */
static void test_pmic_setBatteryCtrlCfg_endOfChargeVoltage(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_rd = {PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_def = {PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };
    uint8_t endOfChargeVoltVal;

    test_pmic_print_unity_testcase_info(9943,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9943,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(endOfChargeVoltVal = PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V;
        endOfChargeVoltVal <= PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_3_3_V;
        endOfChargeVoltVal++)
    {
        batteryCtrlCfg.endOfChargeVoltage = endOfChargeVoltVal;

        pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle,
                                               &batteryCtrlCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(batteryCtrlCfg.endOfChargeVoltage,
                          batteryCtrlCfg_rd.endOfChargeVoltage);
    }

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(batteryCtrlCfg_def.endOfChargeVoltage,
                      batteryCtrlCfg_rd.endOfChargeVoltage);

    pmic_testResultUpdate_pass(9943,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Configure Backup Battery charging
 *          current value
 */
static void test_pmic_setBatteryCtrlCfg_chargeCurrent(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_rd = {PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT,};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_def = {PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT,};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };
    uint8_t  chargeCurrentVal;

    test_pmic_print_unity_testcase_info(9944,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9944,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    for(chargeCurrentVal = PMIC_TPS6594X_BB_CHARGING_CURRENT_100;
        chargeCurrentVal <= PMIC_TPS6594X_BB_CHARGING_CURRENT_500;
        chargeCurrentVal++)
    {
        pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle,
                                               &batteryCtrlCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(batteryCtrlCfg.chargeCurrent,
                          batteryCtrlCfg_rd.chargeCurrent);
    }

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(batteryCtrlCfg_def.chargeCurrent,
                      batteryCtrlCfg_rd.chargeCurrent);

    pmic_testResultUpdate_pass(9944,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Parameter validation for endOfChargeVoltage
 */
static void test_pmic_setBatteryCtrlCfgPrmValTest_endOfChargeVoltage(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        4U,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };

    test_pmic_print_unity_testcase_info(9945,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9945,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9945,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Parameter validation for chargeCurrent
 */
static void test_pmic_setBatteryCtrlCfgPrmValTest_chargeCurrent(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        2U
    };

    test_pmic_print_unity_testcase_info(9946,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9946,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9946,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Parameter validation for handle
 */
static void test_pmic_setBatteryCtrlCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };

    test_pmic_print_unity_testcase_info(9948,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9948,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setBatteryCtrlConfig(NULL, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9948,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getBatteryCtrlConfig : Parameter validation for handle
 */
static void test_pmic_getBatteryCtrlCfgPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_rd = {PMIC_CFG_CHARGING_EN_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9949,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9949,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getBatteryCtrlConfig(NULL, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9949,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getBatteryCtrlConfig : Parameter validation for pBatteryCtrlCfg
 */
static void test_pmic_getBatteryCtrlCfgPrmValTest_pBatteryCtrlCfg(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9950,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9950,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9950,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of nIntPin
 */
static void test_pmic_getCommonCtrlStat_nIntPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_NINT_PIN_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9951,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_LOW,
                      commonCtrlStat_rd.nIntPin);

    pmic_testResultUpdate_pass(9951,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of spmiLpmStat
 */
static void test_pmic_getCommonCtrlStat_spmiLpmStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9952,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_SPMI_LPM_MODE_CTRL_CFG_DISABLED,
                      commonCtrlStat_rd.spmiLpmStat);

    pmic_testResultUpdate_pass(9952,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of forceEnDrvLowStat
 */
static void test_pmic_getCommonCtrlStat_forceEnDrvLowStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9953,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_ENABLE_DRV_I2C_SPI_CONFIG_ENABLE,
                      commonCtrlStat_rd.forceEnDrvLowStat);

    pmic_testResultUpdate_pass(9953,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of bbEndOfChargeIndication
 */
static void test_pmic_getCommonCtrlStat_bbEndOfChargeIndication(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_BB_EOC_INDICATION_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9954,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9954,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }
    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_TPS6594X_BB_EOC_STATUS_NOT_ENABLED,
                      commonCtrlStat_rd.bbEndOfChargeIndication);

    pmic_testResultUpdate_pass(9954,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of regLockStat
 */
static void test_pmic_getCommonCtrlStat_regLockStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9955,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_UNLOCK,
                      commonCtrlStat_rd.regLockStat);

    pmic_testResultUpdate_pass(9955,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of extClkValidity
 */
static void test_pmic_getCommonCtrlStat_extClkValidity(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9956,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_log("\r\n extClkValidity status %d", commonCtrlStat_rd.extClkValidity);

    pmic_testResultUpdate_pass(9956,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of startupPin
 */
static void test_pmic_getCommonCtrlStat_startupPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_STARTUP_PIN_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9957,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_HIGH,
                      commonCtrlStat_rd.startupPin);

    pmic_testResultUpdate_pass(9957,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of enDrvPin
 */
static void test_pmic_getCommonCtrlStat_enDrvPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_EN_DRV_PIN_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9958,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_LOW,
                      commonCtrlStat_rd.enDrvPin);

    pmic_testResultUpdate_pass(9958,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of nRstOutSocPin
 */
static void test_pmic_getCommonCtrlStat_nRstOutSocPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9959,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_LOW,
                      commonCtrlStat_rd.nRstOutSocPin);

    pmic_testResultUpdate_pass(9959,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Get the status of nRstOutPin
 */
static void test_pmic_getCommonCtrlStat_nRstOutPin(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_NRSTOUT_PIN_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9960,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_LOW,
                      commonCtrlStat_rd.nRstOutPin);

    pmic_testResultUpdate_pass(9960,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Parameter validation for handle
 */
static void test_pmic_getCommonCtrlStatPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_NRSTOUT_PIN_STAT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9961,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(NULL, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9961,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlStat : Parameter validation for pCommonCtrlStat
 */
static void test_pmic_getCommonCtrlStatPrmValTest_pCommonCtrlStat(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9962,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9962,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getI2CSpeed :  Parameter validation for handle
 */
static void test_pmic_getI2CSpeedPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t i2c1Speed, i2c2Speed;

    test_pmic_print_unity_testcase_info(9963,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getI2CSpeed(NULL, &i2c1Speed, &i2c2Speed);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9963,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_getI2CSpeed :  Parameter validation for pI2C1Speed
 */
static void test_pmic_getI2CSpeedPrmValTest_pI2C1Speed(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t i2c2Speed;

    test_pmic_print_unity_testcase_info(9964,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getI2CSpeed(pPmicCoreHandle, NULL, &i2c2Speed);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9964,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getI2CSpeed :  Parameter validation for pI2C2Speed
 */
static void test_pmic_getI2CSpeedPrmValTest_pI2C2Speed(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t i2c1Speed;

    test_pmic_print_unity_testcase_info(9965,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getI2CSpeed(pPmicCoreHandle, &i2c1Speed, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9965,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getI2CSpeed :  Get I2C1 or I2C2 Speed value
 */
static void test_pmic_getI2CSpeed(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t i2c1Speed, i2c2Speed;

    test_pmic_print_unity_testcase_info(9966,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        pmic_testResultUpdate_ignore(9966,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getI2CSpeed(pPmicCoreHandle, &i2c1Speed, &i2c2Speed);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_I2C_STANDARD_MODE, i2c1Speed);

    if(pPmicCoreHandle->commMode == PMIC_INTF_DUAL_I2C)
    {
        TEST_ASSERT_EQUAL(PMIC_I2C_STANDARD_MODE, i2c2Speed);
    }

    pmic_testResultUpdate_pass(9966,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getDeviceInfo :  Get the Device Information
 */
static void test_pmic_getDeviceInfo(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_DeviceInfo_t deviceInfo;

    test_pmic_print_unity_testcase_info(9967,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getDeviceInfo(pPmicCoreHandle, &deviceInfo);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_log("\r\n deviceID 0x%x nvmID 0x%x nvmRev 0x%x siliconRev 0x%x customNvmID 0x%x\r\n", \
                 deviceInfo.deviceID, deviceInfo.nvmID, deviceInfo.nvmRev, deviceInfo.siliconRev, deviceInfo.customNvmID);
    }
    else
    {
        pmic_log("\r\n deviceID 0x%x nvmID 0x%x nvmRev 0x%x siliconRev 0x%x \r\n", \
                 deviceInfo.deviceID, deviceInfo.nvmID, deviceInfo.nvmRev, deviceInfo.siliconRev);
    }

    pmic_testResultUpdate_pass(9967,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getDeviceInfo :  Parameter validation for handle
 */
static void test_pmic_getDeviceInfoPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_DeviceInfo_t deviceInfo;

    test_pmic_print_unity_testcase_info(9968,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getDeviceInfo(NULL, &deviceInfo);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9968,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getDeviceInfo :  Parameter validation for pDeviceInfo
 */
static void test_pmic_getDeviceInfoPrmValTest_pDeviceInfo(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9969,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getDeviceInfo(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9969,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getPinValue : Get the status of EN_DRV Pin Value
 */
static void test_pmic_getPinValue_enableDrv(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue;

    test_pmic_print_unity_testcase_info(9970,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_LOW, pinValue);

    pmic_testResultUpdate_pass(9970,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getPinValue : Get the status of NRSTOUT_SOC Pin Value
 */
static void test_pmic_getPinValue_nRstOutSoc(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue;

    test_pmic_print_unity_testcase_info(9971,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_NRSTOUT_SOC,
                                  &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_HIGH,
                      pinValue);

    pmic_testResultUpdate_pass(9971,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getPinValue : Get the status of NRSTOUT Pin Value
 */
static void test_pmic_getPinValue_nRstOut(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue;

    test_pmic_print_unity_testcase_info(9972,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_NRSTOUT,
                                  &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_HIGH,
                      pinValue);

    pmic_testResultUpdate_pass(9972,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getPinValue : Parameter validation for pinType
 */
static void test_pmic_getPinValuePrmValTest_pinType(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue;
    uint8_t pinType = 3U;

    test_pmic_print_unity_testcase_info(9973,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  pinType,
                                  &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);


    pmic_testResultUpdate_pass(9973,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getPinValue : Parameter validation for handle
 */
static void test_pmic_getPinValuePrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    uint8_t pinValue;

    test_pmic_print_unity_testcase_info(9974,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getPinValue(NULL,
                                  PMIC_PIN_TYPE_NRSTOUT,
                                  &pinValue);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);


    pmic_testResultUpdate_pass(9974,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getPinValue : Parameter validation for pPinValue
 */
static void test_pmic_getPinValuePrmValTest_pPinValue(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(9975,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_NRSTOUT,
                                  NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);


    pmic_testResultUpdate_pass(9975,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Configure NRSTOUT_SOC Signal
 */
static void test_pmic_setMiscCtrlCfg_nRstOutSocSignal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };
    int8_t num = 0;

    test_pmic_print_unity_testcase_info(9978,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    if((PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType))
    {
        /* Ignore the test on J721E PG1.0 Not planning to debug this issue as
           the same test is working fine for J721E PG2.0 */
        pmic_testResultUpdate_ignore(9978,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }
#endif

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.nRstOutSocSignal,
                      miscCtrlCfg_rd.nRstOutSocSignal);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and it should be High");
#endif

    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    pmic_log("\r\n SOC will go to reset\n");
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and it should be Low");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and it should be Low");
#endif

    miscCtrlCfg.nRstOutSocSignal = PMIC_PIN_SIGNAL_LEVEL_LOW;
    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.nRstOutSocSignal,
                      miscCtrlCfg_rd.nRstOutSocSignal);

    pmic_testResultUpdate_pass(9978,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Configure NRSTOUT Signal
 */
static void test_pmic_setMiscCtrlCfg_nRstOutSignal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_NRSTOUT_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_NRSTOUT_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };
    int8_t num = 0;

    test_pmic_print_unity_testcase_info(9979,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.nRstOutSignal,
                      miscCtrlCfg_rd.nRstOutSignal);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP29 and it should be High");
#endif

    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    pmic_log("\r\n MCU will go to reset\n");
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP133 and it should be Low");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP29 and it should be Low");
#endif

    miscCtrlCfg.nRstOutSignal = PMIC_PIN_SIGNAL_LEVEL_LOW;
    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.nRstOutSignal,
                      miscCtrlCfg_rd.nRstOutSignal);

    pmic_testResultUpdate_pass(9979,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Parameter validation for NRSTOUT_SOC Signal
 */
static void test_pmic_setMiscCtrlCfgPrmValTest_nRstOutSocSignal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        2U,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(9980,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9980,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Parameter validation for NRSTOUT Signal
 */
static void test_pmic_setMiscCtrlCfgPrmValTest_nRstOutSignal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_NRSTOUT_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        2U
    };

    test_pmic_print_unity_testcase_info(9981,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9981,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getMiscCtrlConfig : Get the status of NRSTOUT_SOC Signal
 */
static void test_pmic_getMiscCtrlCfg_nRstOutSocSignal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9982,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_HIGH,
                      miscCtrlCfg_rd.nRstOutSocSignal);

    pmic_testResultUpdate_pass(9982,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getMiscCtrlConfig : Get the status of NRSTOUT Signal
 */
static void test_pmic_getMiscCtrlCfg_nRstOutSignal(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_NRSTOUT_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(9983,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_PIN_SIGNAL_LEVEL_HIGH,
                      miscCtrlCfg_rd.nRstOutSignal);

    pmic_testResultUpdate_pass(9983,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Configure EN_DRV Signal */
static void test_pmic_setCommonCtrlCfg_enDrv(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_ENABLE_DRV_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_def = {PMIC_CFG_ENABLE_DRV_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_ENABLE_DRV_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    uint8_t   pinValue;
    Pmic_IrqStatus_t errStat  = {0U};
    int8_t num = 0;

    test_pmic_print_unity_testcase_info(9984,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, PMIC_ESM_MODE_SOC, PMIC_ESM_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, commonCtrlCfg_rd.enDrv);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, pinValue);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP105 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP64 and it should be High");
#endif

    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    commonCtrlCfg.enDrv = PMIC_PIN_SIGNAL_LEVEL_LOW;
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, commonCtrlCfg_rd.enDrv);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, pinValue);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP105 and it should be Low");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP64 and it should be Low");
#endif

    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(commonCtrlCfg_def.enDrv, commonCtrlCfg_rd.enDrv);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);
    TEST_ASSERT_EQUAL(commonCtrlCfg_def.enDrv, pinValue);

    pmic_testResultUpdate_pass(9984,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Configure EN_DRV Signal - code coverage */
static void test_pmic_setCommonCtrlCfg_enDrv_codecoverage(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_ENABLE_DRV_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg_def = {PMIC_CFG_ENABLE_DRV_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_ENABLE_DRV_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    uint8_t   pinValue;
    Pmic_IrqStatus_t errStat  = {0U};

    test_pmic_print_unity_testcase_info(10786,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, PMIC_ESM_MODE_MCU, PMIC_ESM_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, PMIC_ESM_MODE_SOC, PMIC_ESM_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, commonCtrlCfg_rd.enDrv);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, pinValue);

    commonCtrlCfg.enDrv = PMIC_PIN_SIGNAL_LEVEL_LOW;
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, commonCtrlCfg_rd.enDrv);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);
    TEST_ASSERT_EQUAL(commonCtrlCfg.enDrv, pinValue);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg_def);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(commonCtrlCfg_def.enDrv, commonCtrlCfg_rd.enDrv);

    pmicStatus = Pmic_getPinValue(pPmicCoreHandle,
                                  PMIC_PIN_TYPE_EN_DRV,
                                  &pinValue);
    TEST_ASSERT_EQUAL(commonCtrlCfg_def.enDrv, pinValue);

    pmic_testResultUpdate_pass(10786,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Parameter validation for EN_DRV Signal
 */
static void test_pmic_setCommonCtrlCfgPrmValTest_enDrv(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_ENABLE_DRV_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        2U,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(9985,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9985,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Load from EEPROM defaults on RTC domain/conf Registers
 */
static void test_pmic_setCommonCtrlCfg_eepromDefaultLoadEnable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 1U, 30U, 6U, 0U, 1U};
    uint8_t  data = 0x67, testData;
    int8_t num = 0;

    if(g_skip_eeprom_test_flag == 0)
    {
        test_pmic_print_unity_testcase_info(9988,
                                            pmic_misc_tests,
                                            PMIC_MISC_NUM_OF_TESTCASES);
    }

    /*Refer the Bug PDK-10450 for more details*/
    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(9988,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        commonCtrlCfg.eepromDefaultLoad = PMIC_LP8764X_EEPROM_DEFAULTS_LOAD_TO_CONF_OTHER_REGS;
    }
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.eepromDefaultLoad,
                      commonCtrlCfg_rd.eepromDefaultLoad);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(data, testData);

    pmicStatus = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    if(g_skip_eeprom_test_flag == 0)
        pmic_log("\r\n Also check for PFSM delay 3 Value register same as EEPROM registers - 0");
    else if(g_skip_eeprom_test_flag == 2)
        pmic_log("\r\n Also check for Configured PFSM delay 3 Value register as 0x67 both in Leo and Hera PMIC");
    else
        pmic_log("\r\n Also check for PFSM delay 3 Value register same as EEPROM registers - 0 both in Leo and Hera PMIC");

    pmicStatus = Pmic_rtcEnableTimerIntr(pPmicCoreHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals for LP Stand-By State */
    Osal_delay(10U);

    pmicStatus =  Pmic_fsmSetMissionState(pPmicCoreHandle, PMIC_FSM_LP_STANBY_STATE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(g_skip_eeprom_test_flag == 0)
    {
        pmic_testResultUpdate_pass(9988,
                                   pmic_misc_tests,
                                   PMIC_MISC_NUM_OF_TESTCASES);
    }
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Disable Load from EEPROM defaults on RTC domain/conf Registers
 */
static void test_pmic_setCommonCtrlCfg_eepromDefaultLoadDisable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {0U};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_NOT_LOADED_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 1U, 30U, 6U, 0U, 1U};
    uint8_t  data = 0x67, testData;
    int8_t num = 0;

    if(g_skip_eeprom_test_flag == 0)
    {
        test_pmic_print_unity_testcase_info(9989,
                                            pmic_misc_tests,
                                            PMIC_MISC_NUM_OF_TESTCASES);
    }

    /*Refer the Bug PDK-10450 for more details*/
    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        pmic_testResultUpdate_ignore(9989,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        commonCtrlCfg.eepromDefaultLoad = PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS;
    }
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    commonCtrlCfg_rd.validParams = PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT;

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.eepromDefaultLoad,
                      commonCtrlCfg_rd.eepromDefaultLoad);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(data, testData);

    pmicStatus = Pmic_rtcSetTimerPeriod(pPmicCoreHandle, PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_rtcSetTimeDateInfo(pPmicCoreHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    if(g_skip_eeprom_test_flag == 0)
        pmic_log("\r\n Also check for Configured PFSM delay 3 Value register as 0x67");
    else
        pmic_log("\r\n Also check for Configured PFSM delay 3 Value register as 0x67 both in Leo and Hera PMIC");

    pmicStatus = Pmic_rtcEnableTimerIntr(pPmicCoreHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmSetNsleepSignalMask(pPmicCoreHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals for LP Stand-By State */
    Osal_delay(10U);

    pmicStatus =  Pmic_fsmSetMissionState(pPmicCoreHandle, PMIC_FSM_LP_STANBY_STATE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    if(g_skip_eeprom_test_flag == 0)
    {
        pmic_testResultUpdate_pass(9989,
                                   pmic_misc_tests,
                                   PMIC_MISC_NUM_OF_TESTCASES);
    }
}

#if defined(SOC_J7200)
static void test_pmic_setCommonCtrlCfg_hera_skipEepromDefLdDisable_eePromDefLdDisable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    uint8_t data = 0x67, testData;
    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_LPSTANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    pmicStatus = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(fsmCfg.lpStandbySel, fsmCfg_rd.lpStandbySel);

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.eepromDefaultLoad,
                      commonCtrlCfg_rd.eepromDefaultLoad);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    pmic_log("\r\n Default Val - PMIC_PFSM_DELAY3  0x%x", testData);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
     pmic_log("\r\n Configured Val - PMIC_PFSM_DELAY3  0x%x", testData);
}

static void test_pmic_setCommonCtrlCfg_hera_skipEepromDefLdDisable_eePromDefLdEnable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_LP8764X_EEPROM_DEFAULTS_LOAD_TO_CONF_OTHER_REGS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    uint8_t data = 0x67, testData;
    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_LPSTANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    pmicStatus = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(fsmCfg.lpStandbySel, fsmCfg_rd.lpStandbySel);


    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.eepromDefaultLoad,
                      commonCtrlCfg_rd.eepromDefaultLoad);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_log("\r\n Default Val - PMIC_PFSM_DELAY3  0x%x", testData);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

     pmic_log("\r\n Configured Val - PMIC_PFSM_DELAY3  0x%x", testData);
}

#if 0 // Refer the Bug PDK-10451 for more details
static void test_pmic_setCommonCtrlCfg_hera_skipEepromDefLdEnable_eePromDefLdEnable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_ENABLED,
        PMIC_LP8764X_EEPROM_DEFAULTS_LOAD_TO_CONF_OTHER_REGS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    uint8_t data = 0x67, testData;
    Pmic_FsmCfg_t fsmCfg_rd = {PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,};
    Pmic_FsmCfg_t fsmCfg =
    {
        PMIC_FSM_CFG_LP_STANDBYSEL_VALID_SHIFT,
        PMIC_FSM_FAST_BIST_DISABLE,
        PMIC_FSM_SELECT_LPSTANDBY_STATE,
        PMIC_FSM_ILIM_INT_FSMCTRL_DISABLE,
        PMIC_FSM_STARTUPDEST_ACTIVE
    };

    pmicStatus = Pmic_fsmSetConfiguration(pPmicCoreHandle, fsmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetConfiguration(pPmicCoreHandle, &fsmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(fsmCfg.lpStandbySel, fsmCfg_rd.lpStandbySel);


    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.eepromDefaultLoad,
                      commonCtrlCfg_rd.eepromDefaultLoad);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_log("\r\n Default Val - PMIC_PFSM_DELAY3  0x%x", testData);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY3,
                                      &testData);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

     pmic_log("\r\n Configured Val - PMIC_PFSM_DELAY3  0x%x", testData);
}
#endif

#endif

/*!
 * \brief   Test Pmic Write Protection when Register is Lock
 */
static void test_pmic_WriteProtection_RegisterLock(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS, status;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_REG_LOCK_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_LOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT};
    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc;
    uint8_t  irqNum1, irqNum2;
    Pmic_RecovCntCfg_t recovCntCfg;

    test_pmic_print_unity_testcase_info(9994,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);


    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_LOCK,
                      commonCtrlStat_rd.regLockStat);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_TPS6594X_REGULATOR_RV_SEL_ENABLE;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
        irqNum1 = PMIC_TPS6594X_ESM_MCU_RST_INT;
        irqNum2 = PMIC_TPS6594X_BUCK1_OV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_LP8764X_REGULATOR_VMON_RV_SEL_ENABLE;
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
        irqNum1 = PMIC_LP8764X_ESM_MCU_RST_INT;
        irqNum2 = PMIC_LP8764X_BUCK1_OV_INT;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_REG_LOCKED_WR_FAIL, pmicStatus);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY4,
                                      0x67);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_REG_LOCKED_WR_FAIL, pmicStatus);

    recovCntCfg.validParams    = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg.thrVal         = PMIC_RECOV_CNT_THR_MAX;

    status = Pmic_setRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_REG_LOCKED_WR_FAIL, status);

    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                      irqNum1);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        status = PMIC_ST_SUCCESS;
    }
    else
    {
        status = PMIC_ST_ERR_REG_LOCKED_WR_FAIL;
    }

    TEST_ASSERT_EQUAL(status, pmicStatus);

    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                      irqNum2);
    TEST_ASSERT_EQUAL(status, pmicStatus);

    commonCtrlCfg.regLock = PMIC_REGISTER_UNLOCK;
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_UNLOCK,
                      commonCtrlStat_rd.regLockStat);

    pmic_testResultUpdate_pass(9994,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test Pmic Write Protection when Register is Lock
 */
static void test_pmic_WriteProtection_RegisterLock_codecoverage(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_REG_LOCK_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_LOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT};
    Pmic_WdgCfg_t wdgCfg;

    test_pmic_print_unity_testcase_info(10752,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);


    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_LOCK,
                      commonCtrlStat_rd.regLockStat);

    wdgCfg.validParams = PMIC_CFG_WDG_LONGWINDURATION_VALID_SHIFT;

    wdgCfg.longWinDuration_ms = 750000U;

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    commonCtrlCfg.regLock = PMIC_REGISTER_UNLOCK;
    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_UNLOCK,
                      commonCtrlStat_rd.regLockStat);

    pmic_testResultUpdate_pass(10752,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test Pmic Write Protection when Register is UnLock
 */
static void test_pmic_WriteProtection_RegisterUnLock(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_REG_LOCK_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT};
    Pmic_PowerResourceCfg_t pPowerCfg =
    {
        PMIC_CFG_REGULATOR_VMON_RV_SEL_VALID_SHIFT,
    };
    uint16_t pwrRsrc;
    uint8_t  irqNum1, irqNum2;

    test_pmic_print_unity_testcase_info(9995,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);


    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(PMIC_REGISTER_STATUS_UNLOCK,
                      commonCtrlStat_rd.regLockStat);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_TPS6594X_REGULATOR_RV_SEL_ENABLE;
        pwrRsrc = PMIC_TPS6594X_REGULATOR_BUCK1;
        irqNum1 = PMIC_TPS6594X_ESM_MCU_RST_INT;
        irqNum2 = PMIC_TPS6594X_BUCK1_OV_INT;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.rvCheckEn = PMIC_LP8764X_REGULATOR_VMON_RV_SEL_ENABLE;
        pwrRsrc = PMIC_LP8764X_REGULATOR_BUCK1;
        irqNum1 = PMIC_LP8764X_ESM_MCU_RST_INT;
        irqNum2 = PMIC_LP8764X_BUCK1_OV_INT;
    }

    pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                             pwrRsrc,
                                             pPowerCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_fsmSetPfsmDelay(pPmicCoreHandle,
                                      PMIC_PFSM_DELAY4,
                                      0x67);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                      irqNum1);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                      irqNum2);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(9995,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getRecoveryCnt : Test to read recovery counter value
 */
static void test_pmic_getRecoveryCntVal(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t recovCntVal = 0U;
    uint8_t recovCntVal_rd;

    test_pmic_print_unity_testcase_info(10573,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCnt(pPmicCoreHandle, &recovCntVal_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    TEST_ASSERT_EQUAL(recovCntVal_rd, recovCntVal);

    pmic_testResultUpdate_pass(10573,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getRecoveryCnt : Parameter validation for pRecovCntVal
 */
static void test_pmic_getRecoveryCntPrmValTest_pRecovCntVal(void)
{
    int32_t status  = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10574,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getRecoveryCnt(pPmicCoreHandle, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10574,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for pPmicCoreHandle
 */
static void test_pmic_initPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0};

    test_pmic_print_unity_testcase_info(10575,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_init(&pmicCfgData, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10575,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for pPmicConfigData
 */
static void test_pmic_initPrmValTest_pmicCfgData(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10577,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_init(NULL, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10577,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for pmicDeviceType
 */
static void test_pmic_initPrmValTest_pmicDevType(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10578,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;
    pmicCfgData.pmicDeviceType = 2U;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10578,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for Interface mode
 */
static void test_pmic_initPrmValTest_commMode(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10579,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_COMM_MODE_VALID_SHIFT;
    pmicCfgData.commMode = 3U;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10579,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for I2C1/SPI Main Interface handle
 */
static void test_pmic_initPrmValTest_pCommHandle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10580,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_COMM_HANDLE_VALID_SHIFT;
    pmicCfgData.pCommHandle = NULL;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);

    pmic_testResultUpdate_pass(10580,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for I2C2-QA Interface handle
 */
static void test_pmic_initPrmValTest_pQACommHandle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10581,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT;
    pmicCfgData.pQACommHandle = NULL;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);

    pmic_testResultUpdate_pass(10581,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for I2C/SPI Comm LLD Read Function
 */
static void test_pmic_initPrmValTest_pFnCommIoRead(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10582,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_COMM_IO_RD_VALID_SHIFT;
    pmicCfgData.pFnPmicCommIoRead = NULL;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);

    pmic_testResultUpdate_pass(10582,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for I2C/SPI Comm LLD Write Function
 */
static void test_pmic_initPrmValTest_pFnCommIoWrite(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10583,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_COMM_IO_WR_VALID_SHIFT;
    pmicCfgData.pFnPmicCommIoWrite = NULL;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);

    pmic_testResultUpdate_pass(10583,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for Critical Section Start Function
 */
static void test_pmic_initPrmValTest_pFnCritSecStart(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10584,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_CRITSEC_START_VALID_SHIFT;
    pmicCfgData.pFnPmicCritSecStart = NULL;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);

    pmic_testResultUpdate_pass(10584,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for Critical Section Stop Function
 */
static void test_pmic_initPrmValTest_pFnCritSecStop(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10585,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;
    pmicCfgData.pFnPmicCritSecStop = NULL;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);

    pmic_testResultUpdate_pass(10585,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_deinit : Parameter validation for pPmicCoreHandle
 */
static void test_pmic_deinitPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10586,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_deinit(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10586,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setScratchPadValue : Test to write data to scratchpad reg1, reg2 and reg3
 */
static void test_pmic_setScratchPadValue_reg1Reg2Reg3(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t scratchPadRegId, data;

    test_pmic_print_unity_testcase_info(10587,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    data = 0x10;
    scratchPadRegId = PMIC_SCRATCH_PAD_REG_1;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    scratchPadRegId = PMIC_SCRATCH_PAD_REG_2;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    scratchPadRegId = PMIC_SCRATCH_PAD_REG_3;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(10587,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for I2C1 Speed
 */
static void test_pmic_initPrmValTest_i2c1Speed(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10588,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_I2C1_SPEED_VALID_SHIFT;
    pmicCfgData.i2c1Speed = 2U;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10588,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Parameter validation for I2C2 Speed
 */
static void test_pmic_initPrmValTest_i2c2Speed(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10589,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    pmicCfgData.validParams = PMIC_CFG_I2C2_SPEED_VALID_SHIFT;
    pmicCfgData.i2c2Speed = 2U;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);

    pmic_testResultUpdate_pass(10589,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

#if defined(SOC_J7200)
/*!
 * \brief   Pmic_setCommonCtrlConfig : Disable Skip EEPROM Default Load to CONF and Other registers
 */
static void test_pmic_setCommonCtrlCfg_hera_skipEepromDefaultLoadDisable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(10590,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10590,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.skipEepromDefaultLoadEn,
                      commonCtrlCfg_rd.skipEepromDefaultLoadEn);

    pmic_testResultUpdate_pass(10590,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Pmic_setI2CSpeedCfg : Parameter validation for handle
 */
static void test_pmic_setI2CSpeedCfgPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10591,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_setI2CSpeedCfg(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10591,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCrcStatus : Parameter validation for handle
 */
static void test_pmic_getCrcStatusPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t i2c1SpiCrcStat = 0xFF, i2c2CrcStat = 0xFF;

    test_pmic_print_unity_testcase_info(10592,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getCrcStatus(NULL, &i2c1SpiCrcStat, &i2c2CrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10592,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCrcStatus : Parameter validation for pI2c2CrcStatus
 */
static void test_pmic_getCrcStatusPrmValTest_pI2c2CrcStatus(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t i2c1SpiCrcStat = 0xFF;

    test_pmic_print_unity_testcase_info(10593,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getCrcStatus(pPmicCoreHandle, &i2c1SpiCrcStat, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10593,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCrcStatus : Parameter validation for pI2c1CrcStatus
 */
static void test_pmic_getCrcStatusPrmValTest_pI2c1CrcStatus(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    uint8_t i2c2SpiCrcStat = 0xFF;

    test_pmic_print_unity_testcase_info(10594,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getCrcStatus(pPmicCoreHandle, NULL, &i2c2SpiCrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10594,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCrcStatus : Parameter validation for both pI2c1CrcStatus and pI2c2CrcStatus
 */
static void test_pmic_getCrcStatusPrmValTest_pI2c1I2C2CrcStatus(void)
{
    int32_t status  = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10595,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_getCrcStatus(pPmicCoreHandle, NULL, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pmic_testResultUpdate_pass(10595,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_enableCRC : Parameter validation for handle
 */
static void test_pmic_enableCRCPrmValTest_handle(void)
{
    int32_t status  = PMIC_ST_SUCCESS;

    test_pmic_print_unity_testcase_info(10596,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    status = Pmic_enableCRC(NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, status);

    pmic_testResultUpdate_pass(10596,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setBatteryCtrlConfig : Negative test for LP8764x Hera PMIC device
 */
static void test_pmic_setBatteryCtrlCfg_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };

    test_pmic_print_unity_testcase_info(10597,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10597,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(10597,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getBatteryCtrlConfig : Negative test for LP8764x Hera PMIC device
 */
static void test_pmic_getBatteryCtrlCfg_hera(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_rd = {PMIC_CFG_CHARGING_EN_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(10598,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10598,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NOT_SUPPORTED, pmicStatus);

    pmic_testResultUpdate_pass(10598,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Disable Load from EEPROM defaults on RTC domain/conf Registers
 */
static void test_pmic_setCommonCtrlCfg_hera_EepromDefaultLoadDisable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(10599,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        commonCtrlCfg.eepromDefaultLoad = PMIC_TPS6594X_EEPROM_DEFAULTS_NOT_LOADED_TO_RTC_DOMAIN_BITS;
    }

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.eepromDefaultLoad,
                      commonCtrlCfg_rd.eepromDefaultLoad);

    pmic_testResultUpdate_pass(10599,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setCommonCtrlConfig : Negative test for TPS6594x Leo PMIC device
 */
static void test_pmic_setCommonCtrlCfg_leo_skipEepromDefaultLoadDisable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg = {PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT};

    test_pmic_print_unity_testcase_info(10600,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10600,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10600,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_getCommonCtrlConfig : Negative test for TPS6594x Leo PMIC device
 */
static void test_pmic_getCommonCtrlCfg_leo_skipEepromDefaultLoadDisable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT};

    test_pmic_print_unity_testcase_info(10601,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(10601,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(10601,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Configure NRSTOUT Signal for high signal
 */
static void test_pmic_setMiscCtrlCfg_nRstOutSignal_high(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_NRSTOUT_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_NRSTOUT_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(10602,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10602,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.nRstOutSignal,
                      miscCtrlCfg_rd.nRstOutSignal);

    pmic_testResultUpdate_pass(10602,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_setMiscCtrlConfig : Configure NRSTOUT_SOC Signal for high signal
 */
static void test_pmic_setMiscCtrlCfg_nRstOutSocSignal_high(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    test_pmic_print_unity_testcase_info(10603,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

#if defined(SOC_J721E)
    if((PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType))
    {
        /* Ignore the test on J721E PG1.0 Not planning to debug this issue as
           the same test is working fine for J721E PG2.0 */
        pmic_testResultUpdate_ignore(10603,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }
#endif

    pmicStatus = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(miscCtrlCfg.nRstOutSocSignal,
                      miscCtrlCfg_rd.nRstOutSocSignal);

    pmic_testResultUpdate_pass(10603,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_init : Config Device Type as PMIC_DEV_HERA_LP8764X
  */
static void test_pmic_init_cfg_deviceType(void)
{
    int32_t status  = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicCfgData = {0U};
    Pmic_CoreHandle_t pHandle = {0U};

    test_pmic_print_unity_testcase_info(10604,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    /** For MC/DC Coverage */
    pmicCfgData.validParams = PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;
    pmicCfgData.pmicDeviceType = PMIC_DEV_HERA_LP8764X;
    status = Pmic_init(&pmicCfgData, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);

    pmic_testResultUpdate_pass(10604,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

#if defined(SOC_J7200)
/*!
 * \brief   Pmic_setCommonCtrlConfig : Enable Skip EEPROM Default Load to CONF and Other registers
 */
static void test_pmic_setCommonCtrlCfg_hera_skipEepromDefaultLoadEnable(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_ENABLED,
        PMIC_LP8764X_EEPROM_DEFAULTS_NOT_LOADED_TO_CONF_OTHER_REGS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };

    test_pmic_print_unity_testcase_info(10751,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(10751,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(commonCtrlCfg.skipEepromDefaultLoadEn,
                      commonCtrlCfg_rd.skipEepromDefaultLoadEn);

    pmic_testResultUpdate_pass(10751,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}
#endif

/*!
 * \brief   Fault Injection and Coverage Gaps
 */
static void test_pmic_coverageGaps(void)
{
    int32_t status        = PMIC_ST_SUCCESS;
    Pmic_RecovCntCfg_t recovCntCfg;
    Pmic_RecovCntCfg_t recovCntCfg_rd;
    Pmic_CoreCfg_t pmicCoreCfg = {0U};
    Pmic_CoreHandle_t pHandle = {0U};
    Pmic_CoreHandle_t *pmicCoreHandle;
    uint8_t scratchPadRegId, data, userSpareRegNum;
    uint8_t i2c1SpiCrcStat = 0xFF, i2c2CrcStat = 0xFF;
    uint8_t i2c1Speed, i2c2Speed;
    uint32_t testDrvInitStatus;
    bool  testCrcEnable;
    Pmic_DeviceInfo_t deviceInfo;
    Pmic_CommonCtrlCfg_t commonCtrlCfg_rd = {0U};
    Pmic_CommonCtrlCfg_t commonCtrlCfg =
    {
        PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT,
        PMIC_SPREAD_SPECTRUM_CFG_ENABLE,
        PMIC_LP8764X_SKIP_EEPROM_DEF_LD_TO_CONF_OTHER_REGS_DISABLED,
        PMIC_TPS6594X_EEPROM_DEFAULTS_LOAD_TO_RTC_DOMAIN_BITS,
        PMIC_PIN_SIGNAL_LEVEL_LOW,
        PMIC_REGISTER_UNLOCK,
        PMIC_SPREAD_SPECTRUM_MODULATION_DEPTH_NONE
    };
    Pmic_CommonCtrlStat_t commonCtrlStat_rd = {0U};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg_rd = {0U};
    Pmic_BatteryCtrlCfg_t batteryCtrlCfg =
    {
        PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT,
        PMIC_TPS6594X_BB_CHARGINGING_CFG_DISABLE,
        PMIC_TPS6594X_BB_ENDOF_CHARGE_VOLATGE_2_5_V,
        PMIC_TPS6594X_BB_CHARGING_CURRENT_100
    };
    Pmic_MiscCtrlCfg_t miscCtrlCfg_rd = {0U};
    Pmic_MiscCtrlCfg_t miscCtrlCfg =
    {
        PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT,
        PMIC_TPS6594X_AMUX_OUT_PIN_CFG_DISABLE,
        PMIC_INTERNAL_CLK_MONITORING_CFG_DISABLE,
        PMIC_SYNCCLKOUT_DISABLE,
        PMIC_INTERNAL_RC_OSC,
        PMIC_TPS6594X_SYNCCLKIN_1_1_MHZ,
        PMIC_PIN_SIGNAL_LEVEL_HIGH,
        PMIC_PIN_SIGNAL_LEVEL_HIGH
    };

    uint8_t testpmicDevSiliconRev;
    uint8_t testCommMode;
    void *testCommHandle;
    void *testQACommHandle;
    void (*testFnPmicCritSecStart)(void);
    void (*testFnPmicCritSecStop)(void);
    int32_t (*testFnPmicCommIoRead)(struct Pmic_CoreHandle_s *,
                                    uint8_t,
                                    uint16_t,
                                    uint8_t *,
                                    uint8_t);
    int32_t (*testFnPmicCommIoWrite)(struct Pmic_CoreHandle_s *,
                                     uint8_t,
                                     uint16_t,
                                     uint8_t *,
                                     uint8_t);
    int32_t crcTestFlag = PMIC_STATUS_CRC_INIT_VAL;

    test_pmic_print_unity_testcase_info(8933,
                                        pmic_misc_tests,
                                        PMIC_MISC_NUM_OF_TESTCASES);

    if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
    {
        pmic_testResultUpdate_ignore(8933,
                                     pmic_misc_tests,
                                     PMIC_MISC_NUM_OF_TESTCASES);
    }

#if defined(SOC_J721E)
    crcTestFlag = gCrcTestFlag_J721E;
#endif

#if defined(SOC_J7200)
    crcTestFlag = gCrcTestFlag_J7VCL;
#endif

    //Fault Injection Tests
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    gPmic_faultInjectCfg.commError = PMIC_ST_SUCCESS;

    recovCntCfg.validParams    = PMIC_CFG_RECOV_CNT_THR_VAL_VALID_SHIFT;
    recovCntCfg.thrVal         = PMIC_RECOV_CNT_THR_MAX;
    status = Pmic_setRecoveryCntCfg(pPmicCoreHandle, recovCntCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setUserSpareValue
    data = PMIC_USER_SPARE_REG_VAL_1;
    userSpareRegNum = PMIC_USER_SPARE_REG_3;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_setUserSpareValue(pPmicCoreHandle, userSpareRegNum, data);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_spreadSpectrumEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg.validParams = PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT;
    status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getSpreadSpectrumEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg_rd.validParams = PMIC_CFG_SPREAD_SPECTRUM_EN_VALID_SHIFT;
    status = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setI2CSpeedCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_setI2CSpeedCfg(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getStartupEndrvNrstoutsocNrstoutNintPinStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_STARTUP_PIN_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getNIntPinStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_NINT_PIN_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getNRstOutPinStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_NRSTOUT_PIN_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getNRstOutSocPinStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_NRSTOUTSOC_PIN_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getEnDrvPinStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_EN_DRV_PIN_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getStartupPinStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_STARTUP_PIN_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getExtClkValidityStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_EXT_CLK_VALIDITY_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getRegLockStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_REGISTER_LOCK_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getBackupBatteryEocIndicationStat
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_BB_EOC_INDICATION_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getEnableDrvI2CSPICfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_FORCE_ENABLE_DRV_LOW_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getSpmiLpmCtrlCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlStat_rd.validParams = PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT;
    status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);


    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        //Pmic_setBackupBatteryChargingCurrentVal
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        batteryCtrlCfg.validParams = PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT;
        status = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        //Pmic_getBackupBatteryChargingCurrentVal
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        batteryCtrlCfg_rd.validParams = PMIC_CFG_CHARGE_CURRENT_VALID_SHIFT;
        status = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        //Pmic_setBackupBatteryEndOfChargeVoltage
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        batteryCtrlCfg.validParams = PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT;
        status = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        //Pmic_getBackupBatteryChargingCurrentVal
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        batteryCtrlCfg_rd.validParams = PMIC_CFG_END_OF_CHARGE_VOLTAGE_VALID_SHIFT;
        status = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        //Pmic_backupBatteryChargingEnable
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        batteryCtrlCfg.validParams = PMIC_CFG_CHARGING_EN_VALID_SHIFT;
        status = Pmic_setBatteryCtrlConfig(pPmicCoreHandle, batteryCtrlCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        //Pmic_getBackupBatteryChargingCfg
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        batteryCtrlCfg_rd.validParams = PMIC_CFG_CHARGING_EN_VALID_SHIFT;
        status = Pmic_getBatteryCtrlConfig(pPmicCoreHandle, &batteryCtrlCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    }

    //Pmic_getExtclkfreqSelNRstOutSocNRstOutCfg
    //Pmic_getExternalClkFreqSelectCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getNRstOutSignalCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_NRSTOUT_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getNRstOutSocSignalCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_selectExternalClkFreq
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_SYNC_CLK_IN_FREQ_VALID_SHIFT;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_selectExternalClk
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getExternalClkSelectCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_EXT_CLK_SEL_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_selectSyncClkOutFreq
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getSyncClkOutFreqSelectCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_SYNC_CLK_OUT_FREQ_SEL_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setInternalClkMonitorCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_CLK_MON_EN_VALID_SHIFT;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getInternalClkMonitorCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_CLK_MON_EN_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setAmuxOutRefOutPinCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getAmuxOutRefOutPinCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg_rd.validParams = PMIC_CFG_AMUX_OUT_REF_OUT_EN_VALID_SHIFT;
    status = Pmic_getMiscCtrlConfig(pPmicCoreHandle, &miscCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setSpreadSpectrumModDepthCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg.validParams = PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT;
    status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getSpreadSpectrumModDepthCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg_rd.validParams = PMIC_CFG_SPREAD_SPECTRUM_DEPTH_VALID_SHIFT;
    status = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setRegisterLockUnLockCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg.validParams = PMIC_CFG_REG_LOCK_VALID_SHIFT;
    status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_eepromDefaultLoadEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg.validParams = PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT;
    status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getEepromDefaultLoadEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg_rd.validParams = PMIC_CFG_EEPROM_DEFAULT_VALID_SHIFT;
    status = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_skipEepromDefaultLoadEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        commonCtrlCfg.validParams = PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT;
        status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    }

    //Pmic_getSkipEepromDefaultLoadEnable
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        commonCtrlCfg_rd.validParams = PMIC_CFG_SKIP_EEPROM_LOAD_VALID_SHIFT;
        status = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    }

    //Pmic_setEnableDrvPinCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg.validParams = PMIC_CFG_ENABLE_DRV_VALID_SHIFT;
    status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_getEnableDrvPinCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    commonCtrlCfg_rd.validParams = PMIC_CFG_ENABLE_DRV_VALID_SHIFT;
    status = Pmic_getCommonCtrlConfig(pPmicCoreHandle, &commonCtrlCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setNRstOutSignalCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_NRSTOUT_VALID_SHIFT;
    miscCtrlCfg.nRstOutSignal = PMIC_PIN_SIGNAL_LEVEL_LOW;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    //Pmic_setNRstOutSocSignalCfg
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    miscCtrlCfg.validParams = PMIC_CFG_NRSTOUT_SOC_VALID_SHIFT;
    miscCtrlCfg.nRstOutSocSignal = PMIC_PIN_SIGNAL_LEVEL_LOW;
    status = Pmic_setMiscCtrlConfig(pPmicCoreHandle, miscCtrlCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    pmicCoreHandle = &pHandle;
    *pmicCoreHandle = *pPmicCoreHandle;

   //Pmic_updateSubSysInfoValidateMainQaCommIFRdWr
    testDrvInitStatus = pHandle.drvInitStatus;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 3;
    pHandle.drvInitStatus = 0;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
    pHandle.drvInitStatus = testDrvInitStatus;

    //Pmic_getDeviceInfo
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_getDeviceInfo(pPmicCoreHandle, &deviceInfo);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    testCommMode = pHandle.commMode;
    testCrcEnable = pHandle.crcEnable;

    //Pmic_initCoreHandleBasicDevCfgParams
    pmicCoreCfg.validParams = PMIC_CFG_SLAVEADDR_VALID_SHIFT;
    pHandle.commMode = 3U;
    pHandle.crcEnable = 0U;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_initCoreHandleBasicDevCfgParams
    pmicCoreCfg.validParams = PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;
    pHandle.commMode = 3U;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_initCoreHandleI2CSpeedCommHandle
    pmicCoreCfg.validParams = PMIC_CFG_I2C1_SPEED_VALID_SHIFT;
    pHandle.commMode = 3U;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_initCoreHandleQADevCfgParams
    pmicCoreCfg.validParams = PMIC_CFG_QASLAVEADDR_VALID_SHIFT;
    pHandle.commMode = 3U;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pHandle.crcEnable = 1U;

    if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
       (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
    {
        //Pmic_initCoreHandleI2CSpeedCommHandle
        pmicCoreCfg.validParams = PMIC_CFG_I2C2_SPEED_VALID_SHIFT;
        pmicCoreCfg.i2c2Speed = PMIC_I2C_STANDARD_MODE;
        pHandle.commMode = PMIC_INTF_SINGLE_I2C;
        status = Pmic_init(&pmicCoreCfg, &pHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_I2C_COMM_FAIL, status);

        //Pmic_initCoreHandleQADevCfgParams
        testQACommHandle = pHandle.pQACommHandle;
        pmicCoreCfg.validParams = PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT;
        pmicCoreCfg.pQACommHandle = test_pmic_coverageGaps;
        pHandle.commMode = PMIC_INTF_SINGLE_I2C;
        status = Pmic_init(&pmicCoreCfg, &pHandle);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_I2C_COMM_FAIL, status);
        pHandle.pQACommHandle = testQACommHandle;
    }
    pHandle.commMode = testCommMode;
    pHandle.crcEnable = testCrcEnable;

    //Pmic_setI2CSpeedCfg
    testCommMode = pPmicCoreHandle->commMode;
    pPmicCoreHandle->commMode = PMIC_INTF_SPI;
    status = Pmic_setI2CSpeedCfg(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_getCrcStatus
    pPmicCoreHandle->commMode = 3U;
    status = Pmic_getCrcStatus(pPmicCoreHandle, &i2c1SpiCrcStat, &i2c2CrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_COMM_MODE, status);

    if((PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_STATUS_CRC_ENABLED == crcTestFlag))
    {
        recovCntCfg_rd.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
        status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, &recovCntCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, status);
    }

    //Pmic_getCrcStatus
    pPmicCoreHandle->commMode = PMIC_INTF_SPI;
    status = Pmic_getCrcStatus(pPmicCoreHandle, &i2c1SpiCrcStat, &i2c2CrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_getCrcStatus
    pPmicCoreHandle->commMode = 0;
    status = Pmic_getCrcStatus(pPmicCoreHandle, &i2c1SpiCrcStat, &i2c2CrcStat);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    //Pmic_getI2CSpeed
    pPmicCoreHandle->commMode = PMIC_INTF_SPI;
    status = Pmic_getI2CSpeed(pPmicCoreHandle, &i2c1Speed, &i2c2Speed);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_COMM_MODE, status);

    pPmicCoreHandle->commMode = testCommMode;

    pmicCoreCfg.validParams = 0U;

    testCommHandle = pHandle.pCommHandle;
    pHandle.pCommHandle = NULL;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);
    pHandle.pCommHandle = testCommHandle;

    testFnPmicCritSecStart = pHandle.pFnPmicCritSecStart;

    pHandle.pFnPmicCritSecStart = NULL;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);
    pHandle.pFnPmicCritSecStart = testFnPmicCritSecStart;

    testFnPmicCritSecStop = pHandle.pFnPmicCritSecStop;
    pHandle.pFnPmicCritSecStop = NULL;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);
    pHandle.pFnPmicCritSecStop = testFnPmicCritSecStop;

    testFnPmicCommIoRead = pHandle.pFnPmicCommIoRead;
    pHandle.pFnPmicCommIoRead = NULL;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);
    pHandle.pFnPmicCommIoRead = testFnPmicCommIoRead;

    testFnPmicCommIoWrite = pHandle.pFnPmicCommIoWrite;
    pHandle.pFnPmicCommIoWrite = NULL;
    status = Pmic_init(&pmicCoreCfg, &pHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, status);
    pHandle.pFnPmicCommIoWrite = testFnPmicCommIoWrite;

    data = 0x10;
    scratchPadRegId = PMIC_SCRATCH_PAD_REG_3;
    testFnPmicCritSecStop = pPmicCoreHandle->pFnPmicCritSecStop;
    testFnPmicCritSecStart = pPmicCoreHandle->pFnPmicCritSecStart;
    testFnPmicCommIoRead = pPmicCoreHandle->pFnPmicCommIoRead;
    testFnPmicCommIoWrite = pPmicCoreHandle->pFnPmicCommIoWrite;

    pPmicCoreHandle->pFnPmicCritSecStop = NULL;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    pPmicCoreHandle->pFnPmicCritSecStop = testFnPmicCritSecStop;

    pPmicCoreHandle->pFnPmicCritSecStart = NULL;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    pPmicCoreHandle->pFnPmicCritSecStart = testFnPmicCritSecStart;

    // Coverage Gaps In IO
    uint8_t data_rd = 0U;
    pPmicCoreHandle->pFnPmicCommIoRead = NULL;
    status = Pmic_getScratchPadValue(pPmicCoreHandle, scratchPadRegId, &data_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);
    pPmicCoreHandle->pFnPmicCommIoRead = testFnPmicCommIoRead;

    pPmicCoreHandle->pFnPmicCommIoWrite = NULL;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_FPTR, status);
    pPmicCoreHandle->pFnPmicCommIoWrite = testFnPmicCommIoWrite;

    testCommHandle = pPmicCoreHandle->pCommHandle;
    pPmicCoreHandle->pCommHandle = NULL;
    status = Pmic_setScratchPadValue(pPmicCoreHandle, scratchPadRegId, data);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    status = Pmic_getScratchPadValue(pPmicCoreHandle, scratchPadRegId, &data_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, status);

    pPmicCoreHandle->pCommHandle = testCommHandle;

    //Pmic_getDeviceInfo
    testpmicDevSiliconRev = pPmicCoreHandle->pmicDevSiliconRev;
    pPmicCoreHandle->pmicDevSiliconRev = 0x01U;
    status = Pmic_getDeviceInfo(pPmicCoreHandle, &deviceInfo);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_SILICON_REVISION, status);
    pPmicCoreHandle->pmicDevSiliconRev = testpmicDevSiliconRev;

    if(PMIC_STATUS_CRC_INIT_VAL == crcTestFlag)
    {
        //Pmic_getRecoveryCntCfg
        gRecoveryCntCfgTestFlag = 1U;
        recovCntCfg_rd.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
        status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, &recovCntCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        gRecoveryCntCfgTestFlag = 0U;

        //Pmic_setEnableDrvPinCfg
        gEnableDrvPinCfgTestFlag = 1U;
        commonCtrlCfg.validParams = PMIC_CFG_ENABLE_DRV_VALID_SHIFT;
        status = Pmic_setCommonCtrlConfig(pPmicCoreHandle, commonCtrlCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_EN_DRV_PIN_CFG, status);
        gEnableDrvPinCfgTestFlag = 0U;

        //Pmic_getSpmiLpmCtrlCfg
        gSpmiLpmStatTestFlag = 1U;
        commonCtrlStat_rd.validParams = PMIC_CFG_SPMI_LPM_STAT_VALID_SHIFT;
        status = Pmic_getCommonCtrlStat(pPmicCoreHandle, &commonCtrlStat_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
        gSpmiLpmStatTestFlag = 0U;

        if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
           (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
        {
            //Pmic_updateCrcEnableStatValidateDevOnBus
            testDrvInitStatus = pHandle.drvInitStatus;
            gI2c2CrcEnableTestFlag = 1U;
            pHandle.drvInitStatus = 0x0U;
            status = Pmic_init(&pmicCoreCfg, &pHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_CRC_STATUS_FAIL, status);
            pHandle.drvInitStatus = testDrvInitStatus;
            gI2c2CrcEnableTestFlag = 0U;
        }

        if((J721E_LEO_PMICB_DEVICE == pmic_device_info) ||
           (J7VCL_HERA_PMICB_DEVICE == pmic_device_info))
        {
            testDrvInitStatus = pHandle.drvInitStatus;
            gI2c1CrcEnableTestFlag = 1U;
            pHandle.drvInitStatus = 0x0U;
            status = Pmic_init(&pmicCoreCfg, &pHandle);
            TEST_ASSERT_EQUAL(PMIC_ST_ERR_DATA_IO_CRC, status);
            pHandle.drvInitStatus = testDrvInitStatus;
            gI2c1CrcEnableTestFlag = 0U;
        }
    }

    if((PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
       (PMIC_STATUS_CRC_ENABLED == crcTestFlag))
    {
        //Pmic_getRecoveryCntCfg
        gRecoveryCntCfgTestFlag = 1U;
        recovCntCfg_rd.validParams = PMIC_CFG_RECOV_CNT_CLR_CNT_VALID_SHIFT;
        status = Pmic_getRecoveryCntCfg(pPmicCoreHandle, &recovCntCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_ERR_DATA_IO_CRC, status);
        gRecoveryCntCfgTestFlag = 0U;

    }

    if((J721E_LEO_PMICA_DEVICE == pmic_device_info) ||
       (J7VCL_LEO_PMICA_DEVICE == pmic_device_info))
    {
        gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;

        test_pmic_appDeInit(pPmicCoreHandle);
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        pmicCoreCfg.validParams = 0U;
        status = test_pmic_leo_pmicA_misc_testApp();
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        test_pmic_appDeInit(pPmicCoreHandle);
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 2;
        pmicCoreCfg.validParams = 0U;
        status = test_pmic_leo_pmicA_misc_testApp();
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

        gPmic_faultInjectCfg.commError = PMIC_ST_SUCCESS;
        test_pmic_appDeInit(pPmicCoreHandle);
        status = test_pmic_leo_pmicA_misc_testApp();

        gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;
    }

    pmic_testResultUpdate_pass(8933,
                               pmic_misc_tests,
                               PMIC_MISC_NUM_OF_TESTCASES);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))
/*!
 * \brief   Run misc unity test cases for Master PMIC
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_misc_tests, PMIC_MISC_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_setRecoveryCntCfg_threshold);
    RUN_TEST(test_pmic_setRecoveryCntCfgPrmValTest_thrVal);
    RUN_TEST(test_pmic_setRecoveryCntCfg_clrCnt);
    RUN_TEST(test_pmic_setRecoveryCntCfgPrmValTest_clrCnt);
    RUN_TEST(test_pmic_setRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_recovCntCfg);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_handle);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_recovCntVal);
    RUN_TEST(test_pmic_setScratchPadValuePrmValTest_handle);
    RUN_TEST(test_pmic_setScratchPadValuePrmValTest_scratchPadRegId);
    RUN_TEST(test_pmic_setScratchPadValue_setget);
    RUN_TEST(test_pmic_getScratchPadValuePrmValTest_handle);
    RUN_TEST(test_pmic_getScratchPadValuePrmValTest_scratchPadRegId);
    RUN_TEST(test_pmic_getScratchPadValuePrmValTest_data);
    RUN_TEST(test_pmic_setUserSpareValuePrmValTest_handle);
    RUN_TEST(test_pmic_setUserSpareValuePrmValTest_userSpareRegNum);
    RUN_TEST(test_pmic_setUserSpareValuePrmValTest_data);
    RUN_TEST(test_pmic_setUserSpareValue);
    RUN_TEST(test_pmic_getUserSpareValuePrmValTest_handle);
    RUN_TEST(test_pmic_getUserSpareValuePrmValTest_userSpareRegNum);
    RUN_TEST(test_pmic_getUserSpareValuePrmValTest_data);
    RUN_TEST(test_pmic_setCommonCtrlCfg_sreadSpectrumEnable);
    RUN_TEST(test_pmic_setCommonCtrlCfg_RegisterLock);
    RUN_TEST(test_pmic_setCommonCtrlCfg_sreadSpectrumDepth);
    RUN_TEST(test_pmic_getCommonCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getCommonCtrlCfgPrmValTest_pCommonCtrlCfg);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_eepromDefaultLoad);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_regLock);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_spreadSpectrumDepth);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_setMiscCtrlCfg_amuxOutRefOutEn);
    RUN_TEST(test_pmic_setMiscCtrlCfg_clkMonEn);
    RUN_TEST(test_pmic_setMiscCtrlCfg_syncClkOutFreqSel);
    RUN_TEST(test_pmic_setMiscCtrlCfg_extClkSel);
    RUN_TEST(test_pmic_setMiscCtrlCfg_syncClkInFreq);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_syncClkOutFreqSel);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_extClkSel);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_syncClkInFreq);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getMiscCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getMiscCtrlCfgPrmValTest_pMiscCtrlCfg);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_chargingEn);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_endOfChargeVoltage);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_chargeCurrent);
    RUN_TEST(test_pmic_setBatteryCtrlCfgPrmValTest_endOfChargeVoltage);
    RUN_TEST(test_pmic_setBatteryCtrlCfgPrmValTest_chargeCurrent);
    RUN_TEST(test_pmic_setBatteryCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getBatteryCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getBatteryCtrlCfgPrmValTest_pBatteryCtrlCfg);
    RUN_TEST(test_pmic_getCommonCtrlStat_nIntPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_spmiLpmStat);
    RUN_TEST(test_pmic_getCommonCtrlStat_forceEnDrvLowStat);
    RUN_TEST(test_pmic_getCommonCtrlStat_bbEndOfChargeIndication);
    RUN_TEST(test_pmic_getCommonCtrlStat_regLockStat);
    RUN_TEST(test_pmic_getCommonCtrlStat_extClkValidity);
    RUN_TEST(test_pmic_getCommonCtrlStat_startupPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_enDrvPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_nRstOutSocPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_nRstOutPin);
    RUN_TEST(test_pmic_getCommonCtrlStatPrmValTest_handle);
    RUN_TEST(test_pmic_getCommonCtrlStatPrmValTest_pCommonCtrlStat);
    RUN_TEST(test_pmic_getI2CSpeedPrmValTest_handle);
    RUN_TEST(test_pmic_getI2CSpeedPrmValTest_pI2C1Speed);
    RUN_TEST(test_pmic_getI2CSpeedPrmValTest_pI2C2Speed);
    RUN_TEST(test_pmic_getI2CSpeed);
    RUN_TEST(test_pmic_getDeviceInfo);
    RUN_TEST(test_pmic_getDeviceInfoPrmValTest_handle);
    RUN_TEST(test_pmic_getDeviceInfoPrmValTest_pDeviceInfo);
    RUN_TEST(test_pmic_getPinValue_enableDrv);
    RUN_TEST(test_pmic_getPinValue_nRstOutSoc);
    RUN_TEST(test_pmic_getPinValue_nRstOut);
    RUN_TEST(test_pmic_getPinValuePrmValTest_pinType);
    RUN_TEST(test_pmic_getPinValuePrmValTest_handle);
    RUN_TEST(test_pmic_getPinValuePrmValTest_pPinValue);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_nRstOutSocSignal);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_nRstOutSignal);
    RUN_TEST(test_pmic_getMiscCtrlCfg_nRstOutSocSignal);
    RUN_TEST(test_pmic_getMiscCtrlCfg_nRstOutSignal);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_enDrv);
    RUN_TEST(test_pmic_WriteProtection_RegisterLock);
    RUN_TEST(test_pmic_WriteProtection_RegisterUnLock);

    RUN_TEST(test_pmic_getRecoveryCntVal);
    RUN_TEST(test_pmic_getRecoveryCntPrmValTest_pRecovCntVal);
    RUN_TEST(test_pmic_initPrmValTest_handle);
    RUN_TEST(test_pmic_initPrmValTest_pmicCfgData);
    RUN_TEST(test_pmic_initPrmValTest_pmicDevType);
    RUN_TEST(test_pmic_initPrmValTest_commMode);
    RUN_TEST(test_pmic_initPrmValTest_pCommHandle);
    RUN_TEST(test_pmic_initPrmValTest_pQACommHandle);
    RUN_TEST(test_pmic_initPrmValTest_pFnCommIoRead);
    RUN_TEST(test_pmic_initPrmValTest_pFnCommIoWrite);
    RUN_TEST(test_pmic_initPrmValTest_pFnCritSecStart);
    RUN_TEST(test_pmic_initPrmValTest_pFnCritSecStop);
    RUN_TEST(test_pmic_deinitPrmValTest_handle);
    RUN_TEST(test_pmic_setScratchPadValue_reg1Reg2Reg3);

    RUN_TEST(test_pmic_initPrmValTest_i2c1Speed);
    RUN_TEST(test_pmic_initPrmValTest_i2c2Speed);
#if defined(SOC_J7200)
    RUN_TEST(test_pmic_setCommonCtrlCfg_hera_skipEepromDefaultLoadDisable);
    RUN_TEST(test_pmic_setCommonCtrlCfg_hera_skipEepromDefaultLoadEnable);
#endif
    RUN_TEST(test_pmic_setI2CSpeedCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_handle);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_pI2c2CrcStatus);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_pI2c1CrcStatus);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_pI2c1I2C2CrcStatus);
    RUN_TEST(test_pmic_enableCRCPrmValTest_handle);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_hera);
    RUN_TEST(test_pmic_getBatteryCtrlCfg_hera);
    RUN_TEST(test_pmic_setCommonCtrlCfg_hera_EepromDefaultLoadDisable);
    RUN_TEST(test_pmic_setCommonCtrlCfg_leo_skipEepromDefaultLoadDisable);
    RUN_TEST(test_pmic_getCommonCtrlCfg_leo_skipEepromDefaultLoadDisable);
    RUN_TEST(test_pmic_setMiscCtrlCfg_nRstOutSignal_high);
    RUN_TEST(test_pmic_setMiscCtrlCfg_nRstOutSocSignal_high);
    RUN_TEST(test_pmic_init_cfg_deviceType);
    RUN_TEST(test_pmic_WriteProtection_RegisterLock_codecoverage);
    RUN_TEST(test_pmic_setCommonCtrlCfg_enDrv_codecoverage);

    RUN_TEST(test_pmic_coverageGaps);

    pmic_updateTestResults(pmic_misc_tests, PMIC_MISC_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   Run misc unity test cases for Slave PMIC
 */
static void test_pmic_run_slave_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_misc_tests, PMIC_MISC_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_setRecoveryCntCfg_threshold);
    RUN_TEST(test_pmic_setRecoveryCntCfgPrmValTest_thrVal);
    RUN_TEST(test_pmic_setRecoveryCntCfg_clrCnt);
    RUN_TEST(test_pmic_setRecoveryCntCfgPrmValTest_clrCnt);
    RUN_TEST(test_pmic_setRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getRecoveryCntCfgPrmValTest_recovCntCfg);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_handle);
    RUN_TEST(test_Pmic_getRecoveryCntPrmValTest_recovCntVal);
    RUN_TEST(test_pmic_setScratchPadValuePrmValTest_handle);
    RUN_TEST(test_pmic_setScratchPadValuePrmValTest_scratchPadRegId);
    RUN_TEST(test_pmic_setScratchPadValue_setget);
    RUN_TEST(test_pmic_getScratchPadValuePrmValTest_handle);
    RUN_TEST(test_pmic_getScratchPadValuePrmValTest_scratchPadRegId);
    RUN_TEST(test_pmic_getScratchPadValuePrmValTest_data);

    RUN_TEST(test_pmic_setUserSpareValuePrmValTest_handle);
    RUN_TEST(test_pmic_setUserSpareValuePrmValTest_userSpareRegNum);
    RUN_TEST(test_pmic_setUserSpareValuePrmValTest_data);
    RUN_TEST(test_pmic_setUserSpareValue);
    RUN_TEST(test_pmic_getUserSpareValuePrmValTest_handle);
    RUN_TEST(test_pmic_getUserSpareValuePrmValTest_userSpareRegNum);
    RUN_TEST(test_pmic_getUserSpareValuePrmValTest_data);
    RUN_TEST(test_pmic_setCommonCtrlCfg_sreadSpectrumEnable);
    RUN_TEST(test_pmic_setCommonCtrlCfg_RegisterLock);
    RUN_TEST(test_pmic_setCommonCtrlCfg_sreadSpectrumDepth);
    RUN_TEST(test_pmic_getCommonCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getCommonCtrlCfgPrmValTest_pCommonCtrlCfg);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_eepromDefaultLoad);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_regLock);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_spreadSpectrumDepth);
    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_setMiscCtrlCfg_amuxOutRefOutEn);
    RUN_TEST(test_pmic_setMiscCtrlCfg_clkMonEn);
    RUN_TEST(test_pmic_setMiscCtrlCfg_syncClkOutFreqSel);
    RUN_TEST(test_pmic_setMiscCtrlCfg_extClkSel);
    RUN_TEST(test_pmic_setMiscCtrlCfg_syncClkInFreq);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_syncClkOutFreqSel);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_extClkSel);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_syncClkInFreq);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getMiscCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getMiscCtrlCfgPrmValTest_pMiscCtrlCfg);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_chargingEn);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_endOfChargeVoltage);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_chargeCurrent);
    RUN_TEST(test_pmic_setBatteryCtrlCfgPrmValTest_endOfChargeVoltage);
    RUN_TEST(test_pmic_setBatteryCtrlCfgPrmValTest_chargeCurrent);
    RUN_TEST(test_pmic_setBatteryCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getBatteryCtrlCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getBatteryCtrlCfgPrmValTest_pBatteryCtrlCfg);
    RUN_TEST(test_pmic_getCommonCtrlStat_nIntPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_spmiLpmStat);

    RUN_TEST(test_pmic_getCommonCtrlStat_bbEndOfChargeIndication);
    RUN_TEST(test_pmic_getCommonCtrlStat_regLockStat);
    RUN_TEST(test_pmic_getCommonCtrlStat_extClkValidity);
    RUN_TEST(test_pmic_getCommonCtrlStat_startupPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_enDrvPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_nRstOutSocPin);
    RUN_TEST(test_pmic_getCommonCtrlStat_nRstOutPin);
    RUN_TEST(test_pmic_getCommonCtrlStatPrmValTest_handle);
    RUN_TEST(test_pmic_getCommonCtrlStatPrmValTest_pCommonCtrlStat);
    RUN_TEST(test_pmic_getI2CSpeedPrmValTest_handle);
    RUN_TEST(test_pmic_getI2CSpeedPrmValTest_pI2C1Speed);
    RUN_TEST(test_pmic_getI2CSpeedPrmValTest_pI2C2Speed);
    RUN_TEST(test_pmic_getI2CSpeed);
    RUN_TEST(test_pmic_getDeviceInfo);
    RUN_TEST(test_pmic_getDeviceInfoPrmValTest_handle);
    RUN_TEST(test_pmic_getDeviceInfoPrmValTest_pDeviceInfo);
    RUN_TEST(test_pmic_getPinValue_enableDrv);


    RUN_TEST(test_pmic_getPinValuePrmValTest_pinType);
    RUN_TEST(test_pmic_getPinValuePrmValTest_handle);
    RUN_TEST(test_pmic_getPinValuePrmValTest_pPinValue);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_nRstOutSocSignal);
    RUN_TEST(test_pmic_setMiscCtrlCfgPrmValTest_nRstOutSignal);


    RUN_TEST(test_pmic_setCommonCtrlCfgPrmValTest_enDrv);
    RUN_TEST(test_pmic_WriteProtection_RegisterLock);
    RUN_TEST(test_pmic_WriteProtection_RegisterUnLock);

    RUN_TEST(test_pmic_getRecoveryCntVal);
    RUN_TEST(test_pmic_getRecoveryCntPrmValTest_pRecovCntVal);
    RUN_TEST(test_pmic_initPrmValTest_handle);
    RUN_TEST(test_pmic_initPrmValTest_pmicCfgData);
    RUN_TEST(test_pmic_initPrmValTest_pmicDevType);
    RUN_TEST(test_pmic_initPrmValTest_commMode);
    RUN_TEST(test_pmic_initPrmValTest_pCommHandle);
    RUN_TEST(test_pmic_initPrmValTest_pQACommHandle);
    RUN_TEST(test_pmic_initPrmValTest_pFnCommIoRead);
    RUN_TEST(test_pmic_initPrmValTest_pFnCommIoWrite);
    RUN_TEST(test_pmic_initPrmValTest_pFnCritSecStart);
    RUN_TEST(test_pmic_initPrmValTest_pFnCritSecStop);
    RUN_TEST(test_pmic_deinitPrmValTest_handle);
    RUN_TEST(test_pmic_setScratchPadValue_reg1Reg2Reg3);

    RUN_TEST(test_pmic_initPrmValTest_i2c1Speed);
    RUN_TEST(test_pmic_initPrmValTest_i2c2Speed);
#if defined(SOC_J7200)
    RUN_TEST(test_pmic_setCommonCtrlCfg_hera_skipEepromDefaultLoadDisable);
    RUN_TEST(test_pmic_setCommonCtrlCfg_hera_skipEepromDefaultLoadEnable);
#endif
    RUN_TEST(test_pmic_setI2CSpeedCfgPrmValTest_handle);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_handle);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_pI2c2CrcStatus);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_pI2c1CrcStatus);
    RUN_TEST(test_pmic_getCrcStatusPrmValTest_pI2c1I2C2CrcStatus);
    RUN_TEST(test_pmic_enableCRCPrmValTest_handle);
    RUN_TEST(test_pmic_setBatteryCtrlCfg_hera);
    RUN_TEST(test_pmic_getBatteryCtrlCfg_hera);
    RUN_TEST(test_pmic_setCommonCtrlCfg_hera_EepromDefaultLoadDisable);
    RUN_TEST(test_pmic_setCommonCtrlCfg_leo_skipEepromDefaultLoadDisable);
    RUN_TEST(test_pmic_getCommonCtrlCfg_leo_skipEepromDefaultLoadDisable);
    RUN_TEST(test_pmic_setMiscCtrlCfg_nRstOutSignal_high);
    RUN_TEST(test_pmic_setMiscCtrlCfg_nRstOutSocSignal_high);
    RUN_TEST(test_pmic_init_cfg_deviceType);
    RUN_TEST(test_pmic_coverageGaps);

    pmic_updateTestResults(pmic_misc_tests, PMIC_MISC_NUM_OF_TESTCASES);
    UNITY_END();
}

/*!
 * \brief   MISC Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_DUAL_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.i2c2Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C2_SPEED_VALID_SHIFT;

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J721E_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;
    }
    if(J7VCL_LEO_PMICA_DEVICE == pmic_device_info)
    {
        pmicConfigData.slaveAddr           = J7VCL_LEO_PMICA_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

        pmicConfigData.qaSlaveAddr         = J7VCL_LEO_PMICA_WDG_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

        pmicConfigData.nvmSlaveAddr        = J7VCL_LEO_PMICA_PAGE1_SLAVE_ADDR;
        pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;
    }

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   MISC Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICB_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief  MISC  Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode            = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr           = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead    = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite   = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart  = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop   = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   MISC Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_misc_testApp(void)
{
    int32_t status                = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType      = PMIC_DEV_LEO_TPS6594X;
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

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

static int32_t setup_pmic_interrupt(uint32_t board, bool enableCRC)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J721E)
            {
                gCrcTestFlag_J721E = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J721E = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_misc_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_misc_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        if(enableCRC == true)
        {
            if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
            {
                gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
            }
        }
        else
        {
            gCrcTestFlag_J7VCL = PMIC_STATUS_CRC_INIT_VAL;
        }

        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_misc_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_misc_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else
    {
        status = PMIC_ST_ERR_INV_DEVICE;
    }
    return status;
}

static const char pmicTestMenu[] =
{
    " \r\n ================================================================="
    " \r\n Test Menu:"
    " \r\n ================================================================="
    " \r\n 0: Automatic run for all board specific misc options"
    " \r\n 1: Manual run for Miscellaneous Test options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM)"
    " \r\n 2: Pmic Leo device(PMIC A on J7VCL EVM)"
    " \r\n 3: Pmic Hera device(PMIC B on J7VCL EVM)"
    " \r\n 4: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 5: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 6: Pmic Leo device(PMIC A on J721E EVM Manual Testcase)"
    " \r\n 7: Pmic Leo device(PMIC A on J7VCL EVM Manual Testcase)"
    " \r\n 8: Pmic Leo device(PMIC A and B on J721E EVM) - Runtime BIST test"
    " \r\n 9: Pmic Leo and Hera device(PMIC A and B on J7VCL EVM) - Runtime BIST test"
    " \r\n 10: Pmic Leo/Hera device on J7VCL EVM - Disabled skip EEPROM defaults load on conf registers and Disabled EEPROM defaults load on conf registers "
    " \r\n 11: Pmic Leo/Hera device on J7VCL EVM - Disabled skip EEPROM defaults load on conf registers and Enabled EEPROM defaults load on conf registers "
    " \r\n 12: Pmic Leo/Hera device on J7VCL EVM - Enabled skip EEPROM defaults load on conf registers and Enabled EEPROM defaults load on conf registers "
    " \r\n 13: Pmic Leo device(PMIC A on J721E EVM Manual Testcase with CRC Disable on PG2.0 Silicon revision)"
    " \r\n 14: Back to Test Menu"
    " \r\n"
    " \r\n Enter option: "
};



static void print_pmicTestAppManualTestMenu(uint32_t deviceType)
{
    char board_name[10] = {0};
    char device_name[10] = {0};
    char deviceSubType[10] = {0};

    if(J721E_LEO_PMICA_DEVICE == deviceType)
    {
        strcpy(board_name, "J721E");
        strcpy(device_name, "Leo");
        strcpy(deviceSubType, "A");
    }
    else if(J7VCL_LEO_PMICA_DEVICE == deviceType)
    {
        strcpy(board_name, "J7VCL");
        strcpy(device_name, "Leo");
        strcpy(deviceSubType, "A");
    }
    else if(J7VCL_HERA_PMICB_DEVICE == deviceType)
    {
        strcpy(board_name, "J7VCL");
        strcpy(device_name, "Hera");
        strcpy(deviceSubType, "B");
    }

    pmic_log(" \r\n =================================================================");
    pmic_log(" \r\n Manual Testcase Menu:");
    pmic_log(" \r\n =================================================================");

    if((pmic_device_info == J721E_LEO_PMICA_DEVICE) ||
       (pmic_device_info == J7VCL_LEO_PMICA_DEVICE))
    {
        pmic_log(" \r\n 0: Pmic %s device(PMIC %s on %s EVM for Check recovery count)", device_name, deviceSubType, board_name);
        pmic_log(" \r\n 1: Pmic %s device(PMIC %s on %s EVM for Check Enable Interrupt)", device_name, deviceSubType, board_name);
        pmic_log(" \r\n 2: Pmic %s device(PMIC %s on %s EVM for Configure NRSTOUT_SOC Signal)", device_name, deviceSubType, board_name);
        pmic_log(" \r\n 3: Pmic %s device(PMIC %s on %s EVM for Configure NRSTOUT Signal)", device_name, deviceSubType, board_name);
        pmic_log(" \r\n 4: Pmic %s device(PMIC %s on %s EVM for Configure EN_DRV Signal)", device_name, deviceSubType, board_name);
        pmic_log(" \r\n 5: Pmic %s device(PMIC %s on %s EVM for Check Load from EEPROM defaults on RTC domain/conf Registers)", device_name, deviceSubType, board_name);
        pmic_log(" \r\n 6: Pmic %s device(PMIC %s on %s EVM for Disable Load from EEPROM defaults on RTC domain/conf Registers)", device_name, deviceSubType, board_name);
    }
    pmic_log(" \r\n 7: Back to Main Menu");
    pmic_log(" \r\n");
    pmic_log(" \r\n Enter option: ");
}

/*!
 * \brief   Run MISC manual test cases
 */
static void test_pmic_run_testcases_manual(uint32_t deviceType)
{
    int8_t menuOption = -1;

    while(1U)
    {
        print_pmicTestAppManualTestMenu(deviceType);
        if(UART_scanFmt("%d", &menuOption) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(menuOption)
        {
            case 0U:
                RUN_TEST(test_pmic_getRecoveryCnt_read_recovCntVal);
               break;
            case 1U:
                 /* Display Enable interrupt */
                startup_type = PMIC_ENABLE_STARTUP_TYPE;
                RUN_TEST(test_pmic_interruptEnableInt);
               break;
            case 2U:
                RUN_TEST(test_pmic_setMiscCtrlCfg_nRstOutSocSignal);
               break;
            case 3U :
                RUN_TEST(test_pmic_setMiscCtrlCfg_nRstOutSignal);
                break;
            case 4U:
                RUN_TEST(test_pmic_setCommonCtrlCfg_enDrv);
               break;
            case 5U:
#if defined(SOC_J721E)
               if(PMIC_SILICON_REV_ID_PG_2_0 ==
                  pPmicCoreHandle->pmicDevSiliconRev)
               {
                    test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
               }
#endif
                RUN_TEST(test_pmic_setCommonCtrlCfg_eepromDefaultLoadEnable);
               break;
            case 6U:
#if defined(SOC_J721E)
               if(PMIC_SILICON_REV_ID_PG_2_0 ==
                  pPmicCoreHandle->pmicDevSiliconRev)
               {
                    test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
               }
#endif
                RUN_TEST(test_pmic_setCommonCtrlCfg_eepromDefaultLoadDisable);
               break;
            case 7U:
                pmic_log(" \r\n Back to Test Menu options\n");
                return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

static int32_t test_pmic_printPfsmDelayValue(void)
{
    uint8_t data;
    int32_t pmicStatus;

    pmicStatus = Pmic_fsmGetPfsmDelay(pPmicCoreHandle,
                                         PMIC_PFSM_DELAY3,
                                         &data);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmic_log("\r\n PFSM Delay 3 Val 0x%x", data);
    }

    return pmicStatus;
}

static void test_pmic_misc_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 1, 4};
    int8_t  userInput = 0;
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {2, 3, 5};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_misc_tests, PMIC_MISC_NUM_OF_TESTCASES);
        }
        pmic_log("%s", pmicTestAppMenu);
        if(option == PMIC_UT_AUTOMATE_OPTION)
        {
            if(idx < (sizeof(automatic_options)/sizeof(automatic_options[0])))
            {
                num = automatic_options[idx++];
            }
            else
            {
                num = 14;
            }
            pmic_log("%d\n", num);
        }
        else
        {
            if(UART_scanFmt("%d", &num) != 0U)
            {
                pmic_log("Read from UART Console failed\n");
                return;
            }
        }
        switch(num)
        {
            case 0U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* MISC Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                       test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 1U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    pmic_device_info = J721E_LEO_PMICB_DEVICE;

                    /* MISC Unity Test App wrapper Function for LEO PMIC-B */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_misc_testApp())
                    {
                        /* Run MISC test cases for Leo PMIC-B */
                        test_pmic_run_slave_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                       test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 2U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* MISC Unity Test App wrapper Function for LEO PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        /* Run MISC test cases for LEO PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 3U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                    /* MISC Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_misc_testApp())
                    {
                        /* Run MISC test cases for HERA PMIC-B */
                        test_pmic_run_slave_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 4U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                     pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* MISC Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                          test_pmic_leo_pmicA_spiStub_misc_testApp())
                    {
                        /* Run misc test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 5U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* MISC Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_misc_testApp())
                    {
                        /* Run misc test cases for Leo PMIC-A */
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
            case 6U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_ST_SUCCESS == test_pmic_printPfsmDelayValue())
                        {
                            /* Run misc manual test cases */
                            test_pmic_run_testcases_manual(J721E_LEO_PMICA_DEVICE);
                        }
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               return;
            case 7U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_ST_SUCCESS == test_pmic_printPfsmDelayValue())
                        {
                            /* Run misc manual test cases */
                            test_pmic_run_testcases_manual(J7VCL_LEO_PMICA_DEVICE);
                        }
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               return;
            case 8U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, true))
                {
                    int32_t pmicStatus = PMIC_ST_SUCCESS;

                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    pmic_log("\n\n Run Time BIST test for Leo PMIC A and B on J721E\n");

                    pmic_log("\n\n Leo PMIC-A Initialization \n\n\n");

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
                        {
                            pmic_log("\n\n Test is not supported \n");
                        }
                        else
                        {
                            test_Pmic_getBistPassInterrupt();
                        }
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
                    {
                        pmic_log("\n\n Leo PMIC-B Initialization \n\n");
                        pmic_log("\n\n Check BIST Pass Interrupt is set on Interrupt status and then it will be cleared \n");

                        pmicStatus = test_pmic_leo_pmicB_misc_testApp();

                        if(PMIC_ST_SUCCESS != pmicStatus)
                        {
                            pmic_log("\n\n Hera PMIC-B Initialization failed \n\n");
                        }

                        /* Deinit pmic handle */
                        if(pPmicCoreHandle != NULL)
                        {
                            test_pmic_appDeInit(pPmicCoreHandle);
                        }
                    }

                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 9U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    int32_t pmicStatus = PMIC_ST_SUCCESS;

                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    pmic_log("\n\n Run Time BIST test for Leo PMIC A and B on J7VCL\n");

                    pmic_log("\n\n Leo PMIC-A Initialization \n\n\n");

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_SILICON_REV_ID_PG_1_0 == pPmicCoreHandle->pmicDevSiliconRev)
                        {
                            pmic_log("\n\n Test is not supported \n");
                        }
                        else
                        {
                            test_Pmic_getBistPassInterrupt();
                        }
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
                    {
                        pmic_log("\n\n Hera PMIC-B Initialization \n\n");
                        pmic_log("\n\n Check BIST Pass Interrupt is set on Interrupt status and then it will be cleared \n");

                        pmicStatus = test_pmic_hera_misc_testApp();

                        if(PMIC_ST_SUCCESS != pmicStatus)
                        {
                            pmic_log("\n\n Hera PMIC-B Initialization failed \n\n");
                        }
                        /* Deinit pmic handle */
                        if(pPmicCoreHandle != NULL)
                        {
                            test_pmic_appDeInit(pPmicCoreHandle);
                        }
                    }

                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 10U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    int32_t pmicStatus = PMIC_ST_SUCCESS;

                    pmic_log("\nHera PMIC Initialization!!!\n");

                    test_pmic_print_unity_testcase_info(9993,
                                                        pmic_misc_tests,
                                                        PMIC_MISC_NUM_OF_TESTCASES);

                     pmicStatus = test_pmic_hera_misc_testApp();

                    if(PMIC_ST_SUCCESS == pmicStatus)
                    {
                        pmic_log("\nHera PMIC skip_eeprom config - skipEepromDefLdDisable, eePromDefLdDisable!!!\n");
                        test_pmic_setCommonCtrlCfg_hera_skipEepromDefLdDisable_eePromDefLdDisable();
                    }

                    pmic_log("\nHera PMIC DeInitialization !!!\n");

                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    pmic_log("\nLeo PMIC Initialization!!!\n");

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_ST_SUCCESS == test_pmic_printPfsmDelayValue())
                        {
                            g_skip_eeprom_test_flag = 1;
                            /* Run misc manual test cases */
                            test_pmic_setCommonCtrlCfg_eepromDefaultLoadDisable();
                            g_skip_eeprom_test_flag = 0;
                        }
                    }

                    pmic_log("\nLeo PMIC DeInitialization!!!\n");
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }


                    pmic_testResultUpdate_pass(9993,
                                               pmic_misc_tests,
                                               PMIC_MISC_NUM_OF_TESTCASES);

                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 11U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    int32_t pmicStatus = PMIC_ST_SUCCESS;

                     pmic_log("\nHera PMIC Initialization!!!\n");

                    test_pmic_print_unity_testcase_info(9991,
                                                        pmic_misc_tests,
                                                        PMIC_MISC_NUM_OF_TESTCASES);

                     pmicStatus = test_pmic_hera_misc_testApp();

                    if(PMIC_ST_SUCCESS == pmicStatus)
                    {
                        pmic_log("\nHera PMIC skip_eeprom config - skipEepromDefLdDisable, eePromDefLdEnable!!!\n");
                        test_pmic_setCommonCtrlCfg_hera_skipEepromDefLdDisable_eePromDefLdEnable();
                    }

                    pmic_log("\nHera PMIC DeInitialization !!!\n");

                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    pmic_log("\nLeo PMIC Initialization!!!\n");

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_ST_SUCCESS == test_pmic_printPfsmDelayValue())
                        {
                            g_skip_eeprom_test_flag = 1;
                            /* Run misc manual test cases */
                            test_pmic_setCommonCtrlCfg_eepromDefaultLoadEnable();
                            g_skip_eeprom_test_flag = 0;
                        }
                    }

                    pmic_log("\nLeo PMIC DeInitialization!!!\n");
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                    pmic_testResultUpdate_pass(9991,
                                               pmic_misc_tests,
                                               PMIC_MISC_NUM_OF_TESTCASES);

                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
               break;
            case 12U:
#if 0//defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD, true))
                {
                    int32_t pmicStatus = PMIC_ST_SUCCESS;

                     pmic_log("\nHera PMIC Initialization!!!\n");

                    test_pmic_print_unity_testcase_info(9990,
                                                        pmic_misc_tests,
                                                        PMIC_MISC_NUM_OF_TESTCASES);

                     pmicStatus = test_pmic_hera_misc_testApp();

                    if(PMIC_ST_SUCCESS == pmicStatus)
                    {
                        pmic_log("\nHera PMIC skip_eeprom config - skipEepromDefLdEnable, eePromDefLdEnable!!!\n");
                        test_pmic_setCommonCtrlCfg_hera_skipEepromDefLdEnable_eePromDefLdEnable();
                    }

                    pmic_log("\nHera PMIC DeInitialization !!!\n");

                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                   /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    pmic_log("\nLeo PMIC Initialization!!!\n");

                    /* MISC Unity Test App wrapper Function */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        if(PMIC_ST_SUCCESS == test_pmic_printPfsmDelayValue())
                        {
                            g_skip_eeprom_test_flag = 2;
                            /* Run misc manual test cases */
                            test_pmic_setCommonCtrlCfg_eepromDefaultLoadEnable();
                            g_skip_eeprom_test_flag = 0;
                        }
                    }

                    pmic_log("\nLeo PMIC DeInitialization!!!\n");
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                        test_pmic_appDeInit(pPmicCoreHandle);
                    }

                    pmic_testResultUpdate_pass(9990,
                                               pmic_misc_tests,
                                               PMIC_MISC_NUM_OF_TESTCASES);

                }
#else
                /*Refer the Bug PDK-10451 for more details*/
                pmic_log("\n Test case is disabled!!!\n");
#endif
               break;
            case 13U:
#if defined(SOC_J721E)
                pmic_log("\r\n Run the test only on Leo PMIC-A PG2.0 after Board Power cycle !!!\n");
                pmic_log("\r\n Enter 1 to continue");
                UART_scanFmt("%d", &userInput);

                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD, false))
                {
                    /* Run MISC test cases for Leo PMIC-A */
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* MISC Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_misc_testApp())
                    {
                        test_pmic_run_testcases();
                    }
                    /* Deinit pmic handle */
                    if(pPmicCoreHandle != NULL)
                    {
                       test_pmic_appDeInit(pPmicCoreHandle);
                    }
                }
#else
                pmic_log("\nInvalid Board!!!\n");
#endif
                break;
           case 14U:
               pmic_log(" \r\n Back to Test Menu options\n");
               return;
           default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

/*!
 * \brief   Function to register MISC Unity Test App wrapper to Unity framework
 */
static void test_pmic_misc_testapp_runner(void)
{
    /* @description : Test runner for misc Test App
     *
     * @requirements: PDK-5827
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t option = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestMenu);
        if(UART_scanFmt("%d", &option) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(option)
        {
            case PMIC_UT_AUTOMATE_OPTION:
                test_pmic_misc_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_misc_testapp_run_options(PMIC_UT_MANUAL_OPTION);
               break;
            case 2U:
                pmic_log(" \r\n Quit from application\n");
                return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}
#endif

/*!
 * \brief   TI RTOS specific MISC TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_print_banner("PMIC Misc Unity Test Application");

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_misc_testapp_runner();
#endif
}
