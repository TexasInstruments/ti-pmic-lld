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
 *  \file   pmic_ut_esm.c
 *
 *  \brief  PMIC Unit Test for testing PMIC ESM APIs
 *
 */

#include <pmic_ut_esm.h>

/* Pointer holds the pPmicCoreHandle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;
extern Pmic_Ut_FaultInject_t gPmic_faultInjectCfg;

/*!
 * \brief   PMIC ESM Test Cases
 */
static Pmic_Ut_Tests_t pmic_esm_tests[] =
{
    /*! TestID
     *  TestDesc
     */
     {
         7769,
         "Pmic_esmStart: Test to Start and Stop ESM MCU"
     },
     {
         7770,
         "Pmic_esmStart: Test to Start and Stop ESM SOC"
     },
     {
         7771,
         "Pmic_esmStart: Parameter validation for handle"
     },
     {
         7772,
         "Pmic_esmEnable: Test to Enable and Disable ESM MCU"
     },
     {
         7773,
         "Pmic_esmEnable: Test to Enable and Disable ESM SOC"
     },
     {
         7774,
         "Pmic_esmEnable: Parameter validation for handle"
     },
     {
         7775,
         "Pmic_esmGetEnableState: Test to verify ESM MCU Enable readback"
     },
     {
         7776,
         "Pmic_esmGetEnableState: Test to verify ESM SOC Enable readback"
     },
     {
         7777,
         "Pmic_esmGetEnableState: Parameter validation for handle"
     },
     {
         7778,
         "Pmic_esmGetEnableState: Parameter validation for pEsmState"
     },
     {
         7779,
         "Pmic_esmSetConfiguration: Test to configure ESM MCU in Level Mode"
     },
     {
         7780,
         "Pmic_esmSetConfiguration: Test to configure ESM SOC in Level Mode"
     },
     {
         7781,
         "Pmic_esmSetConfiguration: Test to configure ESM MCU in PWM Mode"
     },
     {
         7782,
         "Pmic_esmSetConfiguration: Test to configure ESM SOC in PWM Mode"
     },
     {
         7783,
         "Pmic_esmSetConfiguration: Parameter validation for handle"
     },
     {
         7784,
         "Pmic_esmSetConfiguration: Parameter validation for esmDelay1"
     },
     {
         7785,
         "Pmic_esmSetConfiguration: Parameter validation for esmDelay2"
     },
     {
         7786,
         "Pmic_esmSetConfiguration: Parameter validation for esmHMAX"
     },
     {
         7787,
         "Pmic_esmSetConfiguration: Parameter validation for esmHMIN"
     },
     {
         7789,
         "Pmic_esmSetConfiguration: Parameter validation for esmLMAX"
     },
     {
         7790,
         "Pmic_esmSetConfiguration: Parameter validation for esmLMIN"
     },
     {
         7791,
         "Pmic_esmSetConfiguration: Parameter validation for esmErrCntThr"
     },
     {
         7792,
         "Pmic_esmGetConfiguration: Test to verify PMIC ESM MCU Get Configuration"
     },
     {
         7793,
         "Pmic_esmGetConfiguration: Test to verify PMIC ESM SOC Get Configuration"
     },
     {
         7794,
         "Pmic_esmGetConfiguration: Parameter validation for handle"
     },
     {
         7795,
         "Pmic_esmGetConfiguration: Parameter validation for pEsmCfg"
     },
     {
         7796,
         "Pmic_esmGetErrCnt: Test to read the current ESM MCU Error Count Value"
     },
     {
         7797,
         "Pmic_esmGetErrCnt: Test to read the current ESM SOC Error Count Value"
     },
     {
         7798,
         "Pmic_esmGetErrCnt: Parameter validation for handle"
     },
     {
         7799,
         "Pmic_esmGetErrCnt: Parameter validation for pEsmErrCnt"
     },
     {
         7836,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode RST Interrupt"
     },
     {
         7837,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode RST Interrupt"
     },
     {
         7838,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode FAIL Interrupt"
     },
     {
         7839,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode FAIL Interrupt"
     },
     {
         7840,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN Interrupt"
     },
     {
         7841,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode PIN Interrupt"
     },
     {
         0xAB22,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode RST Interrupt"
     },
     {
         0xAB23,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode RST Interrupt"
     },
     {
         0xAB24,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode FAIL Interrupt"
     },
     {
         0xAB25,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode FAIL Interrupt"
     },
     {
         0xAB26,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode PIN Interrupt"
     },
     {
         0xAB27,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN Interrupt"
     },
     {
         0xAB28,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN, Fail and RST Interrupts"
     },
     {
         7849,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN, Fail and RST Interrupts"
     },
     {
         7850,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         0xAB29,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         7852,
         "Pmic_esmSetInterrupt: Parameter validation for handle"
     },
     {
         7853,
         "Pmic_esmStart : Negative test to verify ESM SOC Start for HERA"
     },
     {
         7854,
         "Pmic_esmEnable : Negative test to verify ESM SOC Enable for HERA"
     },
     {
         7855,
         "Pmic_esmSetConfiguration : Negative test to verify PMIC ESM SOC Set configuration for ESM SOC Level Mode for HERA"
     },
     {
         7856,
         "Pmic_esmGetConfiguration : Negative test to verify PMIC ESM SOC Get configuration for ESM SOC Level Mode for HERA"
     },
     {
         7857,
         "Pmic_esmGetErrCnt : Negative test to verify Error count value for HERA"
     },
     {
         7858,
         "Pmic_esmSetInterrupt : Negative test to verify ESM interrupt enable for HERA"
     },
     {
         7859,
         "Pmic_esmGetEnableState : Negative test to verify ESM get state for HERA"
     },
     {
         0xAB2A,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode PIN, Fail and RST Interrupts"
     },
     {
         8009,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode PIN, Fail and RST Interrupts"
     },
     {
         0xAB2B,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         8011,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         9876,
         "Pmic_esmGetStatus: Test to verify PMIC ESM MCU Get Status for Start and Stop functionality"
     },
     {
         9877,
         "Pmic_esmGetStatus: Test to verify PMIC ESM SOC Get Status for Start and Stop functionality"
     },
     {
         9878,
         "Pmic_esmGetStatus: Parameter validation for handle"
     },
     {
         9879,
         "Pmic_esmGetStatus: Parameter validation for pEsmState"
     },
     {
         9880,
         "Pmic_esmGetStatus: Negative test to verify ESM SOC Get Status for HERA"
     },
     {
         10550,
          "Pmic_esmSetInterrupt : Test to disable PMIC ESM SOC Mode PIN, FAIL, RST Interrupts"
     },
     {
         10551,
          "Pmic_esmSetConfiguration : Parameter validation for ESM HMAX Value"
     },
     {
         10552,
          "Pmic_esmSetConfiguration : Parameter validation for ESM HMIN Value"
     },
     {
         10553,
          "Pmic_esmSetConfiguration : Parameter validation for ESM LMAX Value"
     },
     {
         10554,
          "Pmic_esmSetConfiguration : Parameter validation for ESM LMIN Value"
     },
     {
         10555,
          "Pmic_esmSetConfiguration : Test to Disable DRV clear configuration"
     },
     {
         10556,
          "Pmic_esmSetConfiguration : Parameter validation for ValidParams"
     },
     {
         10557,
          "Pmic_esmGetConfiguration : Parameter validation for ValidParams"
     },
     {
         9004,
          "Pmic_esmTests: Dynamic Coverage Gaps and Fault Injection Tests"
     },
};

/*!
 * \brief   Test to verify ESM MCU Start and Stop functionality
 */
static void test_pmic_esm_startEsm_esmMcuStart(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(7769,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ESM_STARTED, pmicStatus);

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle,esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7769,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify ESM SOC Start and Stop functionality
 */
static void test_pmic_esm_startEsm_esmSocStart(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(7770,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7770,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ESM_STARTED, pmicStatus);

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle,esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7770,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_startEsmPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;
    bool esmState       = PMIC_ESM_STOP;

    test_pmic_print_unity_testcase_info(7771,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(NULL, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7771,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify ESM MCU Enable and Disable functionality
 */
static void test_pmic_esm_enableEsm_esmMcuEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7772,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_ENABLE, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_DISABLE, esmState);

    pmic_testResultUpdate_pass(7772,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify ESM SOC Enable and Disable functionality
 */
static void test_pmic_esm_enableEsm_esmSocEnable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7773,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7773,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_ENABLE, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_DISABLE, esmState);

    pmic_testResultUpdate_pass(7773,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_enableEsmPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;

    test_pmic_print_unity_testcase_info(7774,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmEnable(NULL, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7774,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify ESM MCU Enable readback
 */
static void test_pmic_esm_getEnableState_esmMcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7775,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(true, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(false, esmState);

    pmic_testResultUpdate_pass(7775,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify ESM SOC Enable readback
 */
static void test_pmic_esm_getEnableState_esmSoc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7776,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7776,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(true, esmState);

    esmToggle = PMIC_ESM_DISABLE;
    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(false, esmState);

    pmic_testResultUpdate_pass(7776,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief Parameter Validation for handle
 */
static void test_pmic_esm_getEnableStatePrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(7777,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetEnableState(NULL, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7777,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief Parameter Validation for pEsmState
 */
static void test_pmic_esm_getEnableStatePrmValTest_esmState(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7778,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7778,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Set configuration for ESM MCU
 *          Level Mode
 */
static void test_pmic_esm_setConfiguration_esmMcuLevelMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7779,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);

    pmic_testResultUpdate_pass(7779,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM SOC Set configuration for ESM SOC
 *          Level Mode
 */
static void test_pmic_esm_setConfiguration_esmSocLevelMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7780,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7780,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);

    pmic_testResultUpdate_pass(7780,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Set configuration for ESM MCU
 *          PWM Mode and readback the ESM configuration
 */
static void test_pmic_esm_setConfiguration_esmMcuPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7781,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);

    pmic_testResultUpdate_pass(7781,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM SOC Set configuration for ESM SOC
 *          PWM Mode and read back the ESM configuration.
 */
static void test_pmic_esm_setConfiguration_esmSocPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7782,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7782,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);

    pmic_testResultUpdate_pass(7782,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_setConfigurationPrmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7783,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(NULL, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7783,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for esmDelay1
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmDelay1(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        522401,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7784,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(7784,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for esmDelay2
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmDelay2(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        2048U,
        522401U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7785,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(7785,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for HMAX
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmHmax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        0U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7786,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(7786,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for HMIN
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmHmin(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        0U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7787,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(7787,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for LMAX
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmLmax(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        30U,
        0U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7789,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(7789,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for LMIN
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmLmin(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        0U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7790,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(7790,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for ErrCntThr
 */
static void test_pmic_esm_setConfigurationPrmValTest_esmErrCntThr(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        16U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7791,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7791,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Get configuration
 */
static void test_pmic_esm_getConfiguration_esmMcuPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7792,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmEnDrv, esmCfg_rd.esmEnDrv);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);

    pmic_testResultUpdate_pass(7792,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM SOC Get configuration
 */
static void test_pmic_esm_getConfiguration_esmSocPwmMode(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT | PMIC_ESM_CFG_EN_DRV_VALID_SHIFT | PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    test_pmic_print_unity_testcase_info(7793,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7793,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmDelay1_us, esmCfg_rd.esmDelay1_us);
    TEST_ASSERT_EQUAL(esmCfg.esmDelay2_us, esmCfg_rd.esmDelay2_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmax_us, esmCfg_rd.esmHmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmHmin_us, esmCfg_rd.esmHmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmax_us, esmCfg_rd.esmLmax_us);
    TEST_ASSERT_EQUAL(esmCfg.esmLmin_us, esmCfg_rd.esmLmin_us);
    TEST_ASSERT_EQUAL(esmCfg.esmMode, esmCfg_rd.esmMode);
    TEST_ASSERT_EQUAL(esmCfg.esmEnDrv, esmCfg_rd.esmEnDrv);
    TEST_ASSERT_EQUAL(esmCfg.esmErrCntThr, esmCfg_rd.esmErrCntThr);

    pmic_testResultUpdate_pass(7793,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_esm_getConfigurationPrmValTest_handle(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_HMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_HMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMAX_VALID_SHIFT   |
                               PMIC_ESM_CFG_LMIN_VALID_SHIFT   |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT   |
                               PMIC_ESM_CFG_EN_DRV_VALID_SHIFT |
                               PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT,};

    test_pmic_print_unity_testcase_info(7794,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(NULL, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7794,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pEsmCfg
 */
static void test_esm_getConfigurationPrmValTest_pEsmCfg(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;

    test_pmic_print_unity_testcase_info(7795,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7795,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 *  \brief Test to read to current ESM MCU Error Count value
 */
static void test_esm_getErrCnt_esmMcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_MCU;

    test_pmic_print_unity_testcase_info(7796,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7796,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 *  \brief Test to read to current ESM SOC Error Count value
 */
static void test_esm_getErrCnt_esmSoc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7797,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(7797,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }
    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmic_testResultUpdate_pass(7797,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_esm_getErrCntPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType        = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7798,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(NULL, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7798,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pEsmErrCnt
 */
static void test_esm_getErrCntPrmValTest_pEsmErrCnt(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7799,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        pmic_testResultUpdate_ignore(7799,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(7799,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmMcuRstIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7836,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
       /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmMcuRstIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7837,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
       /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmMcuFailIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7838,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmMcuFailIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7839,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmMcuPinIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7840,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmMcuPinIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7841,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode PIN, FAIL and RST Interrupts
 */
static void test_esm_setInterrupt_esmMcuAllIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7849,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    if(J7VCL_HERA_PMICB_DEVICE == pmic_device_info)
    {
        TEST_IGNORE();
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU Level Mode PIN, FAIL and RST Interrupts
 *         disabled
 */
static void test_esm_setInterrupt_esmMcuAllIntrDisabled_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t pin               = 7U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(7850,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode PIN, FAIL, RST Interrupt
 */
static void test_esm_setInterrupt_esmMcuAllIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 7U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(8009,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_PIN_INT);
                break;
            }
        }
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_FAIL_INT);
                break;
            }
        }
                if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_MCU_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_MCU_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to verify PMIC ESM MCU PWM Mode PIN, FAIL, RST Interrupts
 *         disabled
 */
static void test_esm_setInterrupt_esmMcuAllIntrDisabled_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_MCU;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t pin               = 7U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg    =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO7_NERR_MCU,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(8011,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_PIN_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_FAIL_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_MCU_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_MCU_RST_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_SUCCESS;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief Parameter validation for handle
 */
static void test_esm_setInterruptPrmValTest_handle(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    test_pmic_print_unity_testcase_info(7852,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmSetInterrupt(NULL, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(7852,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_esmStart : Negative test to verify ESM SOC Start for HERA
 */
static void test_pmic_esm_startEsm_esmSocStart_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_STOP;

    test_pmic_print_unity_testcase_info(7853,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7853,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7853,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmEnable : Negative test to verify ESM SOC Enable for HERA
 */
static void test_pmic_esm_startEsm_esmSocEnable_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7854,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7854,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7854,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmSetConfiguration : Negative test to verify PMIC ESM SOC Set configuration for ESM SOC
 *          Level Mode for HERA
 */
static void test_pmic_esm_setConfiguration_esmSocLevelMode_hera(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;

    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(7855,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7855,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7855,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmGetConfiguration : Negative test to verify PMIC ESM SOC Get configuration for ESM SOC
 *          Level Mode for HERA
 */
static void test_pmic_esm_getConfiguration_esmSocLevelMode_hera(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_SOC;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_DELAY1_VALID_SHIFT |
                               PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
                               PMIC_ESM_CFG_MODE_VALID_SHIFT,};


    test_pmic_print_unity_testcase_info(7856,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7856,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle,
                                          esmType,
                                          &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7856,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_esmGetErrCnt : Negative test to verify Error count value for HERA
 */
static void test_Pmic_esmGetErrCnt_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(7857,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7857,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);

    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7857,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}


/*!
 * \brief   Pmic_esmSetInterrupt : Negative test to verify ESM interrupt enable for HERA
 */
static void test_Pmic_esmSetInterrupt_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    test_pmic_print_unity_testcase_info(7858,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7858,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);

    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7858,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmGetEnableState : Negative test to verify ESM get state for HERA
 */
static void test_Pmic_esmGetEnableState_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_STOP;

    test_pmic_print_unity_testcase_info(7859,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(7859,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmState);

    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(7859,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify PMIC ESM MCU Get Status for Start and Stop
 *          functionality
 */
static void test_pmic_esm_getStatusEsmMcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmState      = PMIC_ESM_START;
    bool esmState_rd;

    test_pmic_print_unity_testcase_info(9876,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ESM_STARTED, pmicStatus);

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle, esmType, &esmState_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_START, esmState_rd);

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle,esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle, esmType, &esmState_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_STOP, esmState_rd);

    pmic_testResultUpdate_pass(9876,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Test to verify ESM SOC Get Status for Start and Stop
 *          functionality
 */
static void test_pmic_esm_getStatusEsmSoc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_START;
    bool esmState_rd;

    test_pmic_print_unity_testcase_info(9877,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9877,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_ESM_STARTED, pmicStatus);

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle, esmType, &esmState_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_START, esmState_rd);

    esmState = PMIC_ESM_STOP;
    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle, esmType, &esmState_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(PMIC_ESM_STOP, esmState_rd);

    pmic_testResultUpdate_pass(9877,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_getStatusPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;
    bool esmState;

    test_pmic_print_unity_testcase_info(9878,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetStatus(NULL, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);

    pmic_testResultUpdate_pass(9878,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for pEsmState
 */
static void test_pmic_esm_getStatusPrmValTest_pEsmState(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType;

    test_pmic_print_unity_testcase_info(9879,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        esmType        = PMIC_ESM_MODE_SOC;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        esmType        = PMIC_ESM_MODE_MCU;
    }

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);

    pmic_testResultUpdate_pass(9879,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Negative Test to verify ESM SOC Get Status for HERA
 */
static void test_pmic_esm_getStatusEsmSoc_hera(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState;

    test_pmic_print_unity_testcase_info(9880,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmic_testResultUpdate_ignore(9880,
                                     pmic_esm_tests,
                                     PMIC_ESM_NUM_OF_TESTCASES);
    }

    pmicStatus = Pmic_esmGetStatus(pPmicCoreHandle, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, pmicStatus);

    pmic_testResultUpdate_pass(9880,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmSetInterrupt : Test to disable PMIC ESM SOC Mode PIN, FAIL
 *          RST Interrupts
 *          Functionality is not tested due to Known issue - PDK-8333
 */
static void test_pmic_esm_setInterrupt_esmSocAllIntrDisabled(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };
    bool maskStatus;

    test_pmic_print_unity_testcase_info(10550,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus =Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT,
                                              &maskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(maskStatus, PMIC_IRQ_MASK);

        pmicStatus =Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT,
                                              &maskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(maskStatus, PMIC_IRQ_MASK);

        pmicStatus =Pmic_irqGetMaskIntrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT,
                                              &maskStatus);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
        TEST_ASSERT_EQUAL(maskStatus, PMIC_IRQ_MASK);

    }

    pmic_testResultUpdate_pass(10550,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for ESM HMAX Value
 */
static void test_pmic_esm_setConfigurationPrmValTest_hmaxValue(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(10551,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    esmCfg.validParams = PMIC_ESM_CFG_HMAX_VALID_SHIFT;
    esmCfg.esmHmax_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(10551,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for ESM HMIN Value
 */
static void test_pmic_esm_setConfigurationPrmValTest_hminValue(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(10552,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    esmCfg.validParams = PMIC_ESM_CFG_HMIN_VALID_SHIFT;
    esmCfg.esmHmin_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(10552,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for ESM LMAX Value
 */
static void test_pmic_esm_setConfigurationPrmValTest_lmaxValue(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(10553,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    esmCfg.validParams = PMIC_ESM_CFG_LMAX_VALID_SHIFT;
    esmCfg.esmLmax_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(10553,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Parameter validation for ESM LMIN Value
 */
static void test_pmic_esm_setConfigurationPrmValTest_lminValue(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(10554,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    esmCfg.validParams = PMIC_ESM_CFG_LMIN_VALID_SHIFT;
    esmCfg.esmLmin_us = 3850U;
    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);

    pmic_testResultUpdate_pass(10554,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmSetConfiguration : Test to Disable DRV clear configuration
 */
static void test_pmic_esm_setConfiguration_disableDrvClear(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {PMIC_ESM_CFG_EN_DRV_VALID_SHIFT};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_EN_DRV_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_DISABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(10555,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(esmCfg.esmEnDrv, esmCfg_rd.esmEnDrv);

    pmic_testResultUpdate_pass(10555,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmSetConfiguration : Parameter validation for ValidParams
 */
static void test_pmic_esm_setconfiguration_PrmValTest_validParams(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg =
    {
        0U,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(10556,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, pmicStatus);

    pmic_testResultUpdate_pass(10556,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

/*!
 * \brief   Pmic_esmGetConfiguration : Parameter validation for ValidParams
 */
static void test_pmic_esm_getconfiguration_PrmValTest_validParams(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    Pmic_EsmCfg_t esmCfg_rd = {0U};

    test_pmic_print_unity_testcase_info(10557,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INSUFFICIENT_CFG, pmicStatus);

    pmic_testResultUpdate_pass(10557,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

#if defined(ENABLE_SAMPLE_TESTCASES)
/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmSocRstIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB22,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode RST Interrupt
 */
static void test_esm_setInterrupt_esmSocRstIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB23,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmSocFailIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB24,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode FAIL Interrupt
 */
static void test_esm_setInterrupt_esmSocFailIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        true,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB25,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmSocPinIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB26,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode PIN Interrupt
 */
static void test_esm_setInterrupt_esmSocPinIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    uint8_t esmErrCnt         = 0U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB27,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode PIN, FAIL and RST Interrupts
 */
static void test_esm_setInterrupt_esmSocAllIntr_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT | PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT | PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT | PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB28,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC PWM Mode PIN, FAIL and RST Interrupts
 *         disabled
 */
static void test_esm_setInterrupt_esmSocAllIntrDisable_pwmMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType              = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
        PMIC_ESM_CFG_HMAX_VALID_SHIFT | PMIC_ESM_CFG_HMIN_VALID_SHIFT |
        PMIC_ESM_CFG_LMAX_VALID_SHIFT | PMIC_ESM_CFG_LMIN_VALID_SHIFT |
        PMIC_ESM_CFG_MODE_VALID_SHIFT ,
        4096U,
        0U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_PWM_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB29,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode PIN, FAIL and RST Interrupt
 */
static void test_esm_setInterrupt_esmSocAllIntr_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t  irqNum           = 0U;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
        PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        true,
        true,
        true
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB2A,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_PIN_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_PIN_INT);
                break;
            }
        }
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_FAIL_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_FAIL_INT);
                break;
            }
        }
        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) != 0U))
        {
            while(PMIC_TPS6594X_ESM_SOC_RST_INT != irqNum)
            {
                pmicStatus = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                                     &errStat,
                                                     &irqNum);
            }

            if(PMIC_ST_SUCCESS == pmicStatus)
            {
                /* clear the interrupt */
                pmicStatus = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                              PMIC_TPS6594X_ESM_SOC_RST_INT);
                break;
            }
        }
    }

    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * Below test cases are not tested because of HW limitation.
 * Added below test cases as sample for reference.
 */

/*!
 *  \brief Test to verify PMIC ESM SOC Level Mode PIN, FAIL and RST Interrupts
 *         disabled
 */
static void test_esm_setInterrupt_esmSocAllIntrDisabled_levelMode(void)
{
    int32_t pmicStatus        = PMIC_ST_SUCCESS;
    bool esmType               = PMIC_ESM_MODE_SOC;
    bool esmToggle            = PMIC_ESM_ENABLE;
    Pmic_IrqStatus_t errStat  = {0U};
    bool clearIRQ             = false;
    uint8_t pin               = 3U;
    int8_t timeout            = 10U;
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT | PMIC_ESM_CFG_DELAY2_VALID_SHIFT |
        PMIC_ESM_CFG_MODE_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };

    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO3_NERR_SOC,
        PMIC_GPIO_HIGH
    };

    test_pmic_print_unity_testcase_info(0xAB2B,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmEnable(pPmicCoreHandle, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    /* To clear the interrupts*/
    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, clearIRQ);

    while(timeout--)
    {
        /* Delay added to avoid timeout */
        Osal_delay(1000);

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_PIN_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_PIN_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_FAIL_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_FAIL_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }

        if((PMIC_ST_SUCCESS == pmicStatus) &&
           ((errStat.intStatus[PMIC_TPS6594X_ESM_SOC_RST_INT/32U] &
            (1U << (PMIC_TPS6594X_ESM_SOC_RST_INT % 32U))) == 0U))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }
    }
    if(0 > timeout)
    {
        pmicStatus = PMIC_ST_ERR_FAIL;
    }
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}
#endif

/*!
 * \brief   Added for Coverage
 */
static void test_pmic_esm_coverageGaps(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool esmType   = PMIC_ESM_MODE_MCU;
    bool esmState  = PMIC_ESM_START;
    bool esmSt_rd;
    Pmic_EsmIntrCfg_t esmIntrCfg =
    {
        false,
        false,
        false
    };

    Pmic_EsmCfg_t esmCfg_rd = {0U};
    Pmic_EsmCfg_t esmCfg =
    {
        PMIC_ESM_CFG_DELAY1_VALID_SHIFT,
        4096U,
        2048U,
        30U,
        30U,
        30U,
        30U,
        4U,
        PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE,
        PMIC_ESM_LEVEL_MODE
    };

    test_pmic_print_unity_testcase_info(9004,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    // Fault Injection Tests
    gPmic_faultInjectCfg.enableFaultInjectionRead = 1U;
    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    status = Pmic_esmEnable(pPmicCoreHandle, esmType, PMIC_ESM_ENABLE);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        gPmic_faultInjectCfg.readCount = 0;
        gPmic_faultInjectCfg.skipReadCount = 1;
        esmType = PMIC_ESM_MODE_SOC;
        status = Pmic_esmSetInterrupt(pPmicCoreHandle, esmType, esmIntrCfg);
        TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);
        esmType = PMIC_ESM_MODE_MCU;
    }

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    esmCfg.validParams = PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT;
    status = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    esmCfg.validParams = PMIC_ESM_CFG_EN_DRV_VALID_SHIFT;
    status = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 2;
    esmCfg.validParams = PMIC_ESM_CFG_MODE_VALID_SHIFT;
    status = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_DELAY1_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_DELAY2_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_HMAX_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_HMIN_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_LMAX_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_LMIN_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_EN_DRV_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    esmCfg_rd.validParams = PMIC_ESM_CFG_MODE_VALID_SHIFT;
    status = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.readCount = 0;
    gPmic_faultInjectCfg.skipReadCount = 1;
    status = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, &esmSt_rd);
    TEST_ASSERT_EQUAL(gPmic_faultInjectCfg.commError, status);

    gPmic_faultInjectCfg.enableFaultInjectionRead = 0U;

    Pmic_DevSubSysInfo_t pmicDevSubSysInfo =
    {
        .gpioEnable = (bool)true,
        .rtcEnable  = (bool)true,
        .wdgEnable  = (bool)true,
        .buckEnable = (bool)true,
        .ldoEnable  = (bool)true,
        .esmEnable  = (bool)false
    };

    pPmicCoreHandle->pPmic_SubSysInfo = (&pmicDevSubSysInfo);
    status = Pmic_esmStart(pPmicCoreHandle, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_DEVICE, status);

    pmic_testResultUpdate_pass(9004,
                               pmic_esm_tests,
                               PMIC_ESM_NUM_OF_TESTCASES);
}

#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run esm unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_esm_tests, PMIC_ESM_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_esm_startEsm_esmMcuStart);
    RUN_TEST(test_pmic_esm_startEsm_esmSocStart);
    RUN_TEST(test_pmic_esm_startEsmPrmValTest_handle);
    RUN_TEST(test_pmic_esm_enableEsm_esmMcuEnable);
    RUN_TEST(test_pmic_esm_enableEsm_esmSocEnable);
    RUN_TEST(test_pmic_esm_enableEsmPrmValTest_handle);
    RUN_TEST(test_pmic_esm_getEnableState_esmMcu);
    RUN_TEST(test_pmic_esm_getEnableState_esmSoc);
    RUN_TEST(test_pmic_esm_getEnableStatePrmValTest_handle);
    RUN_TEST(test_pmic_esm_getEnableStatePrmValTest_esmState);
    RUN_TEST(test_pmic_esm_setConfiguration_esmMcuLevelMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocLevelMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmMcuPwmMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocPwmMode);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_handle);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmDelay1);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmDelay2);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmHmax);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmHmin);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmLmax);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmLmin);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmErrCntThr);
    RUN_TEST(test_pmic_esm_getConfiguration_esmMcuPwmMode);
    RUN_TEST(test_pmic_esm_getConfiguration_esmSocPwmMode);
    RUN_TEST(test_esm_getConfigurationPrmValTest_handle);
    RUN_TEST(test_esm_getConfigurationPrmValTest_pEsmCfg);
    RUN_TEST(test_esm_getErrCnt_esmMcu);
    RUN_TEST(test_esm_getErrCnt_esmSoc);
    RUN_TEST(test_esm_getErrCntPrmValTest_handle);
    RUN_TEST(test_esm_getErrCntPrmValTest_pEsmErrCnt);
    RUN_TEST(test_esm_setInterruptPrmValTest_handle);
    RUN_TEST(test_pmic_esm_startEsm_esmSocStart_hera);
    RUN_TEST(test_pmic_esm_startEsm_esmSocEnable_hera);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocLevelMode_hera);
    RUN_TEST(test_pmic_esm_getConfiguration_esmSocLevelMode_hera);
    RUN_TEST(test_Pmic_esmGetErrCnt_hera);
    RUN_TEST(test_Pmic_esmSetInterrupt_hera);
    RUN_TEST(test_Pmic_esmGetEnableState_hera);
    RUN_TEST(test_pmic_esm_getStatusEsmMcu);
    RUN_TEST(test_pmic_esm_getStatusEsmSoc);
    RUN_TEST(test_pmic_esm_getStatusPrmValTest_handle);
    RUN_TEST(test_pmic_esm_getStatusPrmValTest_pEsmState);
    RUN_TEST(test_pmic_esm_getStatusEsmSoc_hera);

    RUN_TEST(test_pmic_esm_setInterrupt_esmSocAllIntrDisabled);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_hmaxValue);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_hminValue);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_lmaxValue);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_lminValue);
    RUN_TEST(test_pmic_esm_setConfiguration_disableDrvClear);
    RUN_TEST(test_pmic_esm_setconfiguration_PrmValTest_validParams);
    RUN_TEST(test_pmic_esm_getconfiguration_PrmValTest_validParams);
    RUN_TEST(test_pmic_esm_coverageGaps);

    pmic_updateTestResults(pmic_esm_tests, PMIC_ESM_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   Run esm unity test cases
 */
static void test_pmic_run_slave_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_esm_tests, PMIC_ESM_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_esm_startEsmPrmValTest_handle);
    RUN_TEST(test_pmic_esm_enableEsm_esmMcuEnable);
    RUN_TEST(test_pmic_esm_enableEsm_esmSocEnable);
    RUN_TEST(test_pmic_esm_enableEsmPrmValTest_handle);
    RUN_TEST(test_pmic_esm_getEnableState_esmMcu);
    RUN_TEST(test_pmic_esm_getEnableState_esmSoc);
    RUN_TEST(test_pmic_esm_getEnableStatePrmValTest_handle);
    RUN_TEST(test_pmic_esm_getEnableStatePrmValTest_esmState);
    RUN_TEST(test_pmic_esm_setConfiguration_esmMcuLevelMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocLevelMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmMcuPwmMode);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocPwmMode);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_handle);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmDelay1);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmDelay2);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmHmax);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmHmin);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmLmax);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmLmin);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_esmErrCntThr);
    RUN_TEST(test_pmic_esm_getConfiguration_esmMcuPwmMode);
    RUN_TEST(test_pmic_esm_getConfiguration_esmSocPwmMode);
    RUN_TEST(test_esm_getConfigurationPrmValTest_handle);
    RUN_TEST(test_esm_getConfigurationPrmValTest_pEsmCfg);
    RUN_TEST(test_esm_getErrCnt_esmMcu);
    RUN_TEST(test_esm_getErrCnt_esmSoc);
    RUN_TEST(test_esm_getErrCntPrmValTest_handle);
    RUN_TEST(test_esm_getErrCntPrmValTest_pEsmErrCnt);
    RUN_TEST(test_esm_setInterruptPrmValTest_handle);
    RUN_TEST(test_pmic_esm_startEsm_esmSocStart_hera);
    RUN_TEST(test_pmic_esm_startEsm_esmSocEnable_hera);
    RUN_TEST(test_pmic_esm_setConfiguration_esmSocLevelMode_hera);
    RUN_TEST(test_pmic_esm_getConfiguration_esmSocLevelMode_hera);
    RUN_TEST(test_Pmic_esmGetErrCnt_hera);
    RUN_TEST(test_Pmic_esmSetInterrupt_hera);
    RUN_TEST(test_Pmic_esmGetEnableState_hera);
    RUN_TEST(test_pmic_esm_getStatusPrmValTest_handle);
    RUN_TEST(test_pmic_esm_getStatusPrmValTest_pEsmState);
    RUN_TEST(test_pmic_esm_getStatusEsmSoc_hera);

    RUN_TEST(test_pmic_esm_setInterrupt_esmSocAllIntrDisabled);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_hmaxValue);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_hminValue);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_lmaxValue);
    RUN_TEST(test_pmic_esm_setConfigurationPrmValTest_lminValue);
    RUN_TEST(test_pmic_esm_setConfiguration_disableDrvClear);
    RUN_TEST(test_pmic_esm_setconfiguration_PrmValTest_validParams);
    RUN_TEST(test_pmic_esm_getconfiguration_PrmValTest_validParams);
    RUN_TEST(test_pmic_esm_coverageGaps);

    pmic_updateTestResults(pmic_esm_tests, PMIC_ESM_NUM_OF_TESTCASES);

    UNITY_END();
}

/*!
 * \brief   ESM Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_esm_testApp(void)
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
 * \brief   ESM Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_esm_testApp(void)
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
 * \brief   ESM Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_esm_testApp(void)
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

/*!
 * \brief   ESM Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_esm_testApp(void)
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

static int32_t setup_pmic_interrupt(uint32_t board)
{
    int32_t status = PMIC_ST_SUCCESS;

    if(J721E_BOARD == board)
    {
        if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J721E)
        {
            gCrcTestFlag_J721E = PMIC_CFG_TO_ENABLE_CRC;
        }

        pmic_device_info = J721E_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_esm_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_esm_testApp();
            /* Deinit pmic handle */
            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }
    else if(J7VCL_BOARD == board)
    {
        if(PMIC_STATUS_CRC_INIT_VAL == gCrcTestFlag_J7VCL)
        {
            gCrcTestFlag_J7VCL = PMIC_CFG_TO_ENABLE_CRC;
        }

        pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
        status = test_pmic_leo_pmicA_esm_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_esm_testApp();
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
    " \r\n 0: Automatic run for all board specific ESM options"
    " \r\n 1: Manual run for ESM options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM Using I2C Interface)"
    " \r\n 2: Pmic Leo device(PMIC A on J7VCL EVM Using I2C Interface)"
    " \r\n 3: Pmic Hera device(PMIC B on J7VCL EVM Using I2C Interface)"
    " \r\n 4: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 5: Pmic Leo device(PMIC A on J7VCL EVM Using SPI Stub Functions)"
    " \r\n 6: Pmic Leo device(PMIC A on J721E EVM Manual Testcase for ESM Interrupts)"
    " \r\n 7: Pmic Leo device(PMIC A on J7VCL EVM Manual Testcase for ESM Interrupts)"
    " \r\n 8: Pmic Hera device(PMIC B on J7VCL EVM Manual Testcase for ESM Interrupts)"
    " \r\n 9: Back to Test Menu"
    " \r\n"
    " \r\n Enter option: "
};

static const char pmicTestAppManualTestSubMenu[] =
{
    " \r\n ================================================================="
    " \r\n Sub Menu:"
    " \r\n ================================================================="
    " \r\n 0: ESM PIN Interrupt Test"
    " \r\n 1: ESM FAIL Interrupt Test"
    " \r\n 2: ESM RST Interrupt Test"
    " \r\n 3: ESM ALL Interrupt Test"
    " \r\n 4: ESM Disable Interrupt Test"
    " \r\n 5: Back to Manual tests Menu"
    " \r\n"
    " \r\n Enter option: "
};

static void print_pmicTestAppManualTestMenu(uint32_t board)
{
    char board_name[10] = {0};

    if(J721E_BOARD == board)
    {
        strcpy(board_name, "J721E");
    }
    else if(J7VCL_BOARD == board)
    {
        strcpy(board_name, "J7VCL");
    }

    pmic_log(" \r\n =================================================================");
    pmic_log(" \r\n Manual Testcase Menu:");
    pmic_log(" \r\n =================================================================");
    pmic_log(" \r\n 0: Pmic Leo device(PMIC A on %s EVM for ESM MCU Level Mode)", board_name);
    pmic_log(" \r\n 1: Pmic Leo device(PMIC A on %s EVM for ESM MCU PWM Mode)", board_name);
    pmic_log(" \r\n 2: Back to Main Menu");
    pmic_log(" \r\n");
    pmic_log(" \r\n Enter option: ");
}

/*!
 * \brief   Run ESM manual test cases for MCU Level mode
 */
static void test_pmic_run_testcases_mcuLevelMode(void)
{
    int8_t subMenuOption = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppManualTestSubMenu);
        if(UART_scanFmt("%d", &subMenuOption) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
        UNITY_BEGIN();

        switch(subMenuOption)
        {
            case 0U:
                RUN_TEST(test_esm_setInterrupt_esmMcuPinIntr_levelMode);
               break;
            case 1U:
                RUN_TEST(test_esm_setInterrupt_esmMcuFailIntr_levelMode);
               break;
            case 2U:
                RUN_TEST(test_esm_setInterrupt_esmMcuRstIntr_levelMode);
               break;
            case 3U:
                RUN_TEST(test_esm_setInterrupt_esmMcuAllIntr_levelMode);
               break;
            case 4U:
                RUN_TEST(test_esm_setInterrupt_esmMcuAllIntrDisabled_levelMode);
               break;
            case 5U:
               pmic_log(" \r\n Back to Manual tests Menu\n");
               return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
        UNITY_END();
    }
}

/*!
 * \brief   Run ESM manual test cases for MCU PWM mode
 */
static void test_pmic_run_testcases_mcuPwmMode(void)
{
    int8_t subMenuOption = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppManualTestSubMenu);
        if(UART_scanFmt("%d", &subMenuOption) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
        UNITY_BEGIN();

        switch(subMenuOption)
        {
            case 0U:
                RUN_TEST(test_esm_setInterrupt_esmMcuPinIntr_pwmMode);
               break;
            case 1U:
                RUN_TEST(test_esm_setInterrupt_esmMcuFailIntr_pwmMode);
               break;
            case 2U:
                RUN_TEST(test_esm_setInterrupt_esmMcuRstIntr_pwmMode);
               break;
            case 3U:
                RUN_TEST(test_esm_setInterrupt_esmMcuAllIntr_pwmMode);
               break;
            case 4U:
                RUN_TEST(test_esm_setInterrupt_esmMcuAllIntrDisabled_pwmMode);
               break;
            case 5U:
               pmic_log(" \r\n Back to Manual tests Menu\n");
               return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
        UNITY_END();
    }
}

/*!
 * \brief   Run ESM manual test cases
 */
static void test_pmic_run_testcases_manual(uint32_t board)
{
    int8_t menuOption = -1;

    while(1U)
    {
        print_pmicTestAppManualTestMenu(board);
        if(UART_scanFmt("%d", &menuOption) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(menuOption)
        {
            case 0U:
                RUN_TEST(test_pmic_run_testcases_mcuLevelMode);
               break;
            case 1U:
                RUN_TEST(test_pmic_run_testcases_mcuPwmMode);
               break;
            case 2U:
                pmic_log(" \r\n Back to Test Menu\n");
               return;
            default:
                pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

static void test_pmic_esm_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 1, 4};
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {2, 3, 5};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_esm_tests, PMIC_ESM_NUM_OF_TESTCASES);
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
                num = 9;
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* ESM Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_esm_testApp())
                    {
                        /* Run esm test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICB_DEVICE;

                    /* ESM Unity Test App wrapper Function for LEO PMIC-B */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicB_esm_testApp())
                    {
                        pmic_log(
                             " \r\n ESM feature cannot be tested on PMIC-B\n");
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* ESM Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_esm_testApp())
                    {
                        /* Run esm test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                    /* ESM Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_esm_testApp())
                    {
                        /* Run esm test cases for Hera PMIC */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* ESM Unity Test App wrapper Function for LEO PMIC-A using
                     * SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                           test_pmic_leo_pmicA_spiStub_esm_testApp())
                    {
                        /* Run esm test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* ESM Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_esm_testApp())
                    {
                        /* Run esm test cases for Leo PMIC-A */
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* ESM Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_esm_testApp())
                    {
                        /* Run ESM manual test cases */
                        test_pmic_run_testcases_manual(J721E_BOARD);
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
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* ESM Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_esm_testApp())
                    {
                        /* Run ESM manual test cases */
                        test_pmic_run_testcases_manual(J7VCL_BOARD);
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_HERA_PMICB_DEVICE;

                    /* ESM Unity Test App wrapper Function for HERA PMIC */
                    if(PMIC_ST_SUCCESS == test_pmic_hera_esm_testApp())
                    {
                        /* Run ESM manual test cases */
                        test_pmic_run_testcases_manual(J7VCL_BOARD);
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
            case 9U:
               pmic_log(" \r\n Back to Test Menu options\n");
               return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

/*!
 * \brief   Function to register ESM Unity Test App wrapper to Unity framework
 */
static void test_pmic_esm_testapp_runner(void)
{
    /* @description : Test runner for ESM Test App
     *
     * @requirements: 5833, 5846
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
                test_pmic_esm_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_esm_testapp_run_options(PMIC_UT_MANUAL_OPTION);
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
 * \brief   TI RTOS specific ESM TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    pmic_print_banner("PMIC ESM Unity Test Application");
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_esm_testapp_runner();
#endif
}
