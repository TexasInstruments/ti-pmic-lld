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

static uint8_t pmic_device_info = 0U;

/*!
 * \brief   PMIC ESM Test Cases
 */
static Pmic_Ut_Tests_t pmic_esm_tests[] =
{
    /*! TestID
     *  TestDesc
     */
     {
         0,
         "Pmic_esmStart: Test to Start and Stop ESM MCU"
     },
     {
         1,
         "Pmic_esmStart: Test to Start and Stop ESM SOC"
     },
     {
         2,
         "Pmic_esmStart: Parameter validation for handle"
     },
     {
         3,
         "Pmic_esmEnable: Test to Enable and Disable ESM MCU"
     },
     {
         4,
         "Pmic_esmEnable: Test to Enable and Disable ESM SOC"
     },
     {
         5,
         "Pmic_esmEnable: Parameter validation for handle"
     },
     {
         6,
         "Pmic_esmGetEnableState: Test to verify ESM MCU Enable readback"
     },
     {
         7,
         "Pmic_esmGetEnableState: Test to verify ESM SOC Enable readback"
     },
     {
         8,
         "Pmic_esmGetEnableState: Parameter validation for handle"
     },
     {
         9,
         "Pmic_esmGetEnableState: Parameter validation for pEsmState"
     },
     {
         10,
         "Pmic_esmSetConfiguration: Test to configure ESM MCU in Level Mode"
     },
     {
         11,
         "Pmic_esmSetConfiguration: Test to configure ESM SOC in Level Mode"
     },
     {
         12,
         "Pmic_esmSetConfiguration: Test to configure ESM MCU in PWM Mode"
     },
     {
         13,
         "Pmic_esmSetConfiguration: Test to configure ESM SOC in PWM Mode"
     },
     {
         14,
         "Pmic_esmSetConfiguration: Parameter validation for handle"
     },
     {
         15,
         "Pmic_esmSetConfiguration: Parameter validation for esmDelay1"
     },
     {
         16,
         "Pmic_esmSetConfiguration: Parameter validation for esmDelay2"
     },
     {
         17,
         "Pmic_esmSetConfiguration: Parameter validation for esmHMAX"
     },
     {
         18,
         "Pmic_esmSetConfiguration: Parameter validation for esmHMIN"
     },
     {
         19,
         "Pmic_esmSetConfiguration: Parameter validation for esmLMAX"
     },
     {
         20,
         "Pmic_esmSetConfiguration: Parameter validation for esmLMIN"
     },
     {
         21,
         "Pmic_esmSetConfiguration: Parameter validation for esmErrCntThr"
     },
     {
         22,
         "Pmic_esmGetConfiguration: Test to verify PMIC ESM MCU Get Configuration"
     },
     {
         23,
         "Pmic_esmGetConfiguration: Test to verify PMIC ESM SOC Get Configuration"
     },
     {
         24,
         "Pmic_esmGetConfiguration: Parameter validation for handle"
     },
     {
         25,
         "Pmic_esmGetConfiguration: Parameter validation for pEsmCfg"
     },
     {
         26,
         "Pmic_esmGetErrCnt: Test to read the current ESM MCU Error Count Value"
     },
     {
         27,
         "Pmic_esmGetErrCnt: Test to read the current ESM SOC Error Count Value"
     },
     {
         28,
         "Pmic_esmGetErrCnt: Parameter validation for handle"
     },
     {
         29,
         "Pmic_esmGetErrCnt: Parameter validation for pEsmErrCnt"
     },
     {
         30,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode RST Interrupt"
     },
     {
         31,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode RST Interrupt"
     },
     {
         32,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode FAIL Interrupt"
     },
     {
         33,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode FAIL Interrupt"
     },
     {
         34,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN Interrupt"
     },
     {
         35,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU PWM Mode PIN Interrupt"
     },
     {
         36,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode RST Interrupt"
     },
     {
         37,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode RST Interrupt"
     },
     {
         38,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode FAIL Interrupt"
     },
     {
         39,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode FAIL Interrupt"
     },
     {
         40,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC Level Mode PIN Interrupt"
     },
     {
         41,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN Interrupt"
     },
     {
         42,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN, Fail and RST Interrupts"
     },
     {
         43,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN, Fail and RST Interrupts"
     },
     {
         44,
         "Pmic_esmSetInterrupt: Test to verify ESM MCU Level Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         45,
         "Pmic_esmSetInterrupt: Test to verify ESM SOC PWM Mode PIN, Fail and RST Interrupts disabled"
     },
     {
         46,
         "Pmic_esmSetInterrupt: Parameter validation for handle"
     }
};

/*!
 * \brief   Test to verify ESM MCU Start and Stop functionality
 */
static void test_pmic_esm_startEsm_esmMcuStart(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_MCU;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(0,
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
}

/*!
 * \brief   Test to verify ESM SOC Start and Stop functionality
 */
static void test_pmic_esm_startEsm_esmSocStart(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(1,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
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
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_startEsmPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;
    bool esmState       = PMIC_ESM_START;

    test_pmic_print_unity_testcase_info(2,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(NULL, esmType, esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
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

    test_pmic_print_unity_testcase_info(3,
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

    test_pmic_print_unity_testcase_info(4,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
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
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_pmic_esm_enableEsmPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmToggle     = PMIC_ESM_ENABLE;

    test_pmic_print_unity_testcase_info(5,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmEnable(NULL, esmType, esmToggle);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
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

    test_pmic_print_unity_testcase_info(6,
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

    test_pmic_print_unity_testcase_info(7,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
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
}

/*!
 * \brief Parameter Validation for handle
 */
static void test_pmic_esm_getEnableStatePrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;
    bool esmState      = false;

    test_pmic_print_unity_testcase_info(8,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetEnableState(NULL, esmType, &esmState);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief Parameter Validation for pEsmState
 */
static void test_pmic_esm_getEnableStatePrmValTest_esmState(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(9,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetEnableState(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
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

    test_pmic_print_unity_testcase_info(10,
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

    test_pmic_print_unity_testcase_info(11,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
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

    test_pmic_print_unity_testcase_info(12,
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

    test_pmic_print_unity_testcase_info(13,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
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

    test_pmic_print_unity_testcase_info(14,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(NULL, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
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

    test_pmic_print_unity_testcase_info(15,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
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

    test_pmic_print_unity_testcase_info(16,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
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

    test_pmic_print_unity_testcase_info(17,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
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

    test_pmic_print_unity_testcase_info(18,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
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

    test_pmic_print_unity_testcase_info(19,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
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

    test_pmic_print_unity_testcase_info(20,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_ESM_VAL, pmicStatus);
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

    test_pmic_print_unity_testcase_info(21,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_esmSetConfiguration(pPmicCoreHandle, esmType, esmCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_PARAM, pmicStatus);
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

    test_pmic_print_unity_testcase_info(22,
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

    test_pmic_print_unity_testcase_info(23,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        TEST_IGNORE();
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

    test_pmic_print_unity_testcase_info(24,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(NULL, esmType, &esmCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for pEsmCfg
 */
static void test_esm_getConfigurationPrmValTest_pEsmCfg(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    bool esmType            = PMIC_ESM_MODE_MCU;

    test_pmic_print_unity_testcase_info(25,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetConfiguration(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
}

/*!
 *  \brief Test to read to current ESM MCU Error Count value
 */
static void test_esm_getErrCnt_esmMcu(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_MCU;

    test_pmic_print_unity_testcase_info(26,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 *  \brief Test to read to current ESM SOC Error Count value
 */
static void test_esm_getErrCnt_esmSoc(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType       = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(27,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

/*!
 * \brief   Parameter validation for handle
 */
static void test_esm_getErrCntPrmValTest_handle(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t esmErrCnt  = 0U;
    bool esmType        = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(28,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(NULL, esmType, &esmErrCnt);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
}

/*!
 * \brief   Parameter validation for pEsmErrCnt
 */
static void test_esm_getErrCntPrmValTest_pEsmErrCnt(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    bool esmType        = PMIC_ESM_MODE_SOC;

    test_pmic_print_unity_testcase_info(29,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmGetErrCnt(pPmicCoreHandle, esmType, NULL);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_NULL_PARAM, pmicStatus);
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

    test_pmic_print_unity_testcase_info(30,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    TEST_IGNORE();

    if(J721E_LEO_PMICB_DEVICE == pmic_device_info)
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(31,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(32,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(33,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(34,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(35,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(36,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(37,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(38,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(39,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(40,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(41,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(42,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(43,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    while(1U)
    {
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

    test_pmic_print_unity_testcase_info(44,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_STOP);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
}

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

    test_pmic_print_unity_testcase_info(45,
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

    pmicStatus = Pmic_esmStart(pPmicCoreHandle, esmType, PMIC_ESM_START);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

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

    test_pmic_print_unity_testcase_info(46,
                                        pmic_esm_tests,
                                        PMIC_ESM_NUM_OF_TESTCASES);

    pmicStatus = Pmic_esmSetInterrupt(NULL, esmType, esmIntrCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_ERR_INV_HANDLE, pmicStatus);
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
    RUN_TEST(test_esm_setInterrupt_esmMcuRstIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuRstIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuFailIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuFailIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuPinIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuPinIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocRstIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocRstIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocFailIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocFailIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocPinIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocPinIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmSocAllIntr_pwmMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuAllIntr_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmMcuAllIntrDisabled_levelMode);
    RUN_TEST(test_esm_setInterrupt_esmSocAllIntrDisable_pwmMode);
    RUN_TEST(test_esm_setInterruptPrmValTest_handle);

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

    pmicConfigData.slaveAddr           = LEO_PMICA_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICA_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

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

    pmicConfigData.slaveAddr           = LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

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

    pmicConfigData.slaveAddr           = HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr         = HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

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

static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC B on J721E EVM Using I2C Interface)"
    " \r\n 2: Pmic Hera device"
    " \r\n 3: Pmic Leo device(PMIC A on J721E EVM Using SPI Stub Functions)"
    " \r\n 4: quit"
    " \r\n"
    " \r\n Enter option: "
};

/*!
 * \brief   Function to register ESM Unity Test App wrapper to Unity framework
 */
static void test_pmic_esm_testapp_runner(void)
{
    /* @description : Test runner for ESM Test App
     *
     * @requirements: PDK-5833, PDK-5846
     *
     * @cores       : mcu1_0, mcu1_1
     */

    int8_t num = -1;

    while(1U)
    {
        pmic_log("%s", pmicTestAppMenu);
        if(UART_scanFmt("%d", &num) != 0U)
        {
            pmic_log("Read from UART Console failed\n");
            return;
        }

        switch(num)
        {
           case 0U:
               /* ESM Unity Test App wrapper Function for LEO PMIC-A */
               test_pmic_leo_pmicA_esm_testApp();
               pmic_device_info = J721E_LEO_PMICA_DEVICE;
               /* Run esm test cases for Leo PMIC-A */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 1U:
               /* ESM Unity Test App wrapper Function for LEO PMIC-B */
               test_pmic_leo_pmicB_esm_testApp();
               pmic_device_info = J721E_LEO_PMICB_DEVICE;
               pmic_log(" \r\n ESM feature cannot be tested on PMIC-B\n");

               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 2U:
               /* ESM Unity Test App wrapper Function for HERA PMIC */
               test_pmic_hera_esm_testApp();
               /* Run esm test cases for Hera PMIC */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 3U:
               /* ESM Unity Test App wrapper Function for LEO PMIC-A using
                * SPI stub functions */
               test_pmic_leo_pmicA_spiStub_esm_testApp();
               pmic_device_info = J721E_LEO_PMICA_DEVICE;
               /* Run esm test cases for Leo PMIC-A */
               test_pmic_run_testcases();
               /* Deinit pmic handle */
               if(pPmicCoreHandle != NULL)
               {
                   test_pmic_appDeInit(pPmicCoreHandle);
               }
               break;
           case 4U:
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

    pmic_log("ESM Unity Test Application(%s %s)\n", __TIME__, __DATE__);
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_esm_testapp_runner();
#endif
}
