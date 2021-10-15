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
 *  \file   pmic_ut_stress.c
 *
 *  \brief  PMIC Unit Test for testing PMIC STRESS APIs
 *
 */

#include <pmic_ut_stress.h>

/* Pointer to Pmic Core Handle */
Pmic_CoreHandle_t *pPmicCoreHandle = NULL;

extern uint16_t pmic_device_info;
extern int32_t gCrcTestFlag_J721E;
extern int32_t gCrcTestFlag_J7VCL;

volatile uint32_t pmic_intr_triggered_stress_tst_timer = 1U;
volatile uint32_t pmic_intr_triggered_stress_tst_alarm = 1U;

/*!
 * \brief   PMIC STRESS Test Cases
 */
static Pmic_Ut_Tests_t pmic_stress_tests[] =
{
    {
        8230,
        "Pmic_rtcEnableTimer_Alarm_Intr : Stress test for RTC alarm and timer interrupt"
    },
    {
        8239,
        "test_pmic_rtc_testWakeup_TimerIntr_lpStandbyState : Stress test for pmic sleep and wakeup"
    },
    {
        8231,
        "test_pmic_appInit_appDeInit : Stress test for pmic init/deinit with single i2c"
    },
    {
        8235,
        "test_pmic_appInit_appDeInit : Stress test for pmic init/deinit with dual i2c"
    },
    {
        8232,
        "test_pmic_appInit_appDeInit : Stress test for pmic init/deinit with gpio cfg with single i2c'"
    },
    {
        8237,
        "test_pmic_appInit_appDeInit : Stress test for pmic init/deinit with gpio cfg dual i2c"
    }
};

/*!
 * \brief   configure gpio pin as I2C2 SCLK function
 */
static void test_pmic_gpio_setCfgGpioPin_i2c2_sclk(void)
{
    int32_t pmicStatus       = PMIC_ST_SUCCESS;
    int pin                  = 1U;
    Pmic_GpioCfg_t gpioCfg_rd = {PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,};
    Pmic_GpioCfg_t gpioCfg   =
    {
        PMIC_GPIO_CFG_PINFUNC_VALID_SHIFT,
        PMIC_GPIO_OUTPUT,
        PMIC_GPIO_OPEN_DRAIN_OUTPUT,
        PMIC_GPIO_PULL_DOWN,
        PMIC_GPIO_DEGLITCH_ENABLE,
        PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI,
        PMIC_GPIO_HIGH
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 1U;
        gpioCfg.pinFunc = PMIC_TPS6594X_GPIO_PINFUNC_GPIO1_SCL_I2C2_CS_SPI;
    }
    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pin = 2U;
        gpioCfg.pinFunc = PMIC_LP8764X_GPIO_PINFUNC_GPIO2_SCL_I2C2;
    }
    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((1U == pin) || (2U == pin)))
        {
            TEST_IGNORE();
        }
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        /* Should not break SPI Comm Lines */
        if((PMIC_INTF_SPI == pPmicCoreHandle->commMode) &&
           ((2U == pin) || (3U == pin)))
        {
            TEST_IGNORE();
        }
    }

    pmicStatus = Pmic_gpioSetConfiguration(pPmicCoreHandle, pin, gpioCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_gpioGetConfiguration(pPmicCoreHandle, pin, &gpioCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    TEST_ASSERT_EQUAL(gpioCfg.pinFunc, gpioCfg_rd.pinFunc);

}

/*!
 * \brief   Pmic_powerSetPwrResourceCfg : Test Enable buck pull down checking for Buck regulator
 */
static void test_pmic_powerSetPowerResourceConfig_buckPullDownEn_enable(void)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    Pmic_PowerResourceCfg_t powerCfg_rd =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };
    uint16_t pwrRsrc, pwrRsrcMin, pwrRsrcMax;

    Pmic_PowerResourceCfg_t pPowerCfg   =
    {
        PMIC_CFG_REGULATOR_BUCK_PLDN_EN_VALID_SHIFT,
    };

    if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_TPS6594X_REGULATOR_BUCK_PLDN_ENABLE;
        pwrRsrcMin = PMIC_TPS6594X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_TPS6594X_REGULATOR_BUCK5;
    }

    if(PMIC_DEV_HERA_LP8764X == pPmicCoreHandle->pmicDeviceType)
    {
        pPowerCfg.buckPullDownEn = PMIC_LP8764X_REGULATOR_BUCK_PLDN_ENABLE;
        pwrRsrcMin = PMIC_LP8764X_REGULATOR_BUCK1;
        pwrRsrcMax = PMIC_LP8764X_REGULATOR_BUCK4;
    }

    for(pwrRsrc = pwrRsrcMin; pwrRsrc <= pwrRsrcMax ; pwrRsrc++)
    {
        pmicStatus = Pmic_powerSetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 pPowerCfg);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        pmicStatus = Pmic_powerGetPwrResourceCfg(pPmicCoreHandle,
                                                 pwrRsrc,
                                                 &powerCfg_rd);
        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

        TEST_ASSERT_EQUAL(pPowerCfg.buckPullDownEn, powerCfg_rd.buckPullDownEn);
    }

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
 * \brief   Test to configure a wdg for all params
 */
static void test_pmic_wdg_setCfg_forallparams(void)
{
    int32_t pmicStatus      = PMIC_ST_SUCCESS;
    Pmic_WdgCfg_t wdgCfg_rd = {PMIC_WDG_CFG_SETPARAMS_FORALL, };
    Pmic_WdgCfg_t wdgCfg    =
    {
        PMIC_WDG_CFG_SETPARAMS_FORALL,
        750000U,
        4950U,
        4950U,
        PMIC_WDG_FAIL_THRESHOLD_COUNT_5,
        PMIC_WDG_RESET_THRESHOLD_COUNT_6,
        PMIC_WDG_QA_MODE,
        PMIC_WDG_PWRHOLD_DISABLE,
        PMIC_WDG_RESET_ENABLE,
        PMIC_WDG_RETLONGWIN_ENABLE,
        PMIC_WDG_QA_FEEDBACK_VALUE_2,
        PMIC_WDG_QA_LFSR_VALUE_1,
        PMIC_WDG_QA_QUES_SEED_VALUE_5,
    };

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        wdgCfg.longWinDuration_ms = 752000U;
    }

    /* Enable WDG Timer */
    pmicStatus = Pmic_wdgEnable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgSetCfg(pPmicCoreHandle, wdgCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

    pmicStatus = Pmic_wdgGetCfg(pPmicCoreHandle, &wdgCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);
    TEST_ASSERT_EQUAL(wdgCfg.longWinDuration_ms, wdgCfg_rd.longWinDuration_ms);
    TEST_ASSERT_EQUAL(wdgCfg.win1Duration_us, wdgCfg_rd.win1Duration_us);
    TEST_ASSERT_EQUAL(wdgCfg.win2Duration_us, wdgCfg_rd.win2Duration_us);
    TEST_ASSERT_EQUAL(wdgCfg.failThreshold, wdgCfg_rd.failThreshold);
    TEST_ASSERT_EQUAL(wdgCfg.rstThreshold, wdgCfg_rd.rstThreshold);
    TEST_ASSERT_EQUAL(wdgCfg.wdgMode, wdgCfg_rd.wdgMode);
    TEST_ASSERT_EQUAL(wdgCfg.pwrHold, wdgCfg_rd.pwrHold);
    TEST_ASSERT_EQUAL(wdgCfg.rstEnable, wdgCfg_rd.rstEnable);
    TEST_ASSERT_EQUAL(wdgCfg.retLongWin, wdgCfg_rd.retLongWin);
    TEST_ASSERT_EQUAL(wdgCfg.qaFdbk, wdgCfg_rd.qaFdbk);
    TEST_ASSERT_EQUAL(wdgCfg.qaLfsr, wdgCfg_rd.qaLfsr);
    TEST_ASSERT_EQUAL(wdgCfg.qaQuesSeed, wdgCfg_rd.qaQuesSeed);

    /* Disable WDG Timer */
    pmicStatus = Pmic_wdgDisable(pPmicCoreHandle);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, pmicStatus);

}

/*!
 * \brief   Stress test for RTC alarm and timer interrupt
 */
static void test_pmic_rtc_stressTestTimerAsyncIntr(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    uint8_t            timerPeriod  = 0U;
    Pmic_IrqStatus_t errStat        = {0U};
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t    validDateCfg =  { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 30U, 30U, 6U, 0U, 1U};
    uint32_t durationInHrs          = 24U;
    uint32_t pmic_stress_tst_alarm_count = 0U;
    uint32_t pmic_stress_tst_timer_count = 0U;

    test_pmic_print_unity_testcase_info(8230,
                                        pmic_stress_tests,
                                        PMIC_STRESS_NUM_OF_TESTCASES);

#if ((defined(SOC_J7200) && (defined(BUILD_MCU2_0) || defined(BUILD_MCU2_1))) || \
     (defined(SOC_J721E) && (defined(BUILD_MCU3_0) || defined(BUILD_MCU3_1))))
    /*Refer the Bug-PDK-10394 for more details*/
    pmic_testResultUpdate_ignore(8230,
                                 pmic_stress_tests,
                                 PMIC_STRESS_NUM_OF_TESTCASES);
#endif

    /* Enable GPIO interrupt on the specific gpio pin */
    GPIO_enableInt(0);

    pHandle             = pPmicCoreHandle;

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Get the current time value */
    status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle,
                                           PMIC_RTC_SECOND_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    timeCfg_rd.hour = timeCfg_rd.hour + 1U;
    status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    while((pmic_intr_triggered_stress_tst_alarm - 1U) != durationInHrs)
    {
    }

    pmic_stress_tst_alarm_count = durationInHrs;
    pmic_stress_tst_timer_count = (durationInHrs * 3600U);

    TEST_ASSERT_EQUAL(pmic_stress_tst_alarm_count,
                     (pmic_intr_triggered_stress_tst_alarm - 1U));

    TEST_ASSERT_EQUAL(pmic_stress_tst_timer_count,
                     (pmic_intr_triggered_stress_tst_timer - 1U));

    pmic_log("Total count of Alarm interrupts triggered %d ",
              (pmic_intr_triggered_stress_tst_alarm - 1U));
    pmic_log("Total count of Timer interrupts triggered %d ",
              (pmic_intr_triggered_stress_tst_timer - 1U));

    /* Disable the timer interrupt  */
    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable the alarm interrupt  */
    status = Pmic_rtcEnableAlarmIntr(pHandle, PMIC_RTC_ALARM_INTR_DISABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle, timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Disable GPIO interrupt on the specific gpio pin */
    GPIO_disableInt(0);

    pmic_testResultUpdate_pass(8230,
                               pmic_stress_tests,
                               PMIC_STRESS_NUM_OF_TESTCASES);
}

/*!
 * \brief   PMIC Application Callback Function
 */
void AppPmicCallbackFxn(void)
{
    int32_t status           = PMIC_ST_SUCCESS;
    Pmic_IrqStatus_t errStat = {0U};
    uint8_t irqNum           = 0U;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    Pmic_RtcTime_t     timeCfg_rd   = { 0x1F, 0U, 0U, 0U, 0U, 0U};
    Pmic_RtcDate_t     dateCfg_rd   = { 0x0F, 0U, 0U, 0U, 0U};

    pHandle             = pPmicCoreHandle;

    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, false);

    if(PMIC_ST_SUCCESS == status)
    {
        while(PMIC_ST_ERR_INV_INT != status)
        {
            status = Pmic_getNextErrorStatus(pPmicCoreHandle,
                                             &errStat,
                                             &irqNum);
            if(PMIC_ST_SUCCESS == status)
            {
                switch(irqNum)
                {
                    case PMIC_TPS6594X_RTC_TIMER_INT:
                        if(pmic_intr_triggered_stress_tst_timer % 4U == 1U)
                        {
                             test_pmic_gpio_setCfgGpioPin_i2c2_sclk();
                        }

                        if(pmic_intr_triggered_stress_tst_timer % 4U == 2U)
                        {
                            test_pmic_powerSetPowerResourceConfig_buckPullDownEn_enable();
                        }

                        if(pmic_intr_triggered_stress_tst_timer % 4U == 3U)
                        {
                            test_pmic_esm_setConfiguration_esmMcuLevelMode();
                        }

                        if(pmic_intr_triggered_stress_tst_timer % 4U == 0U)
                        {
                            test_pmic_wdg_setCfg_forallparams();
                        }

                        pmic_intr_triggered_stress_tst_timer++;

                        /* clear the interrupt */
                        status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_RTC_TIMER_INT);

                        break;
                    case PMIC_TPS6594X_RTC_ALARM_INT:

                        pmic_intr_triggered_stress_tst_alarm++;

                        /* clear the interrupt */
                        status = Pmic_irqClrErrStatus(pPmicCoreHandle,
                                                  PMIC_TPS6594X_RTC_ALARM_INT);

                        /* Get the current time value */
                        status = Pmic_rtcGetTimeDateInfo(pHandle, &timeCfg_rd, &dateCfg_rd);
                        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                        if(23U == timeCfg_rd.hour)
                        {
                            timeCfg_rd.hour = 0U;
                            dateCfg_rd.day = dateCfg_rd.day + 1U;
                        }
                        else
                        {
                            timeCfg_rd.hour = timeCfg_rd.hour + 1U;
                        }

                        status   = Pmic_rtcSetAlarmInfo(pHandle, timeCfg_rd, dateCfg_rd);
                        TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

                       break;
                    default:
                       break;
                }
            }
        }
    }
}

/*!
 * \brief   STRESS Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_stress_testApp(void)
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
 * \brief   STRESS Unity Test App wrapper Function for Single I2C LEO PMIC-A
 */
static int32_t test_pmic_leo_singleI2C_pmicA_stress_testApp(void)
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
 * \brief   STRESS Unity Test App wrapper Function for HERA PMIC
 */
static int32_t test_pmic_hera_stress_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_HERA_LP8764X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr          = J7VCL_HERA_PMIC_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = J7VCL_HERA_PMIC_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J7VCL_HERA_PMIC_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead   = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite  = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop  = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;

}

/*!
 * \brief   STRESS Unity Test App wrapper Function for LEO PMIC-B
 */
static int32_t test_pmic_leo_pmicB_stress_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_CoreCfg_t pmicConfigData = {0U};

    /* Fill parameters to pmicConfigData */
    pmicConfigData.pmicDeviceType     = PMIC_DEV_LEO_TPS6594X;
    pmicConfigData.validParams        |= PMIC_CFG_DEVICE_TYPE_VALID_SHIFT;

    pmicConfigData.commMode           = PMIC_INTF_SINGLE_I2C;
    pmicConfigData.validParams        |= PMIC_CFG_COMM_MODE_VALID_SHIFT;

    pmicConfigData.i2c1Speed            = PMIC_I2C_STANDARD_MODE;
    pmicConfigData.validParams         |= PMIC_CFG_I2C1_SPEED_VALID_SHIFT;

    pmicConfigData.slaveAddr          = J721E_LEO_PMICB_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_SLAVEADDR_VALID_SHIFT;

    pmicConfigData.qaSlaveAddr        = J721E_LEO_PMICB_WDG_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_QASLAVEADDR_VALID_SHIFT;

    pmicConfigData.nvmSlaveAddr        = J721E_LEO_PMICB_PAGE1_SLAVE_ADDR;
    pmicConfigData.validParams        |= PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoRead   = test_pmic_regRead;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_RD_VALID_SHIFT;

    pmicConfigData.pFnPmicCommIoWrite  = test_pmic_regWrite;
    pmicConfigData.validParams         |= PMIC_CFG_COMM_IO_WR_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStart = test_pmic_criticalSectionStartFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_START_VALID_SHIFT;

    pmicConfigData.pFnPmicCritSecStop  = test_pmic_criticalSectionStopFn;
    pmicConfigData.validParams         |= PMIC_CFG_CRITSEC_STOP_VALID_SHIFT;

    status = test_pmic_appInit(&pPmicCoreHandle, &pmicConfigData);
    return status;
}

/*!
 * \brief   STRESS Unity Test App wrapper Function for LEO PMIC-A
 */
static int32_t test_pmic_leo_pmicA_spiStub_stress_testApp(void)
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

static void test_pmic_singleI2C_init_deinit_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    int8_t  count  = 0;
    int8_t  loopcount = 100;

    test_pmic_print_unity_testcase_info(8231,
                                        pmic_stress_tests,
                                        PMIC_STRESS_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {

        if(pPmicCoreHandle != NULL)
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        for (count = 0;count < loopcount;count++)
        {
            status = test_pmic_leo_singleI2C_pmicA_stress_testApp();

            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(8231,
                               pmic_stress_tests,
                               PMIC_STRESS_NUM_OF_TESTCASES);
}

static void test_pmic_dualI2C_init_deinit_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    int8_t  count  = 0;
    int8_t  loopcount = 100;

    test_pmic_print_unity_testcase_info(8235,
                                        pmic_stress_tests,
                                        PMIC_STRESS_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {

        if(pPmicCoreHandle != NULL)
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        for (count = 0;count < loopcount;count++)
        {
            status = test_pmic_leo_pmicA_stress_testApp();

            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(8235,
                               pmic_stress_tests,
                               PMIC_STRESS_NUM_OF_TESTCASES);
}

static void test_pmic_singleI2C_init_gpio_deinit_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    int8_t  count  = 0;
    int8_t  loopcount = 100;

    test_pmic_print_unity_testcase_info(8232,
                                        pmic_stress_tests,
                                        PMIC_STRESS_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {

        if(pPmicCoreHandle != NULL)
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        for (count = 0;count < loopcount;count++)
        {
            status = test_pmic_leo_singleI2C_pmicA_stress_testApp();

           if(PMIC_ST_SUCCESS == status)
           {
                test_pmic_gpio_setCfgGpioPin_i2c2_sclk();
           }

            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(8232,
                               pmic_stress_tests,
                               PMIC_STRESS_NUM_OF_TESTCASES);
}

static void test_pmic_dualI2C_init_gpio_deinit_testApp(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    int8_t  count  = 0;
    int8_t  loopcount = 100;

    test_pmic_print_unity_testcase_info(8237,
                                        pmic_stress_tests,
                                        PMIC_STRESS_NUM_OF_TESTCASES);

    if(J721E_LEO_PMICA_DEVICE == pmic_device_info)
    {

        if(pPmicCoreHandle != NULL)
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        for (count = 0;count < loopcount;count++)
        {
            status = test_pmic_leo_pmicA_stress_testApp();

            if(PMIC_ST_SUCCESS == status)
            {
                test_pmic_gpio_setCfgGpioPin_i2c2_sclk();
            }

            if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
            {
                test_pmic_appDeInit(pPmicCoreHandle);
            }
        }
    }

    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    pmic_testResultUpdate_pass(8237,
                               pmic_stress_tests,
                               PMIC_STRESS_NUM_OF_TESTCASES);
}

/*!
 * \brief   RTC wakeup using time interrupt for LP Standby State
 */
static void test_pmic_rtc_testWakeup_TimerIntr_lpStandbyState(void)
{
    int32_t            status       = PMIC_ST_SUCCESS;
    Pmic_CoreHandle_t  *pHandle     = NULL;
    uint8_t            timerPeriod  = 0U;
    int8_t num                      = 0;
    Pmic_RtcDate_t    validDateCfg  = { 0x0F, 15U, 6U, 2055U, 1U};
    Pmic_RtcTime_t    validTimeCfg  = { 0x1F, 1U, 30U, 6U, 0U, 1U};
    Pmic_IrqStatus_t errStat        = {0U};

    test_pmic_print_unity_testcase_info(8239,
                                        pmic_stress_tests,
                                        PMIC_STRESS_NUM_OF_TESTCASES);

    if(PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev)
    {
        /*Refer the Bug PDK-10450 for more details*/
        pmic_testResultUpdate_ignore(8239,
                                     pmic_stress_tests,
                                     PMIC_STRESS_NUM_OF_TESTCASES);
    }
    pHandle                         = pPmicCoreHandle;

#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be High");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be High");
#endif
    pmic_log("\r\n Enter 1 to continue");
    UART_scanFmt("%d", &num);

    status = Pmic_rtcGetTimerPeriod(pHandle, &timerPeriod);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimerPeriod(pHandle, PMIC_RTC_MINUTE_INTR_PERIOD);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_rtcSetTimeDateInfo(pHandle, validTimeCfg, validDateCfg);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* To clear the interrupts*/
    status = Pmic_irqGetErrStatus(pPmicCoreHandle, &errStat, true);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
    {
        int i = 0;
        for(i=0;i<4; i++)
        {
            pmic_log("\r\nINT STAT[%d]: 0x%08x", i, errStat.intStatus[i]);
        }
    }
#if defined(SOC_J721E)
    pmic_log("\r\n Probe TP134 and TP133 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP134 and TP133 and it should be High after 60 sec");
#endif
#if defined(SOC_J7200)
    pmic_log("\r\n Probe TP46 and TP29 and it should be low after 2 sec");

    pmic_log("\r\n Probe TP46 and TP29 and it should be High after 60 sec");
#endif

    pmic_log("\r\n After 60sec Rerun the application in UART Boot mode");

    pmic_log("\r\n Also check for RTC Timer interrupt in Interrupt status register");

    status = Pmic_rtcEnableTimerIntr(pHandle, PMIC_RTC_TIMER_INTR_ENABLE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP1_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    status = Pmic_fsmSetNsleepSignalMask(pHandle,
                                         PMIC_NSLEEP2_SIGNAL,
                                         PMIC_NSLEEPX_MASK);
     TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);

    /* Needs Delay to mask Nsleep1B and Nsleep2B signals for LP Stand-By State */
    Osal_delay(10U);

    status =  Pmic_fsmSetMissionState(pPmicCoreHandle, PMIC_FSM_LP_STANBY_STATE);
    TEST_ASSERT_EQUAL(PMIC_ST_SUCCESS, status);
}


#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E) || defined(SOC_J7200))

/*!
 * \brief   Run stress unity test cases
 */
static void test_pmic_run_testcases(void)
{
    pmic_log("\n\n%s(): %d: Begin Unity Test Cases...\n", __func__, __LINE__);
    UNITY_BEGIN();

    pmic_testResult_init(pmic_stress_tests, PMIC_STRESS_NUM_OF_TESTCASES);

    RUN_TEST(test_pmic_singleI2C_init_deinit_testApp);
    RUN_TEST(test_pmic_dualI2C_init_deinit_testApp);
    RUN_TEST(test_pmic_singleI2C_init_gpio_deinit_testApp);
    RUN_TEST(test_pmic_dualI2C_init_gpio_deinit_testApp);

    pmic_updateTestResults(pmic_stress_tests, PMIC_STRESS_NUM_OF_TESTCASES);

    UNITY_END();
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
        status = test_pmic_leo_pmicA_stress_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J721E_LEO_PMICB_DEVICE;
            status = test_pmic_leo_pmicB_stress_testApp();
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
        status = test_pmic_leo_pmicA_stress_testApp();
        /* Deinit pmic handle */
        if((pPmicCoreHandle != NULL) && (PMIC_ST_SUCCESS == status))
        {
            test_pmic_appDeInit(pPmicCoreHandle);
        }

        if(PMIC_ST_SUCCESS == status)
        {
            pmic_device_info = J7VCL_HERA_PMICB_DEVICE;
            status = test_pmic_hera_stress_testApp();
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
    " \r\n 0: Automatic run for all board specific STRESS options"
    " \r\n 1: Manual run for STRESS options"
    " \r\n 2: quit"
    " \r\n"
    " \r\n Enter option: "
};

volatile static const char pmicTestAppMenu[] =
{
    " \r\n ================================================================="
    " \r\n Menu Options:"
    " \r\n ================================================================="
    " \r\n 0: Pmic Leo device(PMIC A on J721E EVM Using I2C Interface)"
    " \r\n 1: Pmic Leo device(PMIC A on J7VCL EVM Using I2C Interface)"
    " \r\n 2: Pmic Leo device with SPI Stub Functions(PMIC-A on J721E EVM)"
    " \r\n 3: Pmic Leo device with SPI Stub Functions(PMIC-A on J7VCL EVM)"
    " \r\n 4: Pmic Leo device(PMIC A on J721E EVM Manual Stress Testcases)"
    " \r\n 5: Pmic Leo device(PMIC A on J7VCL EVM Manual Stress Testcases)"
    " \r\n 6: Back to Test Menu"
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
    pmic_log(" \r\n 0: Pmic Leo device(PMIC A on %s EVM for RTC WKUP using Timer Interrupt from LP Standby State)", board_name);
    pmic_log(" \r\n 1: Pmic Leo device(PMIC A on %s EVM for RTC Alarm and Timer Asynchronous Interrupts for 24 hours)", board_name);
    pmic_log(" \r\n 2: Back to Main Menu");
    pmic_log(" \r\n");
    pmic_log(" \r\n Enter option: ");
}

/*!
 * \brief   Run STRESS manual test cases
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
                RUN_TEST(test_pmic_rtc_testWakeup_TimerIntr_lpStandbyState);
               break;
            case 1U:
                RUN_TEST(test_pmic_rtc_stressTestTimerAsyncIntr);
                break;
            case 2U:
                pmic_log(" \r\n Back to Test Menu options\n");
                return;
            default:
               pmic_log(" \r\n Invalid option... Try Again!!!\n");
               break;
        }
    }
}

static void test_pmic_stress_testapp_run_options(int8_t option)
{
    int8_t num = -1;
    int8_t idx = 0;
#if defined(SOC_J721E)
    int8_t automatic_options[] = {0, 2};
#elif defined(SOC_J7200)
    int8_t automatic_options[] = {1, 3};
#endif

    while(1U)
    {
        if(idx >= (sizeof(automatic_options)/sizeof(automatic_options[0])))
        {
            pmic_printTestResult(pmic_stress_tests, PMIC_STRESS_NUM_OF_TESTCASES);
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
                num = 6;
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

                    /* STRESS Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_stress_testApp())
                    {
                        if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }
                        /* Run stress test cases for Leo PMIC-A */
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
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* STRESS Unity Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_stress_testApp())
                    {
                        /* Run stress test cases for Leo PMIC-A */
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
            case 2U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* STRESS Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                    if(PMIC_ST_SUCCESS ==
                           test_pmic_leo_pmicA_spiStub_stress_testApp())
                    {
                       if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }
                        /* Run stress test cases for Leo PMIC-A */
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
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;
                    /* STRESS Unity Test App wrapper Function for LEO PMIC-A
                     * using SPI stub functions */
                     if(PMIC_ST_SUCCESS ==
                            test_pmic_leo_pmicA_spiStub_stress_testApp())
                    {
                        /* Run stress test cases for Leo PMIC-A */
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
            case 4U:
#if defined(SOC_J721E)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J721E_BOARD))
                {
                    pmic_device_info = J721E_LEO_PMICA_DEVICE;

                    /* STRESS Manual Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_stress_testApp())
                    {
                        if(PMIC_SILICON_REV_ID_PG_2_0 ==
                          pPmicCoreHandle->pmicDevSiliconRev)
                       {
                            test_pmic_rtc_setCfg_xtalOScEnType(pPmicCoreHandle);
                       }
                        /* Run Stress manual test cases */
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
           case 5U:
#if defined(SOC_J7200)
                if(PMIC_ST_SUCCESS == setup_pmic_interrupt(J7VCL_BOARD))
                {
                    pmic_device_info = J7VCL_LEO_PMICA_DEVICE;

                    /* STRESS Manual Test App wrapper Function for LEO PMIC-A */
                    if(PMIC_ST_SUCCESS == test_pmic_leo_pmicA_stress_testApp())
                    {
                        /* Run Stress manual test cases */
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
            case 6U:
                pmic_log(" \r\n Back to Test Menu options\n");
                return;
            default:
                pmic_log(" \r\n Invalid option... Try Again!!!\n");
                break;
        }
    }
}

/*!
 * \brief   Function to register STRESS Unity Test App wrapper to Unity framework
 */
static void test_pmic_stress_testapp_runner(void)
{
    /* @description : Test runner for STRESS Test App
     *
     * @requirements: XXXX
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
                test_pmic_stress_testapp_run_options(PMIC_UT_AUTOMATE_OPTION);
               break;
            case PMIC_UT_MANUAL_OPTION:
                test_pmic_stress_testapp_run_options(PMIC_UT_MANUAL_OPTION);
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
 * \brief   TI RTOS specific STRESS TEST APP main Function
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values see \ref Pmic_ErrorCodes
 */
int main()
{
    Board_initUART();

    /*
     * Initialze and Register ISR handler to J7 Interrupts
     */
    App_initGPIO(AppPmicCallbackFxn);

    pmic_print_banner("PMIC STRESS Unity Test Application");
#if defined(UNITY_INCLUDE_CONFIG_V2_H) && \
    (defined(SOC_J721E)             || \
     defined(SOC_J7200))
    test_pmic_stress_testapp_runner();
#endif
}
