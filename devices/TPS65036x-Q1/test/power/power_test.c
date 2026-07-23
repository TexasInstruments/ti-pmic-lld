/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
 * @file power_test.c
 * @brief Source file containing definitions to PMIC Power tests.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "power_test.h"

#ifdef BUILD_MOCK
#include "pmic_mock_core.h"

#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* Test macros moved to power_test.h */

/* ========================================================================== */
/*                             Global Variables                               */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0U};

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

#ifdef BUILD_MOCK
extern PmicMockDevice_t* platform_getMockDevice(void);
#endif

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

void power_test(void *args)
{
    char msg[50U] = {0};
    int32_t status = PMIC_ST_SUCCESS;

    platform_init();

    Pmic_HandleCfg_t pmicCfg = {
        .validParams = (PMIC_I2C_ADDR0_VALID |
                        PMIC_COMM_HANDLE_0_VALID |
                        PMIC_IO_READ_VALID |
                        PMIC_IO_WRITE_VALID |
                        PMIC_CRITICAL_SECTION_START_VALID |
                        PMIC_CRITICAL_SECTION_STOP_VALID |
                        PMIC_IRQ_RESPONSE_CALLBACK_VALID |
                        PMIC_TIMER_WAIT_MS_VALID),
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop,
        .irqResponseCallback = &platform_irqResponse,
        .timerWaitMs = &testUtils_timerWaitMs
    };

    testTimer_startModule("Power");

    platform_printString("\r\n");
    platform_printString("POWER_TEST\r\n");
    platform_printString("----------\r\n\r\n");

    status = Pmic_init(&pmicHandle, &pmicCfg);

    if (status == PMIC_ST_SUCCESS)
    {
        testUtils_printSiRev(&pmicHandle);

    
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_irqClrAllFlags(&pmicHandle);

            if (status == PMIC_ST_SUCCESS)
            {
                platform_setupTests();
                POWER_TEST_RUN_ALL();
                platform_tearDownTests();
            }
            else
            {
                (void)sprintf(msg, "Error in clearing all PMIC IRQs: %d\r\n", status);
                platform_printString(msg);
            }
        }
        else
        {
            (void)sprintf(msg, "Error in unlocking PMIC registers: %d\r\n", status);
            platform_printString(msg);
        }
    }
    else
    {
        (void)sprintf(msg, "Error in initializing PMIC LLD: %d\r\n", status);
        platform_printString(msg);
    }

    testTimer_endModule();

    (void)Pmic_deinit(&pmicHandle);
    platform_deinit();
}

/* ========================================================================== */
/*                        LDO Configuration Tests                             */
/* ========================================================================== */

/* ========================================================================== */
/*                    LDO Configuration Negative Tests                        */
/* ========================================================================== */

void test_neg_power_pwrSetLdoCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_ENABLE_VALID,
        .enable = PMIC_ENABLE
    };
    int32_t status = Pmic_pwrSetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetLdoCfg_nullParam_ldoCfg(void)
{
    // Pass NULL ldoCfg into Pmic_pwrSetLdoCfg()
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_mode(void)
{
    // Pass out of bounds mode into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_MODE_VALID,
        .mode = PMIC_LDO_BYP_CONFIG_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_vset(void)
{
    // Pass out of bounds vset into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_VSET_VALID,
        .vset = PMIC_LDO_VSET_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_dischargeSel(void)
{
    // Pass out of bounds dischargeSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_DISCHARGE_SEL_VALID,
        .dischargeSel = PMIC_LDO_DISCHARGE_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_deglitchSel(void)
{
    // Pass out of bounds deglitchSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_DEGLITCH_SEL_VALID,
        .deglitchSel = PMIC_LDO_DEGLITCH_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_uvThr(void)
{
    // Pass out of bounds uvThr into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_UV_THR_VALID,
        .uvThr = PMIC_LDO_UV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_ovThr(void)
{
    // Pass out of bounds ovThr into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_OV_THR_VALID,
        .ovThr = PMIC_LDO_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_ilimSel(void)
{
    // Pass out of bounds ilimSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_ILIM_SEL_VALID,
        .ilimSel = PMIC_LDO_ILIM_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_ovpSel(void)
{
    // Pass out of bounds ovpSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_OVP_SEL_VALID,
        .ovpSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_ovSel(void)
{
    // Pass out of bounds ovSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_OV_SEL_VALID,
        .ovSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_uvSel(void)
{
    // Pass out of bounds uvSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_UV_SEL_VALID,
        .uvSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_scSel(void)
{
    // Pass out of bounds scSel into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_SC_SEL_VALID,
        .scSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetLdoCfg_outOfBounds_rvConf(void)
{
    // Pass out of bounds rvConf into Pmic_pwrSetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = PMIC_LDO_RV_CONF_VALID,
        .rvConf = PMIC_LDO_RV_CONF_MAX + 1U
    };
    int32_t status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetLdoCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrGetLdoCfg()
    Pmic_PwrLdoCfg_t ldoCfg = {0};
    int32_t status = Pmic_pwrGetLdoCfg(NULL, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetLdoCfg_nullParam_ldoCfg(void)
{
    // Pass NULL ldoCfg into Pmic_pwrGetLdoCfg()
    int32_t status = Pmic_pwrGetLdoCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                    LDO Configuration Positive Tests                        */
/* ========================================================================== */

void test_pos_power_ldoSetGetCfg_enable(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_ENABLE_VALID,
        .enable = PMIC_ENABLE
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_ENABLE_VALID
    };

    // Set LDO enable
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO enable
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.enable == ldoCfgSet.enable);

    // Disable LDO
    ldoCfgSet.enable = PMIC_DISABLE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify LDO is disabled
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.enable == PMIC_DISABLE);
}

void test_pos_power_ldoSetGetCfg_mode(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_MODE_VALID,
        .mode = PMIC_LDO_MODE
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_MODE_VALID
    };

    // Set LDO mode
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO mode
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.mode == ldoCfgSet.mode);

    // Set bypass mode
    ldoCfgSet.mode = PMIC_BYPASS_MODE;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify bypass mode
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.mode == PMIC_BYPASS_MODE);
}

void test_pos_power_ldoSetGetCfg_vset(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_VSET_VALID,
        .vset = PMIC_LDO_VSET_MIN
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_VSET_VALID
    };

    // Set LDO vset to minimum
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO vset
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vset == ldoCfgSet.vset);

    // Set LDO vset to maximum
    ldoCfgSet.vset = PMIC_LDO_VSET_MAX;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify vset maximum
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vset == PMIC_LDO_VSET_MAX);
}

void test_pos_power_ldoSetGetCfg_vmonOnly(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_VMON_ONLY_VALID,
        .vmonOnly = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_VMON_ONLY_VALID
    };

    // Set LDO vmonOnly
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO vmonOnly
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vmonOnly == ldoCfgSet.vmonOnly);

    // Disable vmonOnly
    ldoCfgSet.vmonOnly = false;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify vmonOnly is disabled
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.vmonOnly == false);
}

void test_pos_power_ldoSetGetCfg_dischargeEn(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_DISCHARGE_EN_VALID,
        .dischargeEn = true
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_DISCHARGE_EN_VALID
    };

    // Set LDO discharge enable
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO discharge enable
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.dischargeEn == ldoCfgSet.dischargeEn);

    // Disable discharge
    ldoCfgSet.dischargeEn = false;
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify discharge is disabled
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.dischargeEn == false);
}

void test_pos_power_ldoSetGetCfg_dischargeSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_DISCHARGE_SEL_VALID,
        .dischargeSel = PMIC_LDO_DISCHARGE_SEL_50K_OHM
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_DISCHARGE_SEL_VALID
    };

    // Set LDO discharge selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO discharge selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.dischargeSel == ldoCfgSet.dischargeSel);
}

void test_pos_power_ldoSetGetCfg_deglitchSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_DEGLITCH_SEL_VALID,
        .deglitchSel = PMIC_LDO_DEGLITCH_SEL_4_US
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_DEGLITCH_SEL_VALID
    };

    // Set LDO deglitch selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO deglitch selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.deglitchSel == ldoCfgSet.deglitchSel);
}

void test_pos_power_ldoSetGetCfg_uvThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_UV_THR_VALID,
        .uvThr = PMIC_LDO_UV_THR_3P5_PCT
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_UV_THR_VALID
    };

    // Set LDO UV threshold
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO UV threshold
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.uvThr == ldoCfgSet.uvThr);
}

void test_pos_power_ldoSetGetCfg_ovThr(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_OV_THR_VALID,
        .ovThr = PMIC_LDO_OV_THR_4_PCT
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_OV_THR_VALID
    };

    // Set LDO OV threshold
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO OV threshold
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ovThr == ldoCfgSet.ovThr);
}

void test_pos_power_ldoSetGetCfg_ilimSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_ILIM_SEL_VALID,
        .ilimSel = PMIC_LDO_ILIM_SEL_200_MA
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_ILIM_SEL_VALID
    };

    // Set LDO current limit selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO current limit selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ilimSel == ldoCfgSet.ilimSel);
}

void test_pos_power_ldoSetGetCfg_ovpSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_OVP_SEL_VALID,
        .ovpSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_OVP_SEL_VALID
    };

    // Set LDO OVP selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO OVP selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ovpSel == ldoCfgSet.ovpSel);
}

void test_pos_power_ldoSetGetCfg_ovSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_OV_SEL_VALID,
        .ovSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_OV_SEL_VALID
    };

    // Set LDO OV selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO OV selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.ovSel == ldoCfgSet.ovSel);
}

void test_pos_power_ldoSetGetCfg_uvSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_UV_SEL_VALID,
        .uvSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_UV_SEL_VALID
    };

    // Set LDO UV selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO UV selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.uvSel == ldoCfgSet.uvSel);
}

void test_pos_power_ldoSetGetCfg_scSel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_SC_SEL_VALID,
        .scSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_SC_SEL_VALID
    };

    // Set LDO SC selection
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO SC selection
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.scSel == ldoCfgSet.scSel);
}

void test_pos_power_ldoSetGetCfg_rvConf(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrLdoCfg_t ldoCfgSet = {
        .validParams = PMIC_LDO_RV_CONF_VALID,
        .rvConf = PMIC_LDO_RV_DISCHARGE
    };
    Pmic_PwrLdoCfg_t ldoCfgGet = {
        .validParams = PMIC_LDO_RV_CONF_VALID
    };

    // Set LDO RV configuration
    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO RV configuration
    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(ldoCfgGet.rvConf == ldoCfgSet.rvConf);
}

/* ========================================================================== */
/*                    SSM_SEL Coverage Tests                                  */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_ssmSel_buck1(void)
{
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK1;
    setCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;
    setCfg.ssmSel = PMIC_SSM_SEL_MAX;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_PwrBuckCfg_t getCfg = {0};
    getCfg.resource = PMIC_BUCK1;
    getCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssmSel == PMIC_SSM_SEL_MAX);
}

void test_pos_power_buckSetGetCfg_ssmSel_buck2(void)
{
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK2;
    setCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;
    setCfg.ssmSel = PMIC_SSM_SEL_MAX;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_PwrBuckCfg_t getCfg = {0};
    getCfg.resource = PMIC_BUCK2;
    getCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssmSel == PMIC_SSM_SEL_MAX);
}

void test_pos_power_buckSetGetCfg_ssmSel_buck3(void)
{
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK3;
    setCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;
    setCfg.ssmSel = PMIC_SSM_SEL_MAX;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_PwrBuckCfg_t getCfg = {0};
    getCfg.resource = PMIC_BUCK3;
    getCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssmSel == PMIC_SSM_SEL_MAX);
}

void test_pos_power_buckSetGetCfg_ssmSel_allValues(void)
{
    // Test SSM_SEL with all valid values on BUCK1
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK1;
    setCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;

    for (uint8_t val = 0; val <= PMIC_SSM_SEL_MAX; val++)
    {
        setCfg.ssmSel = val;
        int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        Pmic_PwrBuckCfg_t getCfg = {0};
        getCfg.resource = PMIC_BUCK1;
        getCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(getCfg.ssmSel == val);
    }
}

void test_neg_power_buckSetCfg_ssmSel_outOfBounds(void)
{
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_SSM_SEL_VALID;
    buckCfg.ssmSel = PMIC_SSM_SEL_MAX + 1U;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                    UVLO Rising/Falling Coverage Tests (BUCK1)             */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_uvloRising_buck1_allValues(void)
{
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK1;
    setCfg.validParams = PMIC_BUCK_UVLO_RISING_VALID;

    for (uint8_t val = 0; val <= PMIC_BUCK1_UVLO_RISING_MAX; val++)
    {
        setCfg.uvloRising = val;
        int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        Pmic_PwrBuckCfg_t getCfg = {0};
        getCfg.resource = PMIC_BUCK1;
        getCfg.validParams = PMIC_BUCK_UVLO_RISING_VALID;

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(getCfg.uvloRising == val);
    }
}

void test_pos_power_buckSetGetCfg_uvloFalling_buck1_allValues(void)
{
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK1;
    setCfg.validParams = PMIC_BUCK_UVLO_FALLING_VALID;

    for (uint8_t val = 0; val <= PMIC_BUCK1_UVLO_FALLING_MAX; val++)
    {
        setCfg.uvloFalling = val;
        int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        Pmic_PwrBuckCfg_t getCfg = {0};
        getCfg.resource = PMIC_BUCK1;
        getCfg.validParams = PMIC_BUCK_UVLO_FALLING_VALID;

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(getCfg.uvloFalling == val);
    }
}

void test_pos_power_buckSetGetCfg_uvloRisingAndFalling_buck1(void)
{
    Pmic_PwrBuckCfg_t setCfg = {0};
    setCfg.resource = PMIC_BUCK1;
    setCfg.validParams = PMIC_BUCK_UVLO_RISING_VALID | PMIC_BUCK_UVLO_FALLING_VALID;
    setCfg.uvloRising = 0x5;
    setCfg.uvloFalling = 0xA;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_PwrBuckCfg_t getCfg = {0};
    getCfg.resource = PMIC_BUCK1;
    getCfg.validParams = PMIC_BUCK_UVLO_RISING_VALID | PMIC_BUCK_UVLO_FALLING_VALID;

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.uvloRising == 0x5);
    PLATFORM_ASSERT(getCfg.uvloFalling == 0xA);
}

/* ========================================================================== */
/*                    Parameter Mismatch Tests                                */
/* ========================================================================== */

void test_neg_power_pwrSetBuckCfg_buck1_vsetActiveNotSupported(void)
{
    // BUCK1 does not support VSET_ACTIVE parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_VSET_ACTIVE_VALID;
    buckCfg.vsetActive = 0x10;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck1_vsetActiveNotSupported(void)
{
    // BUCK1 does not support VSET_ACTIVE parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_VSET_ACTIVE_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckCfg_buck1_vsetLpwrNotSupported(void)
{
    // BUCK1 does not support VSET_LPWR parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_VSET_LPWR_VALID;
    buckCfg.vsetLPwr = 0x10;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck1_vsetLpwrNotSupported(void)
{
    // BUCK1 does not support VSET_LPWR parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_VSET_LPWR_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckCfg_buck2_vsetNotSupported(void)
{
    // BUCK2 does not support non-active VSET parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK2;
    buckCfg.validParams = PMIC_BUCK_VSET_VALID;
    buckCfg.vset = 0x10;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck2_vsetNotSupported(void)
{
    // BUCK2 does not support non-active VSET parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK2;
    buckCfg.validParams = PMIC_BUCK_VSET_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckCfg_buck3_vsetNotSupported(void)
{
    // BUCK3 does not support non-active VSET parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK3;
    buckCfg.validParams = PMIC_BUCK_VSET_VALID;
    buckCfg.vset = 0x10;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck3_vsetNotSupported(void)
{
    // BUCK3 does not support non-active VSET parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK3;
    buckCfg.validParams = PMIC_BUCK_VSET_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckCfg_buck1_vmonOnlyNotSupported(void)
{
    // BUCK1 does not support VMON_ONLY parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_VMON_ONLY_VALID;
    buckCfg.vmonOnly = true;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck1_vmonOnlyNotSupported(void)
{
    // BUCK1 does not support VMON_ONLY parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK1;
    buckCfg.validParams = PMIC_BUCK_VMON_ONLY_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckCfg_buck2_highSideSlewRateNotSupported(void)
{
    // BUCK2 does not support HIGH_SIDE_SLEW_RATE parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK2;
    buckCfg.validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID;
    buckCfg.highSideSlewRate = 0x1;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck2_highSideSlewRateNotSupported(void)
{
    // BUCK2 does not support HIGH_SIDE_SLEW_RATE parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK2;
    buckCfg.validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckCfg_buck3_highSideSlewRateNotSupported(void)
{
    // BUCK3 does not support HIGH_SIDE_SLEW_RATE parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK3;
    buckCfg.validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID;
    buckCfg.highSideSlewRate = 0x1;

    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckCfg_buck3_highSideSlewRateNotSupported(void)
{
    // BUCK3 does not support HIGH_SIDE_SLEW_RATE parameter
    Pmic_PwrBuckCfg_t buckCfg = {0};
    buckCfg.resource = PMIC_BUCK3;
    buckCfg.validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID;

    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}



/* ========================================================================== */
/*                      TSD Configuration Tests                               */
/* ========================================================================== */

/* ========================================================================== */
/*                  TSD Configuration Negative Tests                          */
/* ========================================================================== */

void test_neg_power_pwrSetTsdCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrSetTsdCfg()
    Pmic_PwrTsdCfg_t tsdCfg = {
        .validParams = PMIC_TSD_IMM_LEVEL_VALID,
        .tsdImmLevel = PMIC_TSD_IMM_LEVEL_150C
    };
    int32_t status = Pmic_pwrSetTsdCfg(NULL, &tsdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetTsdCfg_nullParam_tsdCfg(void)
{
    // Pass NULL tsdCfg into Pmic_pwrSetTsdCfg()
    int32_t status = Pmic_pwrSetTsdCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetTsdCfg_outOfBounds_tsdImmLevel(void)
{
    // Pass out of bounds tsdImmLevel into Pmic_pwrSetTsdCfg()
    Pmic_PwrTsdCfg_t tsdCfg = {
        .validParams = PMIC_TSD_IMM_LEVEL_VALID,
        .tsdImmLevel = PMIC_TSD_IMM_LEVEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetTsdCfg_outOfBounds_twarnLevel(void)
{
    // Pass out of bounds twarnLevel into Pmic_pwrSetTsdCfg()
    Pmic_PwrTsdCfg_t tsdCfg = {
        .validParams = PMIC_TWARN_LEVEL_VALID,
        .twarnLevel = PMIC_TWARN_LEVEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetTsdCfg_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrGetTsdCfg()
    Pmic_PwrTsdCfg_t tsdCfg = {0};
    int32_t status = Pmic_pwrGetTsdCfg(NULL, &tsdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetTsdCfg_nullParam_tsdCfg(void)
{
    // Pass NULL tsdCfg into Pmic_pwrGetTsdCfg()
    int32_t status = Pmic_pwrGetTsdCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetTsdImmStatus_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrGetTsdImmStatus()
    bool tsdImmStat = false;
    int32_t status = Pmic_pwrGetTsdImmStatus(NULL, &tsdImmStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetTsdImmStatus_nullParam_tsdImmStat(void)
{
    // Pass NULL tsdImmStat into Pmic_pwrGetTsdImmStatus()
    int32_t status = Pmic_pwrGetTsdImmStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                  TSD Configuration Positive Tests                          */
/* ========================================================================== */

void test_pos_power_tsdSetGetCfg_twarnStayInSafeState(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfgSet = {
        .validParams = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID,
        .twarnStayInSafeState = true
    };
    Pmic_PwrTsdCfg_t tsdCfgGet = {
        .validParams = PMIC_TWARN_STAY_IN_SAFE_STATE_VALID
    };

    // Set TWARN stay in safe state
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get TWARN stay in safe state
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(tsdCfgGet.twarnStayInSafeState == tsdCfgSet.twarnStayInSafeState);

    // Disable TWARN stay in safe state
    tsdCfgSet.twarnStayInSafeState = false;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify TWARN stay in safe state is disabled
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(tsdCfgGet.twarnStayInSafeState == false);
}

void test_pos_power_tsdSetGetCfg_tsdImmLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfgSet = {
        .validParams = PMIC_TSD_IMM_LEVEL_VALID,
        .tsdImmLevel = PMIC_TSD_IMM_LEVEL_150C
    };
    Pmic_PwrTsdCfg_t tsdCfgGet = {
        .validParams = PMIC_TSD_IMM_LEVEL_VALID
    };

    // Set TSD immediate level to 150C
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get TSD immediate level
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(tsdCfgGet.tsdImmLevel == PMIC_TSD_IMM_LEVEL_150C);

    // Set TSD immediate level to 160C
    tsdCfgSet.tsdImmLevel = PMIC_TSD_IMM_LEVEL_160C;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify TSD immediate level is 160C
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(tsdCfgGet.tsdImmLevel == PMIC_TSD_IMM_LEVEL_160C);
}

void test_pos_power_tsdSetGetCfg_twarnLevel(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrTsdCfg_t tsdCfgSet = {
        .validParams = PMIC_TWARN_LEVEL_VALID,
        .twarnLevel = PMIC_TWARN_LEVEL_130C
    };
    Pmic_PwrTsdCfg_t tsdCfgGet = {
        .validParams = PMIC_TWARN_LEVEL_VALID
    };

    // Set TWARN level to 130C
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get TWARN level
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(tsdCfgGet.twarnLevel == PMIC_TWARN_LEVEL_130C);

    // Set TWARN level to 140C
    tsdCfgSet.twarnLevel = PMIC_TWARN_LEVEL_140C;
    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify TWARN level is 140C
    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(tsdCfgGet.twarnLevel == PMIC_TWARN_LEVEL_140C);
}

void test_pos_power_tsdGetImmStatus(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    bool tsdImmStat = false;

    // Get TSD immediate status
    status = Pmic_pwrGetTsdImmStatus(&pmicHandle, &tsdImmStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Note: tsdImmStat value depends on actual die temperature
}

/* ========================================================================== */
/*                     Resource Status Tests                                  */
/* ========================================================================== */

/* ========================================================================== */
/*                Resource Status Negative Tests                              */
/* ========================================================================== */

void test_neg_power_pwrGetRsrcStatus_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrGetRsrcStatus()
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetRsrcStatus(NULL, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_nullParam_pwrRsrcStat(void)
{
    // Pass NULL pwrRsrcStat into Pmic_pwrGetRsrcStatus()
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetRsrcStatus_outOfBounds_resource(void)
{
    // Pass out of bounds resource into Pmic_pwrGetRsrcStatus()
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_POWER_RESOURCE_MAX + 1U
    };
    int32_t status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                Resource Status Positive Tests                              */
/* ========================================================================== */

void test_pos_power_rsrcGetStatus_buck1(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_BUCK1
    };

    // Get BUCK1 status
    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Note: Status values depend on actual BUCK1 state
}

void test_pos_power_rsrcGetStatus_buck2(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_BUCK2
    };

    // Get BUCK2 status
    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Note: Status values depend on actual BUCK2 state
}

void test_pos_power_rsrcGetStatus_buck3(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_BUCK3
    };

    // Get BUCK3 status
    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Note: Status values depend on actual BUCK3 state
}

void test_pos_power_rsrcGetStatus_ldo(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_LDO
    };

    // Get LDO status
    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    // Note: Status values depend on actual LDO state
}


/* ========================================================================== */
/*                         Sequencing Tests                                   */
/* ========================================================================== */

/* ========================================================================== */
/*                    Sequencing Negative Tests                               */
/* ========================================================================== */

void test_neg_power_pwrSetBuckLdoSeqTrig_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrSetBuckLdoSeqTrig()
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[] = {
        {
            .trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT,
            .exclude = false
        }
    };
    int32_t status = Pmic_pwrSetBuckLdoSeqTrig(NULL, seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetBuckLdoSeqTrig_nullParam_seqTrigCfg(void)
{
    // Pass NULL seqTrigCfg into Pmic_pwrSetBuckLdoSeqTrig()
    int32_t status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, NULL, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetBuckLdoSeqTrig_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrGetBuckLdoSeqTrig()
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[1U] = {0};
    int32_t status = Pmic_pwrGetBuckLdoSeqTrig(NULL, seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetBuckLdoSeqTrig_nullParam_seqTrigCfg(void)
{
    // Pass NULL seqTrigCfg into Pmic_pwrGetBuckLdoSeqTrig()
    int32_t status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, NULL, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetBuckLdoSeqDly_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrSetBuckLdoSeqDly()
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK1,
            .seqDlyOn = PMIC_SEQ_DLY_0_MS
        }
    };
    int32_t status = Pmic_pwrSetBuckLdoSeqDly(NULL, seqDlyCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetBuckLdoSeqDly_nullParam_seqDlyCfg(void)
{
    // Pass NULL seqDlyCfg into Pmic_pwrSetBuckLdoSeqDly()
    int32_t status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, NULL, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOn(void)
{
    // Pass out of bounds seqDlyOn into Pmic_pwrSetBuckLdoSeqDly()
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID,
            .resource = PMIC_BUCK1,
            .seqDlyOn = PMIC_SEQ_DLY_MAX + 1U
        }
    };
    int32_t status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrSetBuckLdoSeqDly_outOfBounds_seqDlyOff(void)
{
    // Pass out of bounds seqDlyOff into Pmic_pwrSetBuckLdoSeqDly()
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[] = {
        {
            .validParams = PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1,
            .seqDlyOff = PMIC_SEQ_DLY_MAX + 1U
        }
    };
    int32_t status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

void test_neg_power_pwrGetBuckLdoSeqDly_nullParam_pmicHandle(void)
{
    // Pass NULL pmicHandle into Pmic_pwrGetBuckLdoSeqDly()
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[1U] = {0};
    int32_t status = Pmic_pwrGetBuckLdoSeqDly(NULL, seqDlyCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

void test_neg_power_pwrGetBuckLdoSeqDly_nullParam_seqDlyCfg(void)
{
    // Pass NULL seqDlyCfg into Pmic_pwrGetBuckLdoSeqDly()
    int32_t status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, NULL, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                    Sequencing Positive Tests                               */
/* ========================================================================== */

void test_pos_power_seqTrigSetGet_buck1(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgSet[] = {
        {
            .trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT,
            .exclude = false
        }
    };
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgGet[] = {
        {
            .trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT,
            .exclude = true  // Will be overwritten
        }
    };

    // Set BUCK1 sequence trigger
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 sequence trigger
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqTrigCfgGet[0].exclude == seqTrigCfgSet[0].exclude);

    // Set BUCK1 sequence trigger to exclude
    seqTrigCfgSet[0].exclude = true;
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK1 sequence trigger is excluded
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqTrigCfgGet[0].exclude == true);
}

void test_pos_power_seqTrigSetGet_buck2(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgSet[] = {
        {
            .trigger = PMIC_BUCK2_TRIGGER_PWR_ON_BIT,
            .exclude = false
        }
    };
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgGet[] = {
        {
            .trigger = PMIC_BUCK2_TRIGGER_PWR_ON_BIT,
            .exclude = true
        }
    };

    // Set BUCK2 sequence trigger
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 sequence trigger
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqTrigCfgGet[0].exclude == seqTrigCfgSet[0].exclude);
}

void test_pos_power_seqTrigSetGet_buck3(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgSet[] = {
        {
            .trigger = PMIC_BUCK3_TRIGGER_PWR_ON_BIT,
            .exclude = false
        }
    };
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgGet[] = {
        {
            .trigger = PMIC_BUCK3_TRIGGER_PWR_ON_BIT,
            .exclude = true
        }
    };

    // Set BUCK3 sequence trigger
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 sequence trigger
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqTrigCfgGet[0].exclude == seqTrigCfgSet[0].exclude);
}

void test_pos_power_seqTrigSetGet_ldo(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgSet[] = {
        {
            .trigger = PMIC_LDO_TRIGGER_PWR_ON_BIT,
            .exclude = false
        }
    };
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgGet[] = {
        {
            .trigger = PMIC_LDO_TRIGGER_PWR_ON_BIT,
            .exclude = true
        }
    };

    // Set LDO sequence trigger
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO sequence trigger
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqTrigCfgGet[0].exclude == seqTrigCfgSet[0].exclude);
}

void test_pos_power_seqTrigSetGet_allResources(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgSet[] = {
        { .trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT, .exclude = false },
        { .trigger = PMIC_BUCK2_TRIGGER_PWR_ON_BIT, .exclude = true },
        { .trigger = PMIC_BUCK3_TRIGGER_PWR_ON_BIT, .exclude = false },
        { .trigger = PMIC_LDO_TRIGGER_PWR_ON_BIT, .exclude = true }
    };
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfgGet[] = {
        { .trigger = PMIC_BUCK1_TRIGGER_PWR_ON_BIT, .exclude = true },
        { .trigger = PMIC_BUCK2_TRIGGER_PWR_ON_BIT, .exclude = false },
        { .trigger = PMIC_BUCK3_TRIGGER_PWR_ON_BIT, .exclude = true },
        { .trigger = PMIC_LDO_TRIGGER_PWR_ON_BIT, .exclude = false }
    };

    // Set all sequence triggers
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgSet, 4U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get all sequence triggers
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfgGet, 4U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    
    // Verify all sequence triggers
    for (uint8_t i = 0U; i < 4U; i++)
    {
        PLATFORM_ASSERT(seqTrigCfgGet[i].exclude == seqTrigCfgSet[i].exclude);
    }
}

void test_pos_power_seqDlySetGet_buck1(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgSet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1,
            .seqDlyOn = PMIC_SEQ_DLY_1_MS,
            .seqDlyOff = PMIC_SEQ_DLY_2_MS
        }
    };
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgGet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1
        }
    };

    // Set BUCK1 sequence delays
    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 sequence delays
    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOn == seqDlyCfgSet[0].seqDlyOn);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOff == seqDlyCfgSet[0].seqDlyOff);
}

void test_pos_power_seqDlySetGet_buck2(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgSet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK2,
            .seqDlyOn = PMIC_SEQ_DLY_3_MS,
            .seqDlyOff = PMIC_SEQ_DLY_4_MS
        }
    };
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgGet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK2
        }
    };

    // Set BUCK2 sequence delays
    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 sequence delays
    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOn == seqDlyCfgSet[0].seqDlyOn);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOff == seqDlyCfgSet[0].seqDlyOff);
}

void test_pos_power_seqDlySetGet_buck3(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgSet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK3,
            .seqDlyOn = PMIC_SEQ_DLY_5_MS,
            .seqDlyOff = PMIC_SEQ_DLY_6_MS
        }
    };
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgGet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK3
        }
    };

    // Set BUCK3 sequence delays
    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 sequence delays
    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOn == seqDlyCfgSet[0].seqDlyOn);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOff == seqDlyCfgSet[0].seqDlyOff);
}

void test_pos_power_seqDlySetGet_ldo(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgSet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_LDO,
            .seqDlyOn = PMIC_SEQ_DLY_7_MS,
            .seqDlyOff = PMIC_SEQ_DLY_8_MS
        }
    };
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgGet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_LDO
        }
    };

    // Set LDO sequence delays
    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfgSet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get LDO sequence delays
    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfgGet, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOn == seqDlyCfgSet[0].seqDlyOn);
    PLATFORM_ASSERT(seqDlyCfgGet[0].seqDlyOff == seqDlyCfgSet[0].seqDlyOff);
}

void test_pos_power_seqDlySetGet_allResources(void)
{
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgSet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1,
            .seqDlyOn = PMIC_SEQ_DLY_0_MS,
            .seqDlyOff = PMIC_SEQ_DLY_1_MS
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK2,
            .seqDlyOn = PMIC_SEQ_DLY_2_MS,
            .seqDlyOff = PMIC_SEQ_DLY_3_MS
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK3,
            .seqDlyOn = PMIC_SEQ_DLY_4_MS,
            .seqDlyOff = PMIC_SEQ_DLY_5_MS
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_LDO,
            .seqDlyOn = PMIC_SEQ_DLY_6_MS,
            .seqDlyOff = PMIC_SEQ_DLY_7_MS
        }
    };
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfgGet[] = {
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK1
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK2
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_BUCK3
        },
        {
            .validParams = PMIC_SEQ_DLY_ON_VALID | PMIC_SEQ_DLY_OFF_VALID,
            .resource = PMIC_LDO
        }
    };

    // Set all sequence delays
    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfgSet, 4U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get all sequence delays
    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfgGet, 4U);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    
    // Verify all sequence delays
    for (uint8_t i = 0U; i < 4U; i++)
    {
        PLATFORM_ASSERT(seqDlyCfgGet[i].seqDlyOn == seqDlyCfgSet[i].seqDlyOn);
        PLATFORM_ASSERT(seqDlyCfgGet[i].seqDlyOff == seqDlyCfgSet[i].seqDlyOff);
    }
}

/* ========================================================================== */
/*                    Buck Configuration Tests                                */
/* ========================================================================== */

/* ========================================================================== */
/*                 Buck Configuration Positive Tests                          */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_enable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1,
        .enable = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == true);
}

void test_pos_power_buckSetGetCfg_enable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2,
        .enable = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == true);
}

void test_pos_power_buckSetGetCfg_enable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3,
        .enable = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == true);
}

void test_pos_power_buckSetGetCfg_disable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1,
        .enable = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == false);
}

void test_pos_power_buckSetGetCfg_disable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2,
        .enable = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == false);
}

void test_pos_power_buckSetGetCfg_disable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3,
        .enable = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == false);
}

void test_pos_power_buckSetGetCfg_fpwmEn_enable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK1,
        .fpwmEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.fpwmEn == true);
}

void test_pos_power_buckSetGetCfg_fpwmEn_enable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK2,
        .fpwmEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.fpwmEn == true);
}

void test_pos_power_buckSetGetCfg_fpwmEn_enable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK3,
        .fpwmEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.fpwmEn == true);
}

void test_pos_power_buckSetGetCfg_fpwmEn_disable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK1,
        .fpwmEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.fpwmEn == false);
}

void test_pos_power_buckSetGetCfg_fpwmEn_disable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK2,
        .fpwmEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.fpwmEn == false);
}

void test_pos_power_buckSetGetCfg_fpwmEn_disable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK3,
        .fpwmEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.fpwmEn == false);
}

void test_pos_power_buckSetGetCfg_pldnEn_enable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK1,
        .pldnEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.pldnEn == true);
}

void test_pos_power_buckSetGetCfg_pldnEn_enable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK2,
        .pldnEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.pldnEn == true);
}

void test_pos_power_buckSetGetCfg_pldnEn_enable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK3,
        .pldnEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.pldnEn == true);
}

void test_pos_power_buckSetGetCfg_pldnEn_disable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK1,
        .pldnEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.pldnEn == false);
}

void test_pos_power_buckSetGetCfg_pldnEn_disable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK2,
        .pldnEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.pldnEn == false);
}

void test_pos_power_buckSetGetCfg_pldnEn_disable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK3,
        .pldnEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_PLDN_EN_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.pldnEn == false);
}

void test_pos_power_buckSetGetCfg_dischargeSel_buck1_active(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK1,
        .dischargeSel = PMIC_BUCK_SLEW_RATE_CONTROLLED
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dischargeSel == PMIC_BUCK_SLEW_RATE_CONTROLLED);
}

void test_pos_power_buckSetGetCfg_dischargeSel_buck1_resistive(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK1,
        .dischargeSel = PMIC_BUCK_RESISTIVE_DISCHARGE
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dischargeSel == PMIC_BUCK_RESISTIVE_DISCHARGE);
}

void test_pos_power_buckSetGetCfg_dischargeSel_buck2_active(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK2,
        .dischargeSel = PMIC_BUCK_SLEW_RATE_CONTROLLED
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dischargeSel == PMIC_BUCK_SLEW_RATE_CONTROLLED);
}

void test_pos_power_buckSetGetCfg_dischargeSel_buck2_resistive(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK2,
        .dischargeSel = PMIC_BUCK_RESISTIVE_DISCHARGE
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dischargeSel == PMIC_BUCK_RESISTIVE_DISCHARGE);
}

void test_pos_power_buckSetGetCfg_dischargeSel_buck3_active(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK3,
        .dischargeSel = PMIC_BUCK_SLEW_RATE_CONTROLLED
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dischargeSel == PMIC_BUCK_SLEW_RATE_CONTROLLED);
}

void test_pos_power_buckSetGetCfg_dischargeSel_buck3_resistive(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK3,
        .dischargeSel = PMIC_BUCK_RESISTIVE_DISCHARGE
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.dischargeSel == PMIC_BUCK_RESISTIVE_DISCHARGE);
}

void test_pos_power_buckSetGetCfg_ssEn_enable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK1,
        .ssEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == true);
}

void test_pos_power_buckSetGetCfg_ssEn_enable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK2,
        .ssEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == true);
}

void test_pos_power_buckSetGetCfg_ssEn_enable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK3,
        .ssEn = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == true);
}

void test_pos_power_buckSetGetCfg_ssEn_disable_buck1(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK1,
        .ssEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == false);
}

void test_pos_power_buckSetGetCfg_ssEn_disable_buck2(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK2,
        .ssEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == false);
}

void test_pos_power_buckSetGetCfg_ssEn_disable_buck3(void)
{
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK3,
        .ssEn = false
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SS_EN_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.ssEn == false);
}

/* ========================================================================== */
/*                    Buck Configuration GET-only Tests                       */
/* ========================================================================== */

/* OVP Fault Response GET Tests */
void test_pos_power_buckGetCfg_ovpSel_buck1(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_ovpSel_buck2(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_ovpSel_buck3(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* OV Fault Response GET Tests */
void test_pos_power_buckGetCfg_ovSel_buck1(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_ovSel_buck2(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_ovSel_buck3(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* UV Fault Response GET Tests */
void test_pos_power_buckGetCfg_uvSel_buck1(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_uvSel_buck2(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_uvSel_buck3(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* SC Fault Response GET Tests */
void test_pos_power_buckGetCfg_scSel_buck1(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_scSel_buck2(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_scSel_buck3(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* RV Configuration GET Tests */
void test_pos_power_buckGetCfg_rvConf_buck1(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_rvConf_buck2(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK2
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_rvConf_buck3(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK3
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* Current Limit Selection GET Tests */
void test_pos_power_buckGetCfg_ilimSel_buck1(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_ilimSel_buck2(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

void test_pos_power_buckGetCfg_ilimSel_buck3(void)
{
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/* ========================================================================== */
/*                     Buck Configuration Tests                               */
/* ========================================================================== */

/* ========================================================================== */
/*                 Buck Current Limit Selection Tests                         */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_ilimSel_buck1_allValues(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Test all valid BUCK1 current limit values
    for (uint8_t ilimSel = PMIC_BUCK1_ILIM_3P1_A; ilimSel <= PMIC_BUCK1_ILIM_MAX; ilimSel++)
    {
        buckCfgSet.ilimSel = ilimSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.ilimSel == ilimSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck2_allValues(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Test all valid BUCK2 current limit values
    for (uint8_t ilimSel = PMIC_BUCK2_3_ILIM_4P5_A; ilimSel <= PMIC_BUCK2_3_ILIM_MAX; ilimSel++)
    {
        buckCfgSet.ilimSel = ilimSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.ilimSel == ilimSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck3_allValues(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Test all valid BUCK3 current limit values
    for (uint8_t ilimSel = PMIC_BUCK2_3_ILIM_4P5_A; ilimSel <= PMIC_BUCK2_3_ILIM_MAX; ilimSel++)
    {
        buckCfgSet.ilimSel = ilimSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.ilimSel == ilimSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck1_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ilimSel = PMIC_BUCK1_ILIM_3P1_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 current limit to minimum
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 current limit
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK1_ILIM_3P1_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck1_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ilimSel = PMIC_BUCK1_ILIM_3P6_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 current limit to maximum
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 current limit
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK1_ILIM_3P6_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck2_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ilimSel = PMIC_BUCK2_3_ILIM_4P5_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 current limit to minimum
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 current limit
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK2_3_ILIM_4P5_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck2_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ilimSel = PMIC_BUCK2_3_ILIM_3_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 current limit to maximum
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 current limit
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK2_3_ILIM_3_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck3_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3,
        .ilimSel = PMIC_BUCK2_3_ILIM_4P5_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 current limit to minimum
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 current limit
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK2_3_ILIM_4P5_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ilimSel_buck3_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3,
        .ilimSel = PMIC_BUCK2_3_ILIM_3_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 current limit to maximum
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 current limit
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK2_3_ILIM_3_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                    Buck OVP Fault Response Tests                           */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_ovpSel_buck1_ignore(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovpSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 OVP fault response to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 OVP fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovpSel_buck1_assertNint(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovpSel = PMIC_WARM_RESET_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 OVP fault response to warm reset
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 OVP fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_WARM_RESET_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovpSel_buck2_waitPwrCycle(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ovpSel = PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 OVP fault response to wait for power cycle
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 OVP fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovpSel_buck3_ordShutdown(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK3,
        .ovpSel = PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 OVP fault response to ordered shutdown
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 OVP fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovpSel_buck1_immShutdown(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovpSel = PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 OVP fault response to ordered shutdown
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 OVP fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovpSel_buck2_allResponses(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Test all valid OVP fault responses for BUCK2
    for (uint8_t ovpSel = PMIC_ASSERT_NINT_PIN; ovpSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; ovpSel++)
    {
        buckCfgSet.ovpSel = ovpSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.ovpSel == ovpSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                    Buck OV Fault Response Tests                            */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_ovSel_buck1_ignore(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 OV fault response to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 OV fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovSel_buck2_assertNint(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ovSel = PMIC_WARM_RESET_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 OV fault response to warm reset
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 OV fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovSel == PMIC_WARM_RESET_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovSel_buck3_waitPwrCycle(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK3,
        .ovSel = PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 OV fault response to wait for power cycle
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 OV fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovSel == PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovSel_buck1_allResponses(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK1
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Test all valid OV fault responses for BUCK1
    for (uint8_t ovSel = PMIC_ASSERT_NINT_PIN; ovSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; ovSel++)
    {
        buckCfgSet.ovSel = ovSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.ovSel == ovSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                    Buck UV Fault Response Tests                            */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_uvSel_buck1_ignore(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK1,
        .uvSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 UV fault response to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 UV fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvSel_buck2_assertNint(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK2,
        .uvSel = PMIC_WARM_RESET_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 UV fault response to warm reset
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 UV fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvSel == PMIC_WARM_RESET_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvSel_buck3_allResponses(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Test all valid UV fault responses for BUCK3
    for (uint8_t uvSel = PMIC_ASSERT_NINT_PIN; uvSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; uvSel++)
    {
        buckCfgSet.uvSel = uvSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.uvSel == uvSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvSel_buck1_waitPwrCycle(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK1,
        .uvSel = PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 UV fault response to wait for power cycle
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 UV fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvSel == PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                    Buck SC Fault Response Tests                            */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_scSel_buck1_ignore(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1,
        .scSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 SC fault response to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 SC fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.scSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_scSel_buck2_assertNint(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK2,
        .scSel = PMIC_WARM_RESET_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 SC fault response to warm reset
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 SC fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.scSel == PMIC_WARM_RESET_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_scSel_buck3_allResponses(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Test all valid SC fault responses for BUCK3
    for (uint8_t scSel = PMIC_ASSERT_NINT_PIN; scSel <= PMIC_REGULATOR_FAULT_RESPONSE_MAX; scSel++)
    {
        buckCfgSet.scSel = scSel;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.scSel == scSel);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_scSel_buck1_ordShutdown(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1,
        .scSel = PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 SC fault response to ordered shutdown
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 SC fault response
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.scSel == PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                    Buck RV Configuration Tests                             */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_rvConf_buck1_wait(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK1,
        .rvConf = PMIC_BUCK_RAIL_DISCHARGE
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 RV configuration to rail discharge
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 RV configuration
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.rvConf == PMIC_BUCK_RAIL_DISCHARGE);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_rvConf_buck2_ignore(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK2,
        .rvConf = PMIC_BUCK_RV_IGNORE
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 RV configuration to ignore
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 RV configuration
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.rvConf == PMIC_BUCK_RV_IGNORE);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_rvConf_buck3_allValues(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK3
    };

    // Test all valid RV configuration values for BUCK3
    for (uint8_t rvConf = PMIC_BUCK_RAIL_DISCHARGE; rvConf <= PMIC_BUCK_RV_CONF_MAX; rvConf++)
    {
        buckCfgSet.rvConf = rvConf;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.rvConf == rvConf);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*          BUCK2/BUCK3 Specific Configuration Path Coverage Tests           */
/* ========================================================================== */

void test_pos_power_buckSetCfg_buck2OvpSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ovpSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 OVP selection to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK2 OVP selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck3OvpSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK3,
        .ovpSel = PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 OVP selection to ordered shutdown
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK3 OVP selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck2OvSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ovSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 OV threshold selection to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK2 OV threshold selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck3OvSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK3,
        .ovSel = PMIC_WARM_RESET_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 OV threshold selection to warm reset
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK3 OV threshold selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovSel == PMIC_WARM_RESET_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck2UvSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK2,
        .uvSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 UV threshold selection to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK2 UV threshold selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck3UvSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK3,
        .uvSel = PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 UV threshold selection to wait for power cycle
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK3 UV threshold selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvSel == PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck2ScSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK2,
        .scSel = PMIC_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 SC threshold selection to assert NINT
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK2 SC threshold selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.scSel == PMIC_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck3ScSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK3,
        .scSel = PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 SC threshold selection to ordered shutdown
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK3 SC threshold selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.scSel == PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck1RvConf(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK1,
        .rvConf = PMIC_BUCK_RV_IGNORE
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 RV configuration to ignore
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK1 RV configuration was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.rvConf == PMIC_BUCK_RV_IGNORE);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetCfg_buck1IlimSel(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ilimSel = PMIC_BUCK1_ILIM_3P6_A
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 current limit selection to 3.6A
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Verify BUCK1 current limit selection was set correctly
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == PMIC_BUCK1_ILIM_3P6_A);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                 Buck Multi-Parameter Configuration Tests                   */
/* ========================================================================== */

void test_pos_power_buckSetGetCfg_multiParam_buck1_enableVsetPldnFpwm(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_VSET_VALID |
                       PMIC_BUCK_PLDN_EN_VALID | PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK1,
        .enable = true,
        .vset = PMIC_BUCK1_VSET_MIN,
        .pldnEn = true,
        .fpwmEn = true
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_VSET_VALID |
                       PMIC_BUCK_PLDN_EN_VALID | PMIC_BUCK_FPWM_EN_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 multi-parameter configuration
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 multi-parameter configuration
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.enable == buckCfgSet.enable);
    PLATFORM_ASSERT(buckCfgGet.vset == buckCfgSet.vset);
    PLATFORM_ASSERT(buckCfgGet.pldnEn == buckCfgSet.pldnEn);
    PLATFORM_ASSERT(buckCfgGet.fpwmEn == buckCfgSet.fpwmEn);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_multiParam_buck2_enableVsetActiveThresholds(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_VSET_ACTIVE_VALID |
                       PMIC_BUCK_UV_THR_VALID | PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK2,
        .enable = true,
        .vsetActive = PMIC_BUCK2_3_VSET_MIN,
        .uvThr = PMIC_BUCK_UV_THR_4_PCT,
        .ovThr = PMIC_BUCK_OV_THR_4_PCT
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_VSET_ACTIVE_VALID |
                       PMIC_BUCK_UV_THR_VALID | PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 multi-parameter configuration
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 multi-parameter configuration
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.enable == buckCfgSet.enable);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == buckCfgSet.vsetActive);
    PLATFORM_ASSERT(buckCfgGet.uvThr == buckCfgSet.uvThr);
    PLATFORM_ASSERT(buckCfgGet.ovThr == buckCfgSet.ovThr);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_multiParam_buck3_allCommonParams(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_PLDN_EN_VALID |
                       PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_SLEW_RATE_VALID |
                       PMIC_BUCK_DEGLITCH_SEL_VALID | PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK3,
        .enable = true,
        .pldnEn = true,
        .fpwmEn = false,
        .slewRate = PMIC_BUCK_SLEW_RATE_10_MV_PER_US,
        .deglitchSel = PMIC_BUCK_DEGLITCH_SEL_10_US,
        .dischargeSel = PMIC_BUCK_SLEW_RATE_CONTROLLED
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_PLDN_EN_VALID |
                       PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_SLEW_RATE_VALID |
                       PMIC_BUCK_DEGLITCH_SEL_VALID | PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK3
    };

    // Set BUCK3 multi-parameter configuration
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK3 multi-parameter configuration
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.enable == buckCfgSet.enable);
    PLATFORM_ASSERT(buckCfgGet.pldnEn == buckCfgSet.pldnEn);
    PLATFORM_ASSERT(buckCfgGet.fpwmEn == buckCfgSet.fpwmEn);
    PLATFORM_ASSERT(buckCfgGet.slewRate == buckCfgSet.slewRate);
    PLATFORM_ASSERT(buckCfgGet.deglitchSel == buckCfgSet.deglitchSel);
    PLATFORM_ASSERT(buckCfgGet.dischargeSel == buckCfgSet.dischargeSel);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_multiParam_buck1_allFaultResponses(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID | PMIC_BUCK_OV_SEL_VALID |
                       PMIC_BUCK_UV_SEL_VALID | PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovpSel = PMIC_ASSERT_NINT_PIN,
        .ovSel = PMIC_WARM_RESET_AND_ASSERT_NINT_PIN,
        .uvSel = PMIC_ORD_SHUTDOWN_AND_ASSERT_NINT_PIN,
        .scSel = PMIC_WAIT_PWR_CYCLE_AND_ASSERT_NINT_PIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID | PMIC_BUCK_OV_SEL_VALID |
                       PMIC_BUCK_UV_SEL_VALID | PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1
    };

    // Set BUCK1 all fault responses
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK1 all fault responses
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovpSel == buckCfgSet.ovpSel);
    PLATFORM_ASSERT(buckCfgGet.ovSel == buckCfgSet.ovSel);
    PLATFORM_ASSERT(buckCfgGet.uvSel == buckCfgSet.uvSel);
    PLATFORM_ASSERT(buckCfgGet.scSel == buckCfgSet.scSel);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_multiParam_buck2_fullConfig(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_PLDN_EN_VALID |
                       PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_VSET_ACTIVE_VALID |
                       PMIC_BUCK_VSET_LPWR_VALID | PMIC_BUCK_ILIM_SEL_VALID |
                       PMIC_BUCK_RV_CONF_VALID | PMIC_BUCK_VMON_ONLY_VALID,
        .resource = PMIC_BUCK2,
        .enable = true,
        .pldnEn = false,
        .fpwmEn = true,
        .vsetActive = PMIC_BUCK2_3_VSET_MIN,
        .vsetLPwr = PMIC_BUCK2_3_VSET_MAX,
        .ilimSel = PMIC_BUCK2_3_ILIM_4P5_A,
        .rvConf = PMIC_BUCK_RAIL_DISCHARGE,
        .vmonOnly = false
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_ENABLE_VALID | PMIC_BUCK_PLDN_EN_VALID |
                       PMIC_BUCK_FPWM_EN_VALID | PMIC_BUCK_VSET_ACTIVE_VALID |
                       PMIC_BUCK_VSET_LPWR_VALID | PMIC_BUCK_ILIM_SEL_VALID |
                       PMIC_BUCK_RV_CONF_VALID | PMIC_BUCK_VMON_ONLY_VALID,
        .resource = PMIC_BUCK2
    };

    // Set BUCK2 full configuration
    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    // Get BUCK2 full configuration
    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.enable == buckCfgSet.enable);
    PLATFORM_ASSERT(buckCfgGet.pldnEn == buckCfgSet.pldnEn);
    PLATFORM_ASSERT(buckCfgGet.fpwmEn == buckCfgSet.fpwmEn);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == buckCfgSet.vsetActive);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == buckCfgSet.vsetLPwr);
    PLATFORM_ASSERT(buckCfgGet.ilimSel == buckCfgSet.ilimSel);
    PLATFORM_ASSERT(buckCfgGet.rvConf == buckCfgSet.rvConf);
    PLATFORM_ASSERT(buckCfgGet.vmonOnly == buckCfgSet.vmonOnly);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*           Buck Voltage and Threshold Configuration Tests                   */
/* ========================================================================== */

/* BUCK1 VSET Tests */
void test_pos_power_buckSetGetCfg_vset_buck1_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1,
        .vset = 0x00
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vset == 0x00);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vset_buck1_mid(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1,
        .vset = 0x06
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vset == 0x06);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vset_buck1_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1,
        .vset = 0x0D
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vset == 0x0D);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vset_buck1_boundary_low(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1,
        .vset = 0x01
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vset == 0x01);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vset_buck1_boundary_high(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1,
        .vset = 0x0C
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vset == 0x0C);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* BUCK2/3 VSET Active Tests */
void test_pos_power_buckSetGetCfg_vsetActive_buck2_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2,
        .vsetActive = PMIC_BUCK2_3_VSET_MIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == PMIC_BUCK2_3_VSET_MIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck2_mid(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2,
        .vsetActive = 0x22
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == 0x22);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck2_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2,
        .vsetActive = 0x45
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == 0x45);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck3_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3,
        .vsetActive = PMIC_BUCK2_3_VSET_MIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == PMIC_BUCK2_3_VSET_MIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck3_mid(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3,
        .vsetActive = 0x22
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == 0x22);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck3_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3,
        .vsetActive = 0x45
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetActive == 0x45);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck2_boundary(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t boundaryValues[] = {PMIC_BUCK2_3_VSET_MIN + 1, PMIC_BUCK2_3_VSET_MAX - 1};
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };

    for (uint8_t i = 0; i < 2; i++)
    {
        buckCfgSet.vsetActive = boundaryValues[i];
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.vsetActive == boundaryValues[i]);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetActive_buck3_boundary(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t boundaryValues[] = {PMIC_BUCK2_3_VSET_MIN + 1, PMIC_BUCK2_3_VSET_MAX - 1};
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };

    for (uint8_t i = 0; i < 2; i++)
    {
        buckCfgSet.vsetActive = boundaryValues[i];
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.vsetActive == boundaryValues[i]);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* BUCK2/3 VSET LPwr Tests */
void test_pos_power_buckSetGetCfg_vsetLPwr_buck2_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2,
        .vsetLPwr = PMIC_BUCK2_3_VSET_MIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == PMIC_BUCK2_3_VSET_MIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetLPwr_buck2_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2,
        .vsetLPwr = 0x45
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == 0x45);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetLPwr_buck3_min(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK3,
        .vsetLPwr = PMIC_BUCK2_3_VSET_MIN
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == PMIC_BUCK2_3_VSET_MIN);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetLPwr_buck3_max(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK3,
        .vsetLPwr = 0x45
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == 0x45);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetLPwr_buck2_mid(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2,
        .vsetLPwr = 0x22
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == 0x22);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vsetLPwr_buck3_mid(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK3,
        .vsetLPwr = 0x22
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vsetLPwr == 0x22);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* VMON Only Tests */
void test_pos_power_buckSetGetCfg_vmonOnly_buck2_enable(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VMON_ONLY_VALID,
        .resource = PMIC_BUCK2,
        .vmonOnly = true
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VMON_ONLY_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vmonOnly == true);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_vmonOnly_buck3_enable(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_VMON_ONLY_VALID,
        .resource = PMIC_BUCK3,
        .vmonOnly = true
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_VMON_ONLY_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.vmonOnly == true);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* UV Threshold Tests */
void test_pos_power_buckSetGetCfg_uvThr_buck1_val0(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK1,
        .uvThr = 0
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvThr == 0);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvThr_buck1_val1(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK1,
        .uvThr = 1
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvThr == 1);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvThr_buck2_val2(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK2,
        .uvThr = 2
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvThr == 2);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvThr_buck2_val3(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK2,
        .uvThr = 3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvThr == 3);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvThr_buck3_val0(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK3,
        .uvThr = 0
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvThr == 0);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_uvThr_buck3_val3(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK3,
        .uvThr = 3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.uvThr == 3);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* OV Threshold Tests */
void test_pos_power_buckSetGetCfg_ovThr_buck1_val0(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK1,
        .ovThr = 0
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovThr == 0);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovThr_buck1_val1(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK1,
        .ovThr = 1
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovThr == 1);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovThr_buck2_val2(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK2,
        .ovThr = 2
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovThr == 2);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovThr_buck2_val3(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK2,
        .ovThr = 3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovThr == 3);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovThr_buck3_val0(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK3,
        .ovThr = 0
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovThr == 0);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_ovThr_buck3_val3(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK3,
        .ovThr = 3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.ovThr == 3);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* Slew Rate Tests */
void test_pos_power_buckSetGetCfg_slewRate_buck1_allValues(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1
    };

    // Test all valid slew rate values for BUCK1
    for (uint8_t slewRate = PMIC_BUCK_SLEW_RATE_10_MV_PER_US; slewRate <= PMIC_BUCK_SLEW_RATE_MAX; slewRate++)
    {
        buckCfgSet.slewRate = slewRate;
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.slewRate == slewRate);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_slewRate_buck2_minMax(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t slewRateValues[] = {PMIC_BUCK_SLEW_RATE_10_MV_PER_US, PMIC_BUCK_SLEW_RATE_MAX};
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK2
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK2
    };

    for (uint8_t i = 0; i < 2; i++)
    {
        buckCfgSet.slewRate = slewRateValues[i];
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.slewRate == slewRateValues[i]);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_slewRate_buck3_minMax(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t slewRateValues[] = {PMIC_BUCK_SLEW_RATE_10_MV_PER_US, PMIC_BUCK_SLEW_RATE_MAX};
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK3
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK3
    };

    for (uint8_t i = 0; i < 2; i++)
    {
        buckCfgSet.slewRate = slewRateValues[i];
        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.slewRate == slewRateValues[i]);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* High Side Slew Rate Tests (BUCK1 only) */
void test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_fast(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1,
        .highSideSlewRate = 0
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.highSideSlewRate == 0);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_slow(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1,
        .highSideSlewRate = 1
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.highSideSlewRate == 1);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_buckSetGetCfg_highSideSlewRate_buck1_slowest(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1,
        .highSideSlewRate = PMIC_HIGH_SIDE_SLEW_RATE_SLOWEST
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(buckCfgGet.highSideSlewRate == PMIC_HIGH_SIDE_SLEW_RATE_SLOWEST);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* Deglitch Selection Test */
void test_pos_power_buckSetGetCfg_deglitchSel_allBucks(void)
{
#ifdef BUILD_MOCK
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t resources[] = {PMIC_BUCK1, PMIC_BUCK2, PMIC_BUCK3};
    Pmic_PwrBuckCfg_t buckCfgSet = {
        .validParams = PMIC_BUCK_DEGLITCH_SEL_VALID,
        .deglitchSel = PMIC_BUCK_DEGLITCH_SEL_10_US
    };
    Pmic_PwrBuckCfg_t buckCfgGet = {
        .validParams = PMIC_BUCK_DEGLITCH_SEL_VALID
    };

    // Test deglitch selection for all three BUCKs
    for (uint8_t i = 0; i < 3; i++)
    {
        buckCfgSet.resource = resources[i];
        buckCfgGet.resource = resources[i];

        status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(buckCfgGet.deglitchSel == PMIC_BUCK_DEGLITCH_SEL_10_US);
    }
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                    Buck Configuration Negative Tests                       */
/* ========================================================================== */

void test_neg_power_pwrSetBuckCfg_nullParam_pmicHandle(void)
{
#ifdef BUILD_MOCK
    // Pass NULL pmicHandle into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK1,
        .enable = true
    };
    int32_t status = Pmic_pwrSetBuckCfg(NULL, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_nullParam_buckCfg(void)
{
#ifdef BUILD_MOCK
    // Pass NULL buckCfg into Pmic_pwrSetBuckCfg()
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrGetBuckCfg_nullParam_buckCfg(void)
{
#ifdef BUILD_MOCK
    // Pass NULL buckCfg into Pmic_pwrGetBuckCfg()
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_invalidParam_resource(void)
{
#ifdef BUILD_MOCK
    // Pass invalid resource into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK_MAX + 1U,
        .enable = true
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrGetBuckCfg_invalidParam_resource(void)
{
#ifdef BUILD_MOCK
    // Pass invalid resource into Pmic_pwrGetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK_MAX + 1U
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_zeroValidParams(void)
{
#ifdef BUILD_MOCK
    // Pass zero validParams into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = 0U,
        .resource = PMIC_BUCK1,
        .enable = true
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrGetBuckCfg_zeroValidParams(void)
{
#ifdef BUILD_MOCK
    // Pass zero validParams into Pmic_pwrGetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = 0U,
        .resource = PMIC_BUCK1
    };
    int32_t status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_vset(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds vset into Pmic_pwrSetBuckCfg() for BUCK1
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_VSET_VALID,
        .resource = PMIC_BUCK1,
        .vset = 0x0EU  // PMIC_BUCK1_VSET_MAX is 0x0D
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_vsetActive(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds vsetActive into Pmic_pwrSetBuckCfg() for BUCK2
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2,
        .vsetActive = 0x46U  // PMIC_BUCK2_3_VSET_MAX is 0x45
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_vsetLPwr(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds vsetLPwr into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_VSET_LPWR_VALID,
        .resource = PMIC_BUCK2,
        .vsetLPwr = 0x46U  // PMIC_BUCK2_3_VSET_MAX is 0x45
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_uvThr(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds uvThr into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_UV_THR_VALID,
        .resource = PMIC_BUCK1,
        .uvThr = PMIC_BUCK_UV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_ovThr(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds ovThr into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_OV_THR_VALID,
        .resource = PMIC_BUCK1,
        .ovThr = PMIC_BUCK_OV_THR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_ilimSel_buck1(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds ilimSel into Pmic_pwrSetBuckCfg() for BUCK1
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ilimSel = PMIC_BUCK1_ILIM_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_ilimSel_buck2(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds ilimSel into Pmic_pwrSetBuckCfg() for BUCK2
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ILIM_SEL_VALID,
        .resource = PMIC_BUCK2,
        .ilimSel = PMIC_BUCK2_3_ILIM_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_ovpSel(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds ovpSel into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_OVP_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovpSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_ovSel(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds ovSel into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_OV_SEL_VALID,
        .resource = PMIC_BUCK1,
        .ovSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_uvSel(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds uvSel into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_UV_SEL_VALID,
        .resource = PMIC_BUCK1,
        .uvSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_scSel(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds scSel into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_SC_SEL_VALID,
        .resource = PMIC_BUCK1,
        .scSel = PMIC_REGULATOR_FAULT_RESPONSE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_rvConf(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds rvConf into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_RV_CONF_VALID,
        .resource = PMIC_BUCK1,
        .rvConf = PMIC_BUCK_RV_CONF_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_slewRate(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds slewRate into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1,
        .slewRate = PMIC_BUCK_SLEW_RATE_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_deglitchSel(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds deglitchSel into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_DEGLITCH_SEL_VALID,
        .resource = PMIC_BUCK1,
        .deglitchSel = PMIC_BUCK_DEGLITCH_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_dischargeSel(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds dischargeSel into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK1,
        .dischargeSel = PMIC_BUCK_DISCHARGE_SEL_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_uvloRising(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds uvloRising into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_UVLO_RISING_VALID,
        .resource = PMIC_BUCK1,
        .uvloRising = 0xFU + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_uvloFalling(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds uvloFalling into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_UVLO_FALLING_VALID,
        .resource = PMIC_BUCK1,
        .uvloFalling = 0xFU + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwrSetBuckCfg_outOfBounds_highSideSlewRate(void)
{
#ifdef BUILD_MOCK
    // Pass out of bounds highSideSlewRate into Pmic_pwrSetBuckCfg()
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_HIGH_SIDE_SLEW_RATE_VALID,
        .resource = PMIC_BUCK1,
        .highSideSlewRate = PMIC_BUCK1_EN_HS_ON_SR_MAX + 1U
    };
    int32_t status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

/* ========================================================================== */
/*                   BUCK2/BUCK3 Coverage Gap Tests                           */
/* ========================================================================== */

void test_pos_power_powerGetCfg_buck2Enable(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerSetCfg_buck2Enable(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2,
        .enable = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == true);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerGetVoutCfg_buck2Voltage(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerSetVoutCfg_buck2Voltage(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2,
        .vsetActive = 0x20
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vsetActive == 0x20);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerGetCfg_buck3Enable(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerSetCfg_buck3Enable(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3,
        .enable = true
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_ENABLE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.enable == true);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerGetVoutCfg_buck3Voltage(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerSetVoutCfg_buck3Voltage(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckCfg_t setCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3,
        .vsetActive = 0x30
    };
    Pmic_PwrBuckCfg_t getCfg = {
        .validParams = PMIC_BUCK_VSET_ACTIVE_VALID,
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &setCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &getCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(getCfg.vsetActive == 0x30);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerGetStat_buck2(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_BUCK2
    };

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_pos_power_powerGetStat_buck3(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrRsrcStatus_t pwrRsrcStat = {
        .resource = PMIC_BUCK3
    };

    status = Pmic_pwrGetRsrcStatus(&pmicHandle, &pwrRsrcStat);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwr_getUvlo_invalidBuck(void)
{
#ifdef BUILD_MOCK
    // Test invalid BUCK resource for UVLO (line 654)
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_UVLO_FALLING_VALID | PMIC_BUCK_UVLO_RISING_VALID,
        .resource = PMIC_BUCK2  // Only BUCK1 supports UVLO
    };

    status = Pmic_pwrGetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwr_setUvlo_invalidBuck(void)
{
#ifdef BUILD_MOCK
    // Test invalid BUCK resource for UVLO (line 931)
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_UVLO_FALLING_VALID | PMIC_BUCK_UVLO_RISING_VALID,
        .resource = PMIC_BUCK3,  // Only BUCK1 supports UVLO
        .uvloFalling = 0x5U,
        .uvloRising = 0x3U
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwr_setBuck_invalidDischargeSel(void)
{
#ifdef BUILD_MOCK
    // Test invalid discharge selector (lines 1358-1359)
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_DISCHARGE_SEL_VALID,
        .resource = PMIC_BUCK2,
        .dischargeSel = PMIC_BUCK_DISCHARGE_SEL_MAX + 1U  // Out of bounds
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwr_setBuck_invalidSlewRate(void)
{
#ifdef BUILD_MOCK
    // Test invalid slew rate (lines 1378-1379)
    int32_t status;
    Pmic_PwrBuckCfg_t buckCfg = {
        .validParams = PMIC_BUCK_SLEW_RATE_VALID,
        .resource = PMIC_BUCK2,
        .slewRate = PMIC_BUCK_SLEW_RATE_MAX + 1U  // Out of bounds
    };

    status = Pmic_pwrSetBuckCfg(&pmicHandle, &buckCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation or mock-specific testing");
#endif
}

void test_neg_power_pwr_setLdoCfg_zeroValidParams(void)
{
#ifdef BUILD_MOCK
    // Test zero validParams for setLdoCfg (lines 1865-1866)
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = 0U  // Zero validParams
    };

    status = Pmic_pwrSetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_getLdoCfg_zeroValidParams(void)
{
#ifdef BUILD_MOCK
    // Test zero validParams for getLdoCfg (lines 2280-2281)
    int32_t status;
    Pmic_PwrLdoCfg_t ldoCfg = {
        .validParams = 0U  // Zero validParams
    };

    status = Pmic_pwrGetLdoCfg(&pmicHandle, &ldoCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_setTsdCfg_zeroValidParams(void)
{
#ifdef BUILD_MOCK
    // Test zero validParams for setTsdCfg (lines 2491-2492)
    int32_t status;
    Pmic_PwrTsdCfg_t tsdCfg = {
        .validParams = 0U  // Zero validParams
    };

    status = Pmic_pwrSetTsdCfg(&pmicHandle, &tsdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_getTsdCfg_zeroValidParams(void)
{
#ifdef BUILD_MOCK
    // Test zero validParams for getTsdCfg (lines 2568-2569)
    int32_t status;
    Pmic_PwrTsdCfg_t tsdCfg = {
        .validParams = 0U  // Zero validParams
    };

    status = Pmic_pwrGetTsdCfg(&pmicHandle, &tsdCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_setSeqTrig_zeroLen(void)
{
#ifdef BUILD_MOCK
    // Test zero length for setSeqTrig (lines 2775-2778)
    int32_t status;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[1];  // Non-NULL array to test len check

    // Pass zero length to the API
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfg, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_setSeqTrig_invalidPwrRsrc(void)
{
#ifdef BUILD_MOCK
    // Test invalid power resource (lines 2715-2716)
    int32_t status;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[] = {
        {
            .trigger = 0xFFFFU,  // Invalid trigger value
            .exclude = false
        }
    };

    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_setSeqTrig_invalidBitPos(void)
{
#ifdef BUILD_MOCK
    /**
     * Test invalid sequence trigger bit position
     * Covers pmic_power.c:2811-2812 (setSeqTrig) and 2724-2725 (getSeqTrig)
     * Trigger format: high byte = pwrRsrc, low byte = bitPos
     * Valid bitPos range: 0-5 (PMIC_PWR_SEQ_TRIG_BIT_POS_MAX = 5)
     */
    int32_t status;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg;

    // Test bitPos = 6 (invalid, max is 5)
    seqTrigCfg.trigger = (uint16_t)((PMIC_BUCK1 << 8U) | 6U);
    seqTrigCfg.exclude = PMIC_ENABLE;

    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Test bitPos = 7 (invalid)
    seqTrigCfg.trigger = (uint16_t)((PMIC_BUCK2 << 8U) | 7U);
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    // Test bitPos = 0xFF (maximum invalid)
    seqTrigCfg.trigger = (uint16_t)((PMIC_BUCK1 << 8U) | 0xFFU);
    status = Pmic_pwrSetBuckLdoSeqTrig(&pmicHandle, &seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_getSeqTrig_zeroLen(void)
{
#ifdef BUILD_MOCK
    // Test zero length for getSeqTrig (lines 2777-2778)
    int32_t status;
    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[1];  // Non-NULL array to test len check

    // Pass zero length to the API
    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfg, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_setSeqDelay_zeroLen(void)
{
#ifdef BUILD_MOCK
    // Test zero length for setSeqDelay (lines 2889-2890)
    int32_t status;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[1];  // Non-NULL array to test len check

    // Pass zero length to the API
    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfg, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

void test_neg_power_pwr_setSeqDelay_invalidConfig(void)
{
#ifdef BUILD_MOCK
    // Test invalid resource or zero validParams (lines 2910-2911)
    int32_t status;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[] = {
        {
            .resource = PMIC_BUCK1,
            .validParams = 0U  // Zero validParams
        }
    };

    status = Pmic_pwrSetBuckLdoSeqDly(&pmicHandle, seqDlyCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckLdoSeqTrig() with invalid resource
 * Covers resource bounds check in Pmic_pwrGetBuckLdoSeqTrig() loop
 */
void test_neg_power_pwr_getSeqTrig_invalidResource(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint16_t invalidResource = (uint16_t)(PMIC_POWER_RESOURCE_MAX + 1U);
    uint16_t invalidTrigger = (uint16_t)(invalidResource << 8U);

    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[] = {
        { .trigger = invalidTrigger, .exclude = false }
    };

    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckLdoSeqTrig() with invalid bit position
 * Covers bit position bounds check in Pmic_pwrGetBuckLdoSeqTrig() loop
 */
void test_neg_power_pwr_getSeqTrig_invalidBitPos(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    uint16_t invalidTrigger = ((uint16_t)PMIC_BUCK1 << 8U) | 6U;  // bitPos 6 > max 5

    Pmic_PwrBuckLdoSeqTrig_t seqTrigCfg[] = {
        { .trigger = invalidTrigger, .exclude = false }
    };

    status = Pmic_pwrGetBuckLdoSeqTrig(&pmicHandle, seqTrigCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckLdoSeqDly() with zero length
 * Covers zero length check in Pmic_pwrGetBuckLdoSeqDly()
 */
void test_neg_power_pwr_getSeqDelay_zeroLen(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[1];

    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfg, 0U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

/**
 * @brief Test Pmic_pwrGetBuckLdoSeqDly() with invalid resource or zero validParams
 * Covers resource/validParams check in Pmic_pwrGetBuckLdoSeqDly() loop
 */
void test_neg_power_pwr_getSeqDelay_invalidConfig(void)
{
#ifdef BUILD_MOCK
    int32_t status;
    Pmic_PwrBuckLdoSeqDly_t seqDlyCfg[] = {
        {
            .resource = PMIC_POWER_RESOURCE_MAX + 1U,  // Invalid resource
            .validParams = PMIC_SEQ_DLY_ON_VALID
        }
    };

    status = Pmic_pwrGetBuckLdoSeqDly(&pmicHandle, seqDlyCfg, 1U);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
#else
    TEST_IGNORE_MESSAGE("Test requires BUILD_MOCK for parameter validation");
#endif
}

