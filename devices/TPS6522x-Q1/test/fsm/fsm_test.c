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
 *    distribution and/or other materials provided with the
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


#include "platform.h"
#include "fsm_test.h"

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

static Pmic_Handle_t pmicHandle = {0};

/* ========================================================================== */
/*                       Negative Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test Pmic_fsmSetTriggerCfg with NULL handle
 */
void test_neg_fsm_fsmSetTriggerCfg_nullHandle(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {0};
    int32_t status = Pmic_fsmSetTriggerCfg(NULL, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetTriggerCfg with NULL triggerCfg pointer
 */
void test_neg_fsm_fsmSetTriggerCfg_nullCfg(void)
{
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetTriggerCfg with invalid severe error trigger
 */
void test_neg_fsm_fsmSetTriggerCfg_invalidSevereErrTrig(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID,
        .severeErrTrig = PMIC_FSM_TRIGGER_MAX + 1
    };
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_fsmGetTriggerCfg with NULL handle
 */
void test_neg_fsm_fsmGetTriggerCfg_nullHandle(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {0};
    int32_t status = Pmic_fsmGetTriggerCfg(NULL, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmGetTriggerCfg with NULL triggerCfg pointer
 */
void test_neg_fsm_fsmGetTriggerCfg_nullCfg(void)
{
    int32_t status = Pmic_fsmGetTriggerCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetGpioTriggerCfg with NULL handle
 */
void test_neg_fsm_fsmSetGpioTriggerCfg_nullHandle(void)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {0};
    int32_t status = Pmic_fsmSetGpioTriggerCfg(NULL, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetGpioTriggerCfg with NULL gpioTriggerCfg pointer
 */
void test_neg_fsm_fsmSetGpioTriggerCfg_nullCfg(void)
{
    int32_t status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetGpioTriggerCfg with invalid pin number
 */
void test_neg_fsm_fsmSetGpioTriggerCfg_invalidPin(void)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {
        .validParams = PMIC_FSM_MASK_VALID,
        .pinNum = PMIC_FSM_GPIO_PIN_MAX + 1,
        .mask = true
    };
    int32_t status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_fsmGetGpioTriggerCfg with NULL handle
 */
void test_neg_fsm_fsmGetGpioTriggerCfg_nullHandle(void)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {0};
    int32_t status = Pmic_fsmGetGpioTriggerCfg(NULL, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmGetGpioTriggerCfg with NULL gpioTriggerCfg pointer
 */
void test_neg_fsm_fsmGetGpioTriggerCfg_nullCfg(void)
{
    int32_t status = Pmic_fsmGetGpioTriggerCfg(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmGetGpioTriggerCfg with invalid pin number
 *
 * Covers pmic_fsm.c:436 - error path when Pmic_fsmGetGpioPinMapping fails
 * due to invalid pinNum in the trigger config structure.
 */
void test_neg_fsm_fsmGetGpioTriggerCfg_invalidPin(void)
{
    Pmic_FsmGpioTriggerCfg_t cfg;
    int32_t status;

    memset(&cfg, 0, sizeof(cfg));
    cfg.validParams = PMIC_FSM_MASK_VALID;  /* Must be non-zero to reach pin validation */
    cfg.pinNum = 0;  /* Invalid pin number (valid range is 1-6) */

    status = Pmic_fsmGetGpioTriggerCfg(&pmicHandle, &cfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_fsmGetRecovCnt with NULL handle
 */
void test_neg_fsm_fsmGetRecovCnt_nullHandle(void)
{
    uint8_t recovCnt = 0;
    int32_t status = Pmic_fsmGetRecovCnt(NULL, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmGetRecovCnt with NULL recovCnt pointer
 */
void test_neg_fsm_fsmGetRecovCnt_nullRecovCnt(void)
{
    int32_t status = Pmic_fsmGetRecovCnt(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmClrRecovCnt with NULL handle
 */
void test_neg_fsm_fsmClrRecovCnt_nullHandle(void)
{
    int32_t status = Pmic_fsmClrRecovCnt(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetRecovCntThr with NULL handle
 */
void test_neg_fsm_fsmSetRecovCntThr_nullHandle(void)
{
    int32_t status = Pmic_fsmSetRecovCntThr(NULL, 5);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetRecovCntThr with invalid threshold value
 */
void test_neg_fsm_fsmSetRecovCntThr_invalidValue(void)
{
    int32_t status = Pmic_fsmSetRecovCntThr(&pmicHandle, 16); /* Max is 15 */
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_fsmGetRecovCntThr with NULL handle
 */
void test_neg_fsm_fsmGetRecovCntThr_nullHandle(void)
{
    uint8_t recovCntThr = 0;
    int32_t status = Pmic_fsmGetRecovCntThr(NULL, &recovCntThr);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmGetRecovCntThr with NULL recovCntThr pointer
 */
void test_neg_fsm_fsmGetRecovCntThr_nullRecovCntThr(void)
{
    int32_t status = Pmic_fsmGetRecovCntThr(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSendSoftRebootReq with NULL handle
 */
void test_neg_fsm_fsmSendSoftRebootReq_nullHandle(void)
{
    int32_t status = Pmic_fsmSendSoftRebootReq(NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetStartupDest with NULL handle
 */
void test_neg_fsm_fsmSetStartupDest_nullHandle(void)
{
    int32_t status = Pmic_fsmSetStartupDest(NULL, PMIC_FSM_STARTUP_DEST_MCU_ONLY);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmSetStartupDest with invalid destination
 */
void test_neg_fsm_fsmSetStartupDest_invalidDest(void)
{
    int32_t status = Pmic_fsmSetStartupDest(&pmicHandle, PMIC_FSM_START_UP_DEST_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test Pmic_fsmGetStartupDest with NULL handle
 */
void test_neg_fsm_fsmGetStartupDest_nullHandle(void)
{
    uint8_t destination = 0;
    int32_t status = Pmic_fsmGetStartupDest(NULL, &destination);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/**
 * @brief Test Pmic_fsmGetStartupDest with NULL destination pointer
 */
void test_neg_fsm_fsmGetStartupDest_nullDestination(void)
{
    int32_t status = Pmic_fsmGetStartupDest(&pmicHandle, NULL);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_NULL_PARAM);
}

/* ========================================================================== */
/*                       Positive Test Functions                              */
/* ========================================================================== */

/**
 * @brief Test FSM trigger configuration set and get
 */
void test_pos_fsm_fsmSetGetTriggerCfg_allTriggers(void)
{
    int32_t status;
    Pmic_FsmTriggerCfg_t triggerCfgSet = {
        .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID |
                       PMIC_FSM_OTHER_RAIL_TRIG_VALID |
                       PMIC_FSM_SOC_RAIL_TRIG_VALID |
                       PMIC_FSM_MCU_RAIL_TRIG_VALID |
                       PMIC_FSM_MODERATE_ERR_TRIG_VALID,
        .severeErrTrig = PMIC_FSM_TRIGGER_MCU_POWER_ERROR,
        .otherRailTrig = PMIC_FSM_TRIGGER_SOC_POWER_ERROR,
        .socRailTrig = PMIC_FSM_TRIGGER_ORDERLY_SHUTDOWN,
        .mcuRailTrig = PMIC_FSM_TRIGGER_IMMEDIATE_SHUTDOWN,
        .moderateErrTrig = PMIC_FSM_TRIGGER_MCU_POWER_ERROR
    };

    status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_FsmTriggerCfg_t triggerCfgGet = {
        .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID |
                       PMIC_FSM_OTHER_RAIL_TRIG_VALID |
                       PMIC_FSM_SOC_RAIL_TRIG_VALID |
                       PMIC_FSM_MCU_RAIL_TRIG_VALID |
                       PMIC_FSM_MODERATE_ERR_TRIG_VALID
    };

    status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(triggerCfgGet.severeErrTrig == triggerCfgSet.severeErrTrig);
    PLATFORM_ASSERT(triggerCfgGet.otherRailTrig == triggerCfgSet.otherRailTrig);
    PLATFORM_ASSERT(triggerCfgGet.socRailTrig == triggerCfgSet.socRailTrig);
    PLATFORM_ASSERT(triggerCfgGet.mcuRailTrig == triggerCfgSet.mcuRailTrig);
    PLATFORM_ASSERT(triggerCfgGet.moderateErrTrig == triggerCfgSet.moderateErrTrig);
}

/**
 * @brief Test severe error trigger configuration
 */
void test_pos_fsm_fsmSetGetTriggerCfg_severeErrorTrigger(void)
{
    int32_t status;

    for (uint8_t val = PMIC_FSM_TRIGGER_MCU_POWER_ERROR; val <= PMIC_FSM_TRIGGER_MAX; val++)
    {
        Pmic_FsmTriggerCfg_t triggerCfgSet = {
            .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID,
            .severeErrTrig = val
        };

        status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        Pmic_FsmTriggerCfg_t triggerCfgGet = {
            .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID
        };

        status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(triggerCfgGet.severeErrTrig == val);
    }
}

/**
 * @brief Test GPIO trigger configuration
 */
void test_pos_fsm_fsmSetGetGpioTriggerCfg_gpio1(void)
{
    int32_t status;

    /* Test GPIO1 trigger configuration */
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfgSet = {
        .validParams = PMIC_FSM_MASK_VALID |
                       PMIC_FSM_MASK_POL_VALID,
        .pinNum = PMIC_FSM_GPIO_PIN1,
        .mask = true,
        .maskPol = PMIC_FSM_GPIO_MASK_POL_1
    };

    status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfgGet = {
        .validParams = PMIC_FSM_MASK_VALID |
                       PMIC_FSM_MASK_POL_VALID,
        .pinNum = PMIC_FSM_GPIO_PIN1
    };

    status = Pmic_fsmGetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(gpioTriggerCfgGet.mask == gpioTriggerCfgSet.mask);
    PLATFORM_ASSERT(gpioTriggerCfgGet.maskPol == gpioTriggerCfgSet.maskPol);
}

/**
 * @brief Test all GPIO pins trigger configuration
 */
void test_pos_fsm_fsmSetGpioTriggerCfg_allGpioPins(void)
{
    int32_t status;

    for (uint8_t pin = PMIC_FSM_GPIO_PIN_MIN; pin <= PMIC_FSM_GPIO_PIN_MAX; pin++)
    {
        Pmic_FsmGpioTriggerCfg_t gpioTriggerCfgSet = {
            .validParams = PMIC_FSM_MASK_VALID,
            .pinNum = pin,
            .mask = false
        };

        status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfgSet);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    }
}

/**
 * @brief Test recovery counter threshold set and get
 */
void test_pos_fsm_fsmSetGetRecovCntThr_allValues(void)
{
    int32_t status;

    /* Test all valid threshold values (0-15) */
    for (uint8_t thr = 0; thr <= 15; thr++)
    {
        status = Pmic_fsmSetRecovCntThr(&pmicHandle, thr);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        uint8_t recovCntThr = 0;
        status = Pmic_fsmGetRecovCntThr(&pmicHandle, &recovCntThr);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(recovCntThr == thr);
    }
}

/**
 * @brief Test recovery counter get and clear
 */
void test_pos_fsm_fsmGetClrRecovCnt_readAndClear(void)
{
    int32_t status;
    uint8_t recovCnt = 0;

    /* Get current recovery counter */
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Clear recovery counter */
    status = Pmic_fsmClrRecovCnt(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify cleared */
    status = Pmic_fsmGetRecovCnt(&pmicHandle, &recovCnt);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(recovCnt == 0);
}

/**
 * @brief Test startup destination set and get
 */
void test_pos_fsm_fsmSetGetStartupDest_allDestinations(void)
{
    int32_t status;

    /* Test all valid startup destinations */
    for (uint8_t dest = PMIC_FSM_STARTUP_DEST_MCU_ONLY; dest <= PMIC_FSM_START_UP_DEST_MAX; dest++)
    {
        status = Pmic_fsmSetStartupDest(&pmicHandle, dest);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

        uint8_t destination = 0;
        status = Pmic_fsmGetStartupDest(&pmicHandle, &destination);
        PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
        PLATFORM_ASSERT(destination == dest);
    }
}

/**
 * @brief Test soft reboot request
 */
void test_pos_fsm_fsmSendSoftRebootReq_validRequest(void)
{
    int32_t status;

    /* Note: This may trigger actual reboot on hardware, so only verify API call succeeds */
    status = Pmic_fsmSendSoftRebootReq(&pmicHandle);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test combined trigger configuration
 */
void test_pos_fsm_fsmSetGetTriggerCfg_combinedTriggers(void)
{
    int32_t status;

    /* Configure multiple triggers at once */
    Pmic_FsmTriggerCfg_t triggerCfgSet = {
        .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID |
                       PMIC_FSM_MCU_RAIL_TRIG_VALID,
        .severeErrTrig = PMIC_FSM_TRIGGER_IMMEDIATE_SHUTDOWN,
        .mcuRailTrig = PMIC_FSM_TRIGGER_MCU_POWER_ERROR
    };

    status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Read back and verify */
    Pmic_FsmTriggerCfg_t triggerCfgGet = {
        .validParams = PMIC_FSM_SEVERE_ERR_TRIG_VALID |
                       PMIC_FSM_MCU_RAIL_TRIG_VALID
    };

    status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(triggerCfgGet.severeErrTrig == triggerCfgSet.severeErrTrig);
    PLATFORM_ASSERT(triggerCfgGet.mcuRailTrig == triggerCfgSet.mcuRailTrig);
}

/**
 * @brief Test GPIO trigger polarity
 */
void test_pos_fsm_fsmSetGpioTriggerCfg_polarity(void)
{
    int32_t status;

    /* Test high polarity */
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {
        .validParams = PMIC_FSM_MASK_POL_VALID,
        .pinNum = PMIC_FSM_GPIO_PIN2,
        .maskPol = PMIC_FSM_GPIO_MASK_POL_1
    };

    status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    /* Test low polarity */
    gpioTriggerCfg.maskPol = PMIC_FSM_GPIO_MASK_POL_0;
    status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
}

/**
 * @brief Test negative case: otherRailTrig exceeds max value
 */
void test_neg_fsm_fsmotherRailTrig_exceeds_max(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = PMIC_FSM_OTHER_RAIL_TRIG_VALID,
        .otherRailTrig = PMIC_FSM_TRIGGER_MAX + 1
    };
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: socRailTrig exceeds max value
 */
void test_neg_fsm_fsmsocRailTrig_exceeds_max(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = PMIC_FSM_SOC_RAIL_TRIG_VALID,
        .socRailTrig = PMIC_FSM_TRIGGER_MAX + 1
    };
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: mcuRailTrig exceeds max value
 */
void test_neg_fsm_fsmmcuRailTrig_exceeds_max(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = PMIC_FSM_MCU_RAIL_TRIG_VALID,
        .mcuRailTrig = PMIC_FSM_TRIGGER_MAX + 1
    };
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: moderateErrTrig exceeds max value
 */
void test_neg_fsm_fsmmoderateErrTrig_exceeds_max(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = PMIC_FSM_MODERATE_ERR_TRIG_VALID,
        .moderateErrTrig = PMIC_FSM_TRIGGER_MAX + 1
    };
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: setTriggerCfg with zero validParams
 */
void test_neg_fsm_fsmsetTriggerCfg_zero_valid_params(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = 0U,
        .severeErrTrig = PMIC_FSM_TRIGGER_MCU_POWER_ERROR
    };
    int32_t status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: getTriggerCfg with zero validParams
 */
void test_neg_fsm_fsmgetTriggerCfg_zero_valid_params(void)
{
    Pmic_FsmTriggerCfg_t triggerCfg = {
        .validParams = 0U
    };
    int32_t status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: setGpioTriggerCfg with zero validParams
 */
void test_neg_fsm_fsmsetGpioTriggerCfg_zero_valid_params(void)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {
        .validParams = 0U,
        .pinNum = PMIC_FSM_GPIO_PIN1,
        .mask = true
    };
    int32_t status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test negative case: getGpioTriggerCfg with zero validParams
 */
void test_neg_fsm_fsmgetGpioTriggerCfg_zero_valid_params(void)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {
        .validParams = 0U,
        .pinNum = PMIC_FSM_GPIO_PIN1
    };
    int32_t status = Pmic_fsmGetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/**
 * @brief Test positive case: setTriggerCfg with OTHER_RAIL_TRIG_VALID
 */
void test_pos_fsm_fsmSetGetTriggerCfg_otherRailTrig(void)
{
    int32_t status;
    Pmic_FsmTriggerCfg_t triggerCfgSet = {
        .validParams = PMIC_FSM_OTHER_RAIL_TRIG_VALID,
        .otherRailTrig = PMIC_FSM_TRIGGER_ORDERLY_SHUTDOWN
    };

    status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_FsmTriggerCfg_t triggerCfgGet = {
        .validParams = PMIC_FSM_OTHER_RAIL_TRIG_VALID
    };

    status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(triggerCfgGet.otherRailTrig == triggerCfgSet.otherRailTrig);
}

/**
 * @brief Test positive case: setTriggerCfg with SOC_RAIL_TRIG_VALID
 */
void test_pos_fsm_fsmSetGetTriggerCfg_socRailTrig(void)
{
    int32_t status;
    Pmic_FsmTriggerCfg_t triggerCfgSet = {
        .validParams = PMIC_FSM_SOC_RAIL_TRIG_VALID,
        .socRailTrig = PMIC_FSM_TRIGGER_IMMEDIATE_SHUTDOWN
    };

    status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_FsmTriggerCfg_t triggerCfgGet = {
        .validParams = PMIC_FSM_SOC_RAIL_TRIG_VALID
    };

    status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(triggerCfgGet.socRailTrig == triggerCfgSet.socRailTrig);
}

/**
 * @brief Test positive case: setTriggerCfg with MCU_RAIL_TRIG_VALID
 */
void test_pos_fsm_fsmSetGetTriggerCfg_mcuRailTrig(void)
{
    int32_t status;
    Pmic_FsmTriggerCfg_t triggerCfgSet = {
        .validParams = PMIC_FSM_MCU_RAIL_TRIG_VALID,
        .mcuRailTrig = PMIC_FSM_TRIGGER_SOC_POWER_ERROR
    };

    status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_FsmTriggerCfg_t triggerCfgGet = {
        .validParams = PMIC_FSM_MCU_RAIL_TRIG_VALID
    };

    status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(triggerCfgGet.mcuRailTrig == triggerCfgSet.mcuRailTrig);
}

/**
 * @brief Test positive case: setTriggerCfg with MODERATE_ERR_TRIG_VALID only
 */
void test_pos_fsm_fsmSetGetTriggerCfg_moderateErrTrig(void)
{
    int32_t status;
    Pmic_FsmTriggerCfg_t triggerCfgSet = {
        .validParams = PMIC_FSM_MODERATE_ERR_TRIG_VALID,
        .moderateErrTrig = PMIC_FSM_TRIGGER_MCU_POWER_ERROR
    };

    status = Pmic_fsmSetTriggerCfg(&pmicHandle, &triggerCfgSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    Pmic_FsmTriggerCfg_t triggerCfgGet = {
        .validParams = PMIC_FSM_MODERATE_ERR_TRIG_VALID
    };

    status = Pmic_fsmGetTriggerCfg(&pmicHandle, &triggerCfgGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(triggerCfgGet.moderateErrTrig == triggerCfgSet.moderateErrTrig);
}

/**
 * @brief Test negative case: setStartupDest with invalid startup destination
 */
void test_neg_fsm_fsmSetStartupDest_invalidState(void)
{
    int32_t status;

    /* Test with destination value beyond max */
    status = Pmic_fsmSetStartupDest(&pmicHandle, PMIC_FSM_START_UP_DEST_MAX + 1);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);

    /* API only validates upper bound (> MAX), not gap values */
    /* Valid enum values are 0, 2, 3 but API accepts 1 as well */
}

/**
 * @brief Test positive case: getStartupDest reads current destination correctly
 */
void test_pos_fsm_fsmGetStartupDest_validRead(void)
{
    int32_t status;
    uint8_t destinationSet, destinationGet;

    /* Test reading after setting STANDBY destination */
    destinationSet = PMIC_FSM_STARTUP_DEST_STANDBY;
    status = Pmic_fsmSetStartupDest(&pmicHandle, destinationSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetStartupDest(&pmicHandle, &destinationGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(destinationGet == destinationSet);

    /* Test reading after setting MCU_ONLY destination */
    destinationSet = PMIC_FSM_STARTUP_DEST_MCU_ONLY;
    status = Pmic_fsmSetStartupDest(&pmicHandle, destinationSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetStartupDest(&pmicHandle, &destinationGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(destinationGet == destinationSet);

    /* Test reading after setting ACTIVE destination */
    destinationSet = PMIC_FSM_STARTUP_DEST_ACTIVE;
    status = Pmic_fsmSetStartupDest(&pmicHandle, destinationSet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);

    status = Pmic_fsmGetStartupDest(&pmicHandle, &destinationGet);
    PLATFORM_ASSERT(status == PMIC_ST_SUCCESS);
    PLATFORM_ASSERT(destinationGet == destinationSet);
}

/**
 * @brief Test Pmic_fsmSetGpioTriggerCfg with invalid maskPol value
 * Covers line 373 in pmic_fsm.c
 */
void test_neg_fsm_fsmsetGpioTrigger_invalidMaskPol(void)
{
    Pmic_FsmGpioTriggerCfg_t gpioTriggerCfg = {
        .validParams = PMIC_FSM_MASK_POL_VALID,
        .pinNum = PMIC_FSM_GPIO_PIN1,
        .maskPol = 2U  /* Invalid, max is 1 */
    };

    int32_t status = Pmic_fsmSetGpioTriggerCfg(&pmicHandle, &gpioTriggerCfg);
    PLATFORM_ASSERT(status == PMIC_ST_ERR_INV_PARAM);
}

/* ========================================================================== */
/*                         Entry Point Function                               */
/* ========================================================================== */

void fsm_test(void *args)
{
    (void)args;
    int32_t status;

    platform_init();
    testTimer_startModule("FSM");
    platform_setupTests();

    /* Initialize PMIC handle */
    Pmic_HandleCfg_t handleCfg = {
        .validParams = PMIC_COMM_MODE_VALID |
                       PMIC_I2C_ADDR0_VALID |
                       PMIC_COMM_HANDLE_0_VALID |
                       PMIC_IO_READ_VALID |
                       PMIC_IO_WRITE_VALID |
                       PMIC_CRITICAL_SECTION_START_VALID |
                       PMIC_CRITICAL_SECTION_STOP_VALID,
        .commMode = PMIC_INTF_I2C_SINGLE,
        .i2cAddr0 = PLATFORM_TARGET_I2C_ADDR,
        .commHandle0 = platform_getCommHandle0(),
        .ioRead = &platform_rxByte,
        .ioWrite = &platform_txByte,
        .criticalSectionStart = &platform_critSecStart,
        .criticalSectionStop = &platform_critSecStop
    };

    status = Pmic_init(&pmicHandle, &handleCfg);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to initialize PMIC handle\r\n");
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    /* Unlock registers for FSM trigger configuration */
    status = Pmic_setRegLockState(&pmicHandle, false);
    if (status != PMIC_ST_SUCCESS)
    {
        platform_printString("\r\nERROR: Failed to unlock registers\r\n");
        Pmic_deinit(&pmicHandle);
        platform_tearDownTests();
        platform_deinit();
        return;
    }

    platform_printString("\r\n=== FSM Module Tests ===\r\n");
    FSM_TEST_RUN_ALL();

    testTimer_endModule();
    Pmic_deinit(&pmicHandle);
    platform_tearDownTests();
    platform_deinit();
}
