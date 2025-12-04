#include "pmic_wdg_qa_example.h"

#define NUM_SEQUENCES (100U)

#define REPORT_ERROR(STATUS, MSG) \
({ \
    char _buffer[100U]; \
    (void)sprintf(_buffer, "Function Name: %s\r\nLine Number: %d\r\nError Code: %d\r\nMessage: %s\r\n", __func__, __LINE__, (int32_t)STATUS, (char *)MSG); \
    platform_printString(_buffer); \
    (int32_t)STATUS; \
})

static int32_t app_clearPmicIrqFlags(Pmic_Handle_t *pmicHandle);
static bool app_wdgCfgCheck(Pmic_WdgCfg_t *actWdgCfg, Pmic_WdgCfg_t *expWdgCfg);

int run_pmic_wdg_qa_example(void) {
    bool isLocked = (bool)false;
	Pmic_Handle_t pmicHandle = {0U};
	const Pmic_HandleCfg_t pmicHandleCfg = {
		.validParams = PMIC_SYNC_OPERATION_VALID,
		.crcEnable = (bool)false,
		.commHandle0 = platform_getCommHandle(),
		.ioRead = &platform_rxByte,
		.ioWrite = &platform_txByte,
		.criticalSectionStart = &platform_critSecStart,
		.criticalSectionStop = &platform_critSecStop
	};
    Pmic_WdgCfg_t expWdgCfg = {
        .validParams = PMIC_WDG_CFG_VALID_ALL,
        .rstEn = (bool)true,
        .failThr = PMIC_WDG_THR_CNT_MAX,
        .rstThr = PMIC_WDG_THR_CNT_MAX,
        .win1Code = PMIC_WDG_WIN_CODE_MAX,
        .win2Code = PMIC_WDG_WIN_CODE_MAX,
        .longWinCode = PMIC_WDG_LONG_WIN_CODE_MAX,
        .qaFdbk = 0U,
        .qaLfsr = 1U,
        .qaSeed = 2U
    };
    Pmic_WdgCfg_t actWdgCfg = {.validParams = PMIC_WDG_CFG_VALID_ALL};
    Pmic_WdgErrStatus_t wdgErrStatus = {.validParams = PMIC_WDG_ERR_STATUS_VALID_ALL};
    Pmic_WdgFailCntStatus_t wdgFailCntStatus = {.validParams = PMIC_WDG_FAIL_CNT_STATUS_VALID_ALL};

    // Initialize platform
	platform_init();

    platform_printString("Running PMIC Watchdog QA example...\r\n");

    // Initialize PMIC LLD
	int32_t status = Pmic_init(&pmicHandle, &pmicHandleCfg);
	if (status != PMIC_ST_SUCCESS) {
		return REPORT_ERROR(status, "Failed to initialize PMIC LLD.\r\n");
	}

    // Unlock PMIC user-space registers
	status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK_DISABLE);
	if (status != PMIC_ST_SUCCESS) {
		return REPORT_ERROR(status, "Failed to unlock PMIC user-space registers.\r\n");
	}

    // Disable configuration register CRC
    status = Pmic_ioUpdateByte(&pmicHandle, 0x62U, 0U, 1U << 0U, 0U);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to update PMIC register.\r\n");
    }

    // Validate PMIC register lock state
    status = Pmic_getRegLockState(&pmicHandle, &isLocked);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to get PMIC register lock state.\r\n");
    } else {
        if (isLocked) {
            return REPORT_ERROR(-1, "PMIC user-space registers are still locked.\r\n");
        }
    }

    // Clear PMIC interrupt flags
    status = app_clearPmicIrqFlags(&pmicHandle);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to clear PMIC interrupt flags\r\n");
    }

    // Clear watchdog statuses
    status = Pmic_wdgClrErrStatusAll(&pmicHandle);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to clear all watchdog statuses\r\n");
    }

    // Enable watchdog before configuring it
    status = Pmic_wdgSetEnableState(&pmicHandle, PMIC_ENABLE);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to enable watchdog\r\n");
    }

    // Enable Power Hold so that watchdog stays in Long Window
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to enable watchdog Power Hold\r\n");
    }

    // Enable Return to Long Window so that watchdog can be configured
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to enable watchdog Return to Long Window\r\n");
    }

    // Set expected watchdog configurations
    status = Pmic_wdgSetCfg(&pmicHandle, &expWdgCfg);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to set expected watchdog configurations\r\n");
    }

    // Get actual watchdog configurations
    status = Pmic_wdgGetCfg(&pmicHandle, &actWdgCfg);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to get actual watchdog configurations\r\n");
    }

    // Compare actual vs. expected configurations
    if (app_wdgCfgCheck(&actWdgCfg, &expWdgCfg) == (bool)false) {
        return REPORT_ERROR(-1, "Actual and expected watchdog configurations do not match\r\n");
    }

    // Disable watchdog Power Hold once configurations have been set
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_DISABLE);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to disable watchdog Power Hold\r\n");
    }

    // Disable watchdog Return to Long Window once configurations have been set
    status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_DISABLE);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to disable watchdog Return to Long Window\r\n");
    }

    // Send four answer bytes to exit Long Window
    for (uint8_t i = 0U; i < 4U; i++) {
        status = Pmic_wdgQaWriteAnswer(&pmicHandle);
        if (status != PMIC_ST_SUCCESS) {
            return REPORT_ERROR(status, "Failed to send watchdog answer byte\r\n");
        }
    }

    // Check for any watchdog errors that could have occured when sending answer bytes to exit Long Window
    status = Pmic_wdgGetErrStatus(&pmicHandle, &wdgErrStatus);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to get watchdog error status\r\n");
    } else {
        if (wdgErrStatus.rstInt || wdgErrStatus.failInt || wdgErrStatus.answErr || wdgErrStatus.seqErr ||
            wdgErrStatus.answEarlyErr || wdgErrStatus.timeoutErr || wdgErrStatus.longWinTimeoutInt) {
            return REPORT_ERROR(-1, "Watchdog error status indicates a problem\r\n");
        }
    }

    // Iterate for 100 sequences
    for (uint8_t i = 0U; i < NUM_SEQUENCES; i++) {
        // If on last sequence, enable return to Long Window
        if (i == (NUM_SEQUENCES - 1U)) {
            status = Pmic_wdgSetReturnToLongWindow(&pmicHandle, PMIC_ENABLE);
            if (status != PMIC_ST_SUCCESS) {
                return REPORT_ERROR(status, "Failed to enable return to Long Window\r\n");
            }
        }

        // Send 3 answers in Window-1
        for (uint8_t j = 0U; j < 3U; j++) {
            status = Pmic_wdgQaWriteAnswer(&pmicHandle);
            if (status != PMIC_ST_SUCCESS) {
                return REPORT_ERROR(status, "Failed to send watchdog answer byte\r\n");
            }
        }

        // Wait for Window-1 to elapse
        platform_timerWaitMs(71U);

        // Check watchdog fail counter statuses
        status = Pmic_wdgGetFailCntStatus(&pmicHandle, &wdgFailCntStatus);
        if (status != PMIC_ST_SUCCESS) {
            return REPORT_ERROR(status, "Failed to get watchdog fail counter status\r\n");
        } else {
            if (wdgFailCntStatus.badEvent || (wdgFailCntStatus.failCnt != 0U)) {
                return REPORT_ERROR(-1, "Watchdog fail counter indicates a problem\r\n");
            }
        }

        // Send the last answer byte of the sequence in Window-2
        status = Pmic_wdgQaWriteAnswer(&pmicHandle);
        if (status != PMIC_ST_SUCCESS) {
            return REPORT_ERROR(status, "Failed to send watchdog answer byte\r\n");
        }

        // End of watchdog sequence; next one will begin. Check watchdog error statuses
        status = Pmic_wdgGetErrStatus(&pmicHandle, &wdgErrStatus);
        if (status != PMIC_ST_SUCCESS) {
            return REPORT_ERROR(status, "Failed to get watchdog error status\r\n");
        } else {
            if (wdgErrStatus.rstInt || wdgErrStatus.failInt || wdgErrStatus.answErr || wdgErrStatus.seqErr ||
                wdgErrStatus.answEarlyErr || wdgErrStatus.timeoutErr || wdgErrStatus.longWinTimeoutInt) {
                return REPORT_ERROR(-1, "Watchdog error status indicates a problem\r\n");
            }
        }
    }

    // Enable Power Hold in Long Window
    status = Pmic_wdgSetPowerHold(&pmicHandle, PMIC_ENABLE);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to enable watchdog Power Hold\r\n");
    }

    // De-initialize PMIC LLD
    status = Pmic_deinit(&pmicHandle);
    if (status != PMIC_ST_SUCCESS) {
        return REPORT_ERROR(status, "Failed to de-initialize PMIC LLD\r\n");
    }

    platform_printString("\tPMIC Watchdog QA example successful!\r\n");

	return 0U;
}

static int32_t app_clearPmicIrqFlags(Pmic_Handle_t *pmicHandle) {
    int32_t status = PMIC_ST_SUCCESS;

    // Clearable LP8772x interrupt registers are from 0x48U to 0x53U.
    // The interrupts in each register must be written with 1 to clear
    for (uint8_t reg = 0x48U; reg <= 0x53U; reg++) {
        status = Pmic_ioTxByte(pmicHandle, reg, 0xFFU);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static bool app_wdgCfgCheck(Pmic_WdgCfg_t *actWdgCfg, Pmic_WdgCfg_t *expWdgCfg) {
    return (
        ((actWdgCfg->rstEn == expWdgCfg->rstEn) && (actWdgCfg->failThr == expWdgCfg->failThr) &&
        (actWdgCfg->rstThr == expWdgCfg->rstThr) && (actWdgCfg->longWinCode == expWdgCfg->longWinCode) &&
        (actWdgCfg->win1Code == expWdgCfg->win1Code) && (actWdgCfg->win2Code == expWdgCfg->win2Code) &&
        (actWdgCfg->qaFdbk == expWdgCfg->qaFdbk) && (actWdgCfg->qaLfsr == expWdgCfg->qaLfsr) &&
        (actWdgCfg->qaSeed == expWdgCfg->qaSeed))
    );
}
