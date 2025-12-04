#include "pmic_init_example.h"

int run_pmic_init_example(void) {
	bool isLocked = (bool)false;
	uint8_t initVal = 0U, newVal = 0U, actVal = 0U;
	Pmic_Handle_t pmicHandle = {0U};
	Pmic_HandleCfg_t pmicHandleCfg = {
		.validParams = PMIC_SYNC_OPERATION_VALID,
		.crcEnable = (bool)false,
		.commHandle0 = platform_getCommHandle(),
		.ioRead = &platform_rxByte,
		.ioWrite = &platform_txByte,
		.criticalSectionStart = &platform_critSecStart,
		.criticalSectionStop = &platform_critSecStop
	};

	platform_init();

	platform_printString("--------------------------------------------------------------------------\r\n");
	platform_printString("Platform has been initialized.\r\n");

	platform_printString("Initializing PMIC LLD...\r\n");
	int32_t status = Pmic_init(&pmicHandle, &pmicHandleCfg);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tPMIC LLD initialization failed\r\n");
		return status;
	} else {
		platform_printString("\tPMIC LLD initialization succeeded\r\n");
	}

	platform_printString("Unlocking PMIC registers...\r\n");
	status = Pmic_setRegLockState(&pmicHandle, PMIC_LOCK_DISABLE);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tFailed attempt to unlock PMIC registers\r\n");
		return status;
	} else {
		platform_printString("\tPMIC register unlock succeeded\r\n");
	}

	platform_printString("Validating PMIC register lock state...\r\n");
	status = Pmic_getRegLockState(&pmicHandle, &isLocked);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tFailed to get PMIC register lock state\r\n");
		return status;
	}

	if (isLocked) {
		platform_printString("\tError: PMIC registers are locked\r\n");
		return -1;
	} else {
		platform_printString("\tPMIC registers are unlocked\r\n");
	}

	platform_printString("Performing read-modify-write test on Scratchpad register 1...\r\n");
	status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, &initVal);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tFailed to get initial value of PMIC scratchpad register 1\r\n");
		return status;
	}

	newVal = ~initVal;
	status = Pmic_setScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, newVal);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tFailed to set expected value of PMIC scratchpad register 1\r\n");
		return status;
	}

	status = Pmic_getScratchPadValue(&pmicHandle, PMIC_SCRATCH_PAD_REG_1, &actVal);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tFailed to get actual value of PMIC scratchpad register 1\r\n");
		return status;
	}

	char msg[100U] = {0U};
	(void)sprintf(msg, "\tPMIC scratchpad register 1: initVal=0x%02X, newVal=0x%02X, actVal=0x%02X\r\n", initVal, newVal, actVal);
	platform_printString(msg);
	if ((actVal == initVal) || (actVal != newVal)) {
		platform_printString("\tPMIC scratchpad register 1 test failed\r\n");
		return -1;
	} else {
		platform_printString("\tPMIC scratchpad register 1 test succeeded\r\n");
	}

	platform_printString("De-initializing PMIC LLD instance...\r\n");
	status = Pmic_deinit(&pmicHandle);
	if (status != PMIC_ST_SUCCESS) {
		platform_printString("\tPMIC LLD de-initialization failed\r\n");
		return status;
	} else {
		platform_printString("\tPMIC LLD de-initialization succeeded\r\n");
	}
    platform_printString("--------------------------------------------------------------------------\r\n");

	return 0U;
}
