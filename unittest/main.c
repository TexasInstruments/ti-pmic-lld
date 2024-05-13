#include <criterion/criterion.h>
#include <criterion/new/assert.h>

#include <stdint.h>
#include <stddef.h>

#include "emulator.h"

#include "pmic.h"
#include "pmic_io.h"
#include "pmic_core.h"
#include "regmap/core.h"

Pmic_CoreHandle_t PmicHandle;

// A helper function used as a setup step for all other tests to ensure we have
// a valid handle created
int32_t PmicHandleInit(void) {
    // if the handle has already been initialized, exit early
    if (PmicHandle.drvInitStatus != 0) {
        return PMIC_ST_SUCCESS;
    }

    // Create communications handles which are used in the CommIoRead/Write
    // functions
    tCommunicationHandle commHandle = {};

    // Initialize communications handles with transport specific details
    CommunicationHandleInit(&commHandle, false);

    Pmic_CoreCfg_t coreCfg = {
        .validParams = (
            PMIC_CFG_DEVICE_TYPE_VALID_SHIFT   |
            PMIC_CFG_COMM_MODE_VALID_SHIFT     |
            PMIC_CFG_COMM_HANDLE_VALID_SHIFT   |
            PMIC_CFG_COMM_IO_RD_VALID_SHIFT    |
            PMIC_CFG_COMM_IO_WR_VALID_SHIFT    |
            PMIC_CFG_CRITSEC_START_VALID_SHIFT |
            PMIC_CFG_CRITSEC_STOP_VALID_SHIFT
        ),
        .instType = (PMIC_MAIN_INST | PMIC_QA_INST),
        .pmicDeviceType = PMIC_DEV_BB_TPS65386X,
        .commMode = PMIC_INTF_SPI,
        .pCommHandle = &commHandle,
        .pFnPmicCommIoRead = PmicCommIoRead,
        .pFnPmicCommIoWrite = PmicCommIoWrite,
        .pFnPmicCritSecStart = CritSecStart,
        .pFnPmicCritSecStop = CritSecStop,
    };

    return Pmic_init(&PmicHandle, &coreCfg);
}

void PmicHandleInitSetup(void) {
    PmicHandleInit();
}

Test(core, init) {
    int32_t status = PmicHandleInit();
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
}

Test(core, write_and_read, .init = PmicHandleInitSetup) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regDataInitial = 0;
    uint8_t regData = 0;

    const uint8_t testPattern = 0x55;

    // Read the initial value of the register so we can set it back when done.
    status = Pmic_commIntf_recvByte(&PmicHandle, PMIC_CUSTOMER_SCRATCH1_REGADDR, &regDataInitial);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    // Write to the customer scratch 1 register, then read it back to confirm
    // transport layer.
    status = Pmic_commIntf_sendByte(&PmicHandle, PMIC_CUSTOMER_SCRATCH1_REGADDR, testPattern);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    status = Pmic_commIntf_recvByte(&PmicHandle, PMIC_CUSTOMER_SCRATCH1_REGADDR, &regData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    cr_assert(eq(u8, regData, testPattern));

    // Now set the register back to what it was originally
    status = Pmic_commIntf_sendByte(&PmicHandle, PMIC_CUSTOMER_SCRATCH1_REGADDR, regDataInitial);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
}

Test(core, mutex_obtain_release, .init = PmicHandleInitSetup) {
    // Nothing really for us to check here, just ensure no crash.
    Pmic_criticalSectionStart(&PmicHandle);
    Pmic_criticalSectionStop(&PmicHandle);
}

Test(core, mutex_obtain_release_corrupted, .init = PmicHandleInitSetup) {
    // Corrupt the function pointers for the critical section functions and
    // ensure no crash.
    PmicHandle.pFnPmicCritSecStart = NULL;
    PmicHandle.pFnPmicCritSecStop = NULL;

    Pmic_criticalSectionStart(&PmicHandle);
    Pmic_criticalSectionStop(&PmicHandle);

    // Deinit the handle so that it will be reinitialized by the next test that needs it
    Pmic_deinit(&PmicHandle);
}