#include <criterion/criterion.h>
#include <criterion/new/assert.h>

#include <stdint.h>
#include <stddef.h>

#include "emulator.h"

#include "pmic.h"
#include "regmap/core.h"

extern struct sTestFramework Emulator;
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
        .instType = PMIC_MAIN_INST,
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
    TestFrameworkInit(&Emulator, &PmicHandle);
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

Test(core, locking_reg_lock_starts_locked, .init = PmicHandleInitSetup) {
    int32_t status = PMIC_ST_SUCCESS;
    const uint16_t NVM_REV_REG = 0xB0;

    // Attempt a write to NVM_REV_REG and read back, the write should not do
    // anything as this register is locked by default (CFG_REG_LOCK)
    uint8_t startData = 0;
    uint8_t endData = 0;

    status = Pmic_commIntf_recvByte(&PmicHandle, NVM_REV_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    status = Pmic_commIntf_sendByte(&PmicHandle, NVM_REV_REG, ~startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    status = Pmic_commIntf_recvByte(&PmicHandle, NVM_REV_REG, &endData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    cr_assert(eq(u8, startData, endData));
}

void TestRegisterIsLocked(uint16_t regAddr, uint8_t startData) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t endData = 0;

    // Write and then read-back from the register and confirm no change
    status = Pmic_commIntf_sendByte(&PmicHandle, regAddr, ~startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    status = Pmic_commIntf_recvByte(&PmicHandle, regAddr, &endData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    cr_assert(eq(u8, startData, endData));
}

void TestRegisterIsUnlocked(uint16_t regAddr, uint8_t startData) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t endData = 0;

    // Write and then read-back from the register and confirm the write took
    status = Pmic_commIntf_sendByte(&PmicHandle, regAddr, ~startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    status = Pmic_commIntf_recvByte(&PmicHandle, regAddr, &endData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    cr_assert(eq(u8, endData, ~startData));
}

void TestLockStatusDedicatedFn(uint16_t regAddr, int32_t (setLockState(struct Pmic_CoreHandle_s *, uint8_t state))) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t startData = 0;
    uint8_t endData = 0;

    status = Pmic_commIntf_recvByte(&PmicHandle, regAddr, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    // Unlock the area and then test that it can be changed
    status = setLockState(&PmicHandle, PMIC_LOCK_DISABLE);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsUnlocked(regAddr, startData);

    // Set the register back to what it started the test as
    status = Pmic_commIntf_sendByte(&PmicHandle, regAddr, startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    // Re-lock the area and then test that it can't be changed
    status = setLockState(&PmicHandle, PMIC_LOCK_ENABLE);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsLocked(regAddr, startData);
}

Test(core, locking_reg_lock_can_unlock_relock_dedicated, .init = PmicHandleInitSetup) {
    const uint16_t NVM_REV_REG = 0xB0;
    TestLockStatusDedicatedFn(NVM_REV_REG, Pmic_setRegLockState);
}

Test(core, locking_cnt_lock_can_unlock_relock_dedicated, .init = PmicHandleInitSetup) {
    const uint16_t RC_SIN0_REG = 0x8B;
    TestLockStatusDedicatedFn(RC_SIN0_REG, Pmic_setCntLockState);
}

Test(core, locking_control_through_struct, .init = PmicHandleInitSetup) {
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t startData = 0;

    const uint16_t NVM_REV_REG = 0xB0;  // For testing CFG_REG_LOCK
    const uint16_t RC_SIN0_REG = 0x8B;  // For testing CNT_REG_LOCK

    Pmic_Lock_t lockState = { .validParams = PMIC_CFG_LOCK_ALL_VALID_SHIFT };
    status = Pmic_getLockCfg(&PmicHandle, &lockState);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    // Expect both register locks to be reported as enabled
    cr_assert(eq(u8, lockState.cfgLock, 1));
    cr_assert(eq(u8, lockState.cntLock, 1));

    // Validate that each of the test registers cannot be modified
    status = Pmic_commIntf_recvByte(&PmicHandle, NVM_REV_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsLocked(NVM_REV_REG, startData);

    status = Pmic_commIntf_recvByte(&PmicHandle, RC_SIN0_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsLocked(RC_SIN0_REG, startData);

    // Use the configuration struct to unlock both register spaces
    lockState.cfgLock = 0;
    lockState.cntLock = 0;
    status = Pmic_setLockCfg(&PmicHandle, &lockState);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    // Validate that each of the test registers can now be modified
    status = Pmic_commIntf_recvByte(&PmicHandle, NVM_REV_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsUnlocked(NVM_REV_REG, startData);

    status = Pmic_commIntf_recvByte(&PmicHandle, RC_SIN0_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsUnlocked(RC_SIN0_REG, startData);

    // Ensure that the getCfg API now reports these registers as unlocked
    // (clearing previous memory as a precaution)
    memset(&lockState, 0, sizeof(Pmic_Lock_t));
    lockState.validParams = PMIC_CFG_LOCK_ALL_VALID_SHIFT;
    status = Pmic_getLockCfg(&PmicHandle, &lockState);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    cr_assert(eq(u8, lockState.cfgLock, 0));
    cr_assert(eq(u8, lockState.cntLock, 0));

    // Re-lock the registers and validate once more that they cannot be modified
    lockState.validParams = PMIC_CFG_LOCK_ALL_VALID_SHIFT;
    lockState.cfgLock = 1;
    lockState.cntLock = 1;
    status = Pmic_setLockCfg(&PmicHandle, &lockState);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));

    status = Pmic_commIntf_recvByte(&PmicHandle, NVM_REV_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsLocked(NVM_REV_REG, startData);

    status = Pmic_commIntf_recvByte(&PmicHandle, RC_SIN0_REG, &startData);
    cr_assert(eq(i32, status, PMIC_ST_SUCCESS));
    TestRegisterIsLocked(RC_SIN0_REG, startData);
}