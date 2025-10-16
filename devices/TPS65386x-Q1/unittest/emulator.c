#include <criterion/logging.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "emulator.h"
#include "pmic.h"
#include "pmic_core.h"
#include "regmap/core.h"

// The maximum number of registers supported on this device
#define MAX_REGS_SUPPORTED (0xB5)

// Useful macros for re-use
#define IN_RANGE(var, low, hi)  ((var >= low) && (var <= hi))
#define LEN(arr)                (sizeof(arr) / sizeof(arr[0]))

// The register size for this device in bits, represented as a stdint type.
typedef uint8_t tRegSize;

struct sLockTrack {
    uint8_t lastCfgUnlockByte;
    uint8_t lastCntUnlockByte;
};

struct sTestFramework {
    struct Pmic_CoreHandle_s *pmicCoreHandle;
    unsigned int mutexCounter;
    struct sLockTrack locking;

    tRegSize regMap[MAX_REGS_SUPPORTED];
};

// Instantiate one global state tracker
struct sTestFramework Emulator;

void TestFrameworkInit(tTestFramework *framework, struct Pmic_CoreHandle_s *pmicCoreHandle) {
    // Begin with the registers locked
    framework->regMap[REG_STAT_REG] = 0x03;

    framework->pmicCoreHandle = pmicCoreHandle;
}

void TestFrameworkReset(tTestFramework *framework) {
    memset(framework->regMap, 0, sizeof(LEN(framework->regMap)));
}

void PrintRegisterMap(void) {
    for (uint8_t i = 0; i <= MAX_REGS_SUPPORTED; i++) {
        printf("REG[0x%X]=0x%X\n", i, Emulator.regMap[i]);
    }
}

void CommunicationHandleInit(tCommunicationHandle *handle, bool isQAComm)
{
    handle->id = isQAComm ? 1 : 2;
}

static bool CfgRegLockedRegister(uint16_t regAddr) {
    uint16_t ranges[][2] = {
        { 0x5, 0x8 },
        { 0xC, 0xE },
        { 0x12, 0x16 },
        { 0x19, 0x43 },
        { 0x47, 0x50 },
        { 0x52, 0x52 },
        { 0x54, 0x57 },
        { 0x60, 0x61 },
        { 0x6A, 0x6D },
        { 0x71, 0x88 },
        { 0x91, 0xAC },
        { 0xAF, 0xB2 },
        { 0xB4, 0xB4 },
    };

    for (unsigned int row = 0; row < LEN(ranges); row++) {
        if (IN_RANGE(regAddr, ranges[row][0], ranges[row][1])) {
            return true;
        }
    }

    return false;
}

static bool CntRegLockedRegister(uint16_t regAddr) {
    if (IN_RANGE(regAddr, 0x8B, 0x8F)) {
        return true;
    }

    return false;
}

bool RegisterIsLocked(tTestFramework *framework, uint16_t regAddr) {
    Pmic_Lock_t lockStatus = { .validParams = PMIC_CFG_LOCK_ALL_VALID_SHIFT };
    Pmic_getLockCfg(framework->pmicCoreHandle, &lockStatus);

    if (!lockStatus.cfgLock && !lockStatus.cntLock) {
        return false;
    }

    const bool isCfgReg = CfgRegLockedRegister(regAddr);
    const bool isCntReg = CntRegLockedRegister(regAddr);

    if (isCfgReg && lockStatus.cfgLock) {
        return true;
    }

    if (isCntReg && lockStatus.cntLock) {
        return true;
    }

    return false;
}

int32_t PmicCommIoRead(struct Pmic_CoreHandle_s *pmicCorehandle,
                       uint8_t instType,
                       uint16_t regAddr,
                       uint8_t *pRxBuf,
                       uint8_t bufLen)
{
    tRegSize ioBuf[4] = {0, 0, Emulator.regMap[regAddr], 0};

    // don't care about instType at the moment, this just becomes a simple
    // memcpy, we create an intermediate buffer with the data in the third byte
    // because this function is called *prior* to going through the PMIC
    // protocol layer which handles page addressing and CRC
    memcpy(pRxBuf, &ioBuf, bufLen);

    return PMIC_ST_SUCCESS;
}

int32_t PmicCommIoWrite(struct Pmic_CoreHandle_s *pmicCorehandle,
                        uint8_t instType,
                        uint16_t regAddr,
                        uint8_t *pTxBuf,
                        uint8_t bufLen)
{
    uint8_t dataByte;

    // extract the data byte we extract the third byte (offset 2) because this
    // function is called *after* going through the PMIC protocol layer which
    // handles page addressing and CRC
    memcpy(&dataByte, pTxBuf + 2, 1);

    // Check for writes to the register locking sequence registers and
    // intercept them
    if (regAddr == CFG_REG_UNLOCK_SEQ_REG) {
        if ((Emulator.locking.lastCfgUnlockByte == 0x98) && (dataByte == 0xB8)) {
            // if we previously recieved the first unlock byte, and just got
            // the next one, the registers are now unlocked
            Emulator.regMap[REG_STAT_REG] &= ~0x01;
        } else if ((Emulator.regMap[REG_STAT_REG] & 0x1) == 0x0) {
            // if we got literally any other data and the registers are
            // currently unlocked, relock them
            Emulator.regMap[REG_STAT_REG] |= 0x01;
        }

        // set the lastCfgUnlockByte, then exit before storing to memory
        Emulator.locking.lastCfgUnlockByte = dataByte;
        return PMIC_ST_SUCCESS;
    } else if (regAddr == CNT_REG_UNLOCK_SEQ_REG) {
        if ((Emulator.locking.lastCntUnlockByte == 0x13) && (dataByte == 0x7D)) {
            // if we previously recieved the first unlock byte, and just got
            // the next one, the registers are now unlocked
            Emulator.regMap[REG_STAT_REG] &= ~0x02;
        } else if ((Emulator.regMap[REG_STAT_REG] & 0x2) == 0x0) {
            // if we got literally any other data and the registers are
            // currently unlocked, relock them
            Emulator.regMap[REG_STAT_REG] |= 0x02;
        }

        // set the lastCntUnlockByte, then exit before storing to memory
        Emulator.locking.lastCntUnlockByte = dataByte;
        return PMIC_ST_SUCCESS;
    }

    // Check for writes to locked registers, and disallow them by exiting early
    if (RegisterIsLocked(&Emulator, regAddr)) {
        return PMIC_ST_SUCCESS;
    }

    // Copy into the memory buffer of the emulator
    memcpy(&Emulator.regMap[regAddr], &dataByte, 1);
    return PMIC_ST_SUCCESS;
}

void CritSecStart(void)
{
    Emulator.mutexCounter += 1;
    cr_log_info("mutex=%u\n", Emulator.mutexCounter);
}

void CritSecStop(void)
{
    // only decrement if the mutex is currently owned by at least one claim,
    // releasing a claim on an onowned mutex is never a problem
    if (Emulator.mutexCounter > 0) {
        Emulator.mutexCounter -= 1;
    }

    cr_log_info("mutex=%u\n", Emulator.mutexCounter);
}
