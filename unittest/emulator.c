#include <criterion/logging.h>

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "emulator.h"
#include "pmic.h"

// The maximum number of registers supported on this device
#define MAX_REGS_SUPPORTED (0xB5)

// The register size for this device in bits, represented as a stdint type.
typedef uint8_t tRegSize;

typedef struct {
    tRegSize regMap[MAX_REGS_SUPPORTED];
    unsigned int mutexCounter;
} tEmulatorState;

// Instantiate one global state tracker
tEmulatorState Emulator;

void PrintRegisterMap(void) {
    for (uint8_t i = 0; i <= MAX_REGS_SUPPORTED; i++) {
        printf("REG[0x%X]=0x%X\n", i, Emulator.regMap[i]);
    }
}

void CommunicationHandleInit(tCommunicationHandle *handle, bool isQAComm)
{
    handle->id = isQAComm ? 1 : 2;
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
    // don't care about instType at the moment, this just becomes a simple
    // memcpy, we extract the third byte (offset 2) because this function is
    // called *after* going through the PMIC protocol layer which handles page
    // addressing and CRC
    memcpy(&Emulator.regMap[regAddr], pTxBuf + 2, 1);

    return PMIC_ST_SUCCESS;
}

void CritSecStart(void)
{
    Emulator.mutexCounter += 1;
    cr_log_info("mutex(claim  )=%u\n", Emulator.mutexCounter);
}

void CritSecStop(void)
{
    // only decrement if the mutex is currently owned by at least one claim,
    // releasing a claim on an onowned mutex is never a problem
    if (Emulator.mutexCounter > 0) {
        Emulator.mutexCounter -= 1;
    }

    cr_log_info("mutex(release)=%u\n", Emulator.mutexCounter);
}
