#ifndef __UNITTEST_EMULATOR_H__
#define __UNITTEST_EMULATOR_H__

#include <stdint.h>
#include "pmic.h"

typedef struct {
    uint8_t id;
} tCommunicationHandle;

typedef struct sTestFramework tTestFramework;

void CommunicationHandleInit(tCommunicationHandle *handle, bool isQAComm);
void TestFrameworkInit(tTestFramework *framework, struct Pmic_CoreHandle_s *pmicCoreHandle);
void TestFrameworkReset(tTestFramework *framework);

void PrintRegisterMap(void);

int32_t PmicCommIoRead(
    struct Pmic_CoreHandle_s *pmicCorehandle,
    uint8_t instType,
    uint16_t regAddr,
    uint8_t *pRxBuf,
    uint8_t bufLen);

int32_t PmicCommIoWrite(
    struct Pmic_CoreHandle_s *pmicCorehandle,
    uint8_t instType,
    uint16_t regAddr,
    uint8_t *pTxBuf,
    uint8_t bufLen);

void CritSecStart(void);
void CritSecStop(void);

#endif // __UNITTEST_EMULATOR_H__