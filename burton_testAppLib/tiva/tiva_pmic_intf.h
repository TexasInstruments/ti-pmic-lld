#ifndef TIVA_PMIC_INTF_H_
#define TIVA_PMIC_INTF_H_

#include "pmic_drv/pmic.h"

#ifdef __cplusplus
extern "C"
{
#endif

int32_t initializePmicCoreHandle(Pmic_CoreHandle_t *pmicCoreHandle);
int32_t pmicI2CRead(Pmic_CoreHandle_t *pmicCorehandle, uint8_t instType, uint16_t regAddr, uint8_t *pRxBuf,
                    uint8_t bufLen);
int32_t pmicI2CWrite(Pmic_CoreHandle_t *pmicCorehandle, uint8_t instType, uint16_t regAddr, uint8_t *pTxBuf,
                     uint8_t bufLen);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_PMIC_INTF_H_ */
