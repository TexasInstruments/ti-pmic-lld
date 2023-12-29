#ifndef TIVA_PMIC_INTF_H_
#define TIVA_PMIC_INTF_H_

#include "pmic.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \brief API used by the PMIC driver to transmit bytes over I2C to the target PMIC
 *
 * \param pmicCorehandle    [IN]        Handle to PMIC
 * \param instType          [IN]        Instance type of the write
 * \param regAddr           [IN]        Target internal register address of the PMIC
 * \param pTxBuf            [IN]        Buffer containing the bytes to transmit over I2C
 * \param bufLen            [IN]        Number of bytes to send over I2C
 *
 * \return                  int32_t     PMIC_ST_SUCCESS in case of success or appropriate error code.
 *                                      For valid values \ref Pmic_ErrorCodes.
 */
int32_t
pmicI2CRead(Pmic_CoreHandle_t *pmicCorehandle, uint8_t instType, uint16_t regAddr, uint8_t *pRxBuf, uint8_t bufLen);

/**
 * \brief API used by the PMIC driver to receive bytes over I2C from the target PMIC
 *
 * \param pmicCorehandle    [IN]        Handle to PMIC
 * \param instType          [IN]        Instance type of the read
 * \param regAddr           [IN]        Target internal register address of the PMIC
 * \param pRxBuf            [OUT]       Buffer containing the bytes received over I2C
 * \param bufLen            [IN]        Number of bytes to receive over I2C
 *
 * \return                  int32_t     PMIC_ST_SUCCESS in case of success or appropriate error code.
 *                                      For valid values \ref Pmic_ErrorCodes.
 */
int32_t
pmicI2CWrite(Pmic_CoreHandle_t *pmicCorehandle, uint8_t instType, uint16_t regAddr, uint8_t *pTxBuf, uint8_t bufLen);

/**
 * \brief Function used by the PMIC driver to indicate and start a critical section
 */
void pmicCritSecStart(void);

/**
 * \brief Function used by the PMIC driver to stop a critical section
 */
void pmicCritSecStop(void);

/**
 * \brief This function initalizes the handle to a PMIC.
 *
 * \param pmicCoreHandle [OUT] PMIC handle to initialize
 */
void initializePmicCoreHandle(Pmic_CoreHandle_t *pmicCoreHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_PMIC_INTF_H_ */
