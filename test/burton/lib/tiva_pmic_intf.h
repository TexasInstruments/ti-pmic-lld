/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 *    documentation and/or other materials provided with the
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
 * \param pmicHandle [OUT] PMIC handle to initialize
 */
void initializePmicCoreHandle(Pmic_CoreHandle_t *pmicHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_PMIC_INTF_H_ */
