/******************************************************************************
 * Copyright (c) 2024 Texas Instruments Incorporated - http://www.ti.com
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
/**
 * @file pmic_io.h
 *
 * @brief LLD-Communication header file containing I2C read/write APIs used
 * internally by PMIC LLD.
 */
#ifndef __PMIC_IO_H__
#define __PMIC_IO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/*==========================================================================  */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Write a single byte to a target register of the PMIC. This function
 * is only meant to be used internally by the driver, however the end-user
 * could use the API for direct register access.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param txData [IN] Data to write to PMIC register.
 *
 * @return Success code if byte has been successfully transmitted to PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioTxByte(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t txData);

/**
 * @brief Identical to Pmic_ioTxByte(), however, a critical section is started
 * before the read operation occurs. After the operation, the critical section
 * is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param txData [IN] Data to write to PMIC register.
 *
 * @return Success code if byte has been successfully transmitted to PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioTxByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t txData);

/**
 * @brief Read a single byte from a target register of the PMIC. This function
 * is only meant to be used internally by the driver, however the end-user could
 * use the API for direct register access.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param rxData [OUT] Pointer to variable in which PMIC register data will
 * be stored.
 *
 * @return Success code if byte has been successfully obtained from PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioRxByte(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t *rxData);

/**
 * @brief Identical to Pmic_ioRxByte(), however, a critical section is started
 * before the read operation occurs. After the operation, the critical section
 * is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param rxData [OUT] Pointer to variable in which PMIC register data will
 * be stored.
 *
 * @return Success code if byte has been successfully obtained from PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioRxByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t *rxData);

/**
 * @brief Executes a read-modify-write routine such that the target bit field is
 * modified without modifying other bit fields.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target register address.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @param value [IN] Desired value to set the bit field to.
 *
 * @return Success code if read-modify-write operation was successful, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioReadModifyWrite(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to Pmic_ioReadModifyWrite() API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the
 * critical section is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target register address.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @param value [IN] Desired value to set the bit field to.
 *
 * @return Success code if read-modify-write operation was successful, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioReadModifyWrite_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to Pmic_ioReadModifyWrite() API, but only the name of the
 * bit field needs to be specified (case-sensitive).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target register address.
 *
 * @param name [IN] Target bit field name.
 *
 * @param value [IN] Desired value to set the bit field to.
 *
 * @return Success code if read-modify-write operation was successful, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
#define Pmic_ioReadModifyWriteByName(pmicHandle, regAddr, name, value) \
    Pmic_ioReadModifyWrite((Pmic_CoreHandle_t*)pmicHandle, (uint8_t)regAddr, (uint8_t)(name##_SHIFT), (uint8_t)(name##_MASK), (uint8_t)value)

/**
 * @brief Identical to Pmic_ioReadModifyWrite_CS() API, but only the name of the
 * bit field needs to be specified (case-sensitive).
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target register address.
 *
 * @param name [IN] Target bit field name.
 *
 * @param value [IN] Desired value to set the bit field to.
 *
 * @return Success code if read-modify-write operation was successful, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
#define Pmic_ioReadModifyWriteByName_CS(pmicHandle, regAddr, name, value) \
    Pmic_ioReadModifyWrite_CS((Pmic_CoreHandle_t*)pmicHandle, (uint8_t)regAddr, (uint8_t)(name##_SHIFT), (uint8_t)(name##_MASK), (uint8_t)value)

/**
 * @brief Modify a target bit field of width 1 without modifying other bit fields.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target register address.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param value [IN] Desired value to set the bit field to. If value is true,
 * the bit field is set to 1. Otherwise, the bit field is set to 0.
 *
 * @return Success code if read-modify-write operation was successful, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioReadModifyWrite_b(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value);

/**
 * @brief Identical to Pmic_ioReadModifyWrite_b() API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the
 * critical section is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Target register address.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param value [IN] Desired value to set the bit field to. If value is true,
 * the bit field is set to 1. Otherwise, the bit field is set to 0.
 *
 * @return Success code if read-modify-write operation was successful, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioReadModifyWrite_bCS(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_IO_H__ */
