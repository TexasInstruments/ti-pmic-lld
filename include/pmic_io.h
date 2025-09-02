/******************************************************************************
 * Copyright (c) 2025 Texas Instruments Incorporated - http://www.ti.com
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
#ifndef PMIC_IO_H
#define PMIC_IO_H

/**
 * @file pmic_io.h
 * @brief PMIC LLD serial communication I/O module.
 */

/**
 * @defgroup DRV_PMIC_IO_MODULE PMIC Driver I/O Module
 *
 * @brief This module contains serial communication related functionalities,
 * including reading and writing to/from the PMIC, as well as disabling/enabling
 * communication CRC.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function Declarations                            */
/* ========================================================================== */

/**
 * @brief Write a byte to the given PMIC `regAddr`, performing CRC on
 * communications if necessary and enabled.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Register address to write to.
 *
 * @param txData [IN] Data to send to `regAddr`.
 *
 * @return PMIC_ST_SUCCESS if data was successfully transmitted, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @brief Identical to Pmic_ioTxByte() in terms of functionality, but a critical
 * section is started before the write. After the write, the critical section is
 * stopped.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Register address to write to.
 *
 * @param txData [IN] Data to send to `regAddr`.
 *
 * @return PMIC_ST_SUCCESS if data was successfully transmitted, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte_CS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @brief Identical to Pmic_ioTxByte() in terms of functionality, but invokes
 * the critical section stop hook after the write routine.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Register address to write to.
 *
 * @param txData [IN] Data to send to `regAddr`.
 *
 * @return PMIC_ST_SUCCESS if data was successfully transmitted, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte_endCS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @brief Read a byte from the given PMIC `regAddr`, extracting the desired
 * register data from the CRC framed data returned by the PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Register address to read from.
 *
 * @param rxBuffer [OUT] Buffer to store result data in.
 *
 * @return PMIC_ST_SUCCESS if data was successfully obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxData);

/**
 * @brief Identical to Pmic_ioRxByte() in terms of functionality, but a critical
 * section is started before the read. After the read, the critical section is
 * stopped.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Register address to read from.
 *
 * @param rxBuffer [OUT] Buffer to store result data in.
 *
 * @return PMIC_ST_SUCCESS if data was successfully obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte_CS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxData);

/**
 * @brief Identical to Pmic_ioRxByte() in terms of functionality, but invokes
 * the critical section start hook before the read routine.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] Register address to read from.
 *
 * @param rxData [IN] Data received from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if data was successfully obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte_startCS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxData);

/**
 * @brief Write up to 4 bytes to a linear sequence of registers starting at
 * `baseAddr`, performing CRC on communications if necessary and enabled.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param baseAddr [IN] Register address to start writing to.
 *
 * @param txData [IN] Data to send.
 *
 * @param count [IN] Number of bytes to write, must not be more than what can
 * be stored in a uint32_t.
 *
 * @return PMIC_ST_SUCCESS if data was successfully transmitted, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxWordSeq(const Pmic_Handle_t *handle, uint16_t baseAddr, uint32_t txData, uint8_t count);

/**
 * @brief Read up to 4 bytes from a linear sequence of registers starting at
 * `baseAddr`, extracting the desired register data from the CRC framed data
 * returned by the PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param baseAddr [IN] Register address to start reading from.
 *
 * @param rxData [OUT] Buffer to store result data in.
 *
 * @param count [IN] Number of bytes to read, must not be more than what can
 * be stored in a uint32_t.
 *
 * @return PMIC_ST_SUCCESS if data was successfully obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxWordSeq(const Pmic_Handle_t *handle, uint16_t baseAddr, uint32_t *rxData, uint8_t count);

/**
 * @brief Executes a read-modify-write routine such that the target bit field is
 * modified without modifying other bit fields.
 *
 * @param handle [IN] PMIC interface handle.
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
int32_t Pmic_ioReadModifyWrite(const Pmic_Handle_t *handle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to Pmic_ioReadModifyWrite() API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the
 * critical section is stopped.
 *
 * @param handle [IN] PMIC interface handle.
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
int32_t Pmic_ioReadModifyWrite_CS(const Pmic_Handle_t *handle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to Pmic_ioReadModifyWrite() API, but only the name of the
 * bit field needs to be specified (case-sensitive).
 *
 * @param handle [IN] PMIC interface handle.
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
#define Pmic_ioReadModifyWriteByName(handle, regAddr, name, value) \
    Pmic_ioReadModifyWrite((Pmic_Handle_t*)handle, (uint8_t)regAddr, (uint8_t)(name##_SHIFT), (uint8_t)(name##_MASK), (uint8_t)value)

/**
 * @brief Identical to Pmic_ioReadModifyWrite_CS() API, but only the name of the
 * bit field needs to be specified (case-sensitive).
 *
 * @param handle [IN] PMIC interface handle.
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
#define Pmic_ioReadModifyWriteByName_CS(handle, regAddr, name, value) \
    Pmic_ioReadModifyWrite_CS((Pmic_Handle_t*)handle, (uint8_t)regAddr, (uint8_t)(name##_SHIFT), (uint8_t)(name##_MASK), (uint8_t)value)

/**
 * @brief Modify a target bit field of width 1 without modifying other bit fields.
 *
 * @param handle [IN] PMIC interface handle.
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
int32_t Pmic_ioReadModifyWrite_b(const Pmic_Handle_t *handle, uint8_t regAddr, uint8_t shift, bool value);

/**
 * @brief Identical to Pmic_ioReadModifyWrite_b() API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the
 * critical section is stopped.
 *
 * @param handle [IN] PMIC interface handle.
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
int32_t Pmic_ioReadModifyWrite_bCS(const Pmic_Handle_t *handle, uint8_t regAddr, uint8_t shift, bool value);

/**
 * @brief Get serial communication CRC enable/disable status.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] Set to true (PMIC_ENABLE) if comms CRC is enabled,
 * false (PMIC_DISABLE) if disabled. See @ref Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS if serial communications CRC enable/disable status
 * has been obtained, error code otherwise. For valid success/error codes, refer
 * to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioGetCrcEnableState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Enable/disable serial communication CRC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] Set to true (PMIC_ENABLE) to enable comms CRC, false
 * (PMIC_DISABLE) to disable. See @ref Pmic_EnableDisable.
 *
 * @return PMIC_ST_SUCCESS if serial communication CRC has been enabled or
 * disabled, error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioSetCrcEnableState(Pmic_Handle_t *handle, bool enable);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_IO_H */
