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
 * Design: PMICDRV-618
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioTxByte(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t txData);

/**
 * @brief Write a byte to the given PMIC `regAddr`, performing CRC on communications
 * if necessary and enabled. Additionally, obtain and release a critical section
 * before/after the write.
 *
 * Design: PMICDRV-619
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioTxByte_CS(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t txData);

/**
 * @brief Read a single byte from a target register of the PMIC. This function
 * is only meant to be used internally by the driver, however the end-user could
 * use the API for direct register access.
 *
 * Design: PMICDRV-620
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551, PMICDRV-506
 *               PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioRxByte(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t *rxData);

/**
 * @brief Read a byte from the given PMIC `regAddr`, extracting the desired register
 * data from the CRC framed data returned by the PMIC. Additionally, obtain and
 * release a critical section before/after the read.
 *
 * Design: PMICDRV-621
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551, PMICDRV-506
 *               PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioRxByte_CS(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t *rxData);

/**
 * @brief Executes a read-modify-write routine such that the target bit field is
 * modified without modifying other bit fields.
 *
 * Design: PMICDRV-622
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioUpdateByte(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to `Pmic_ioUpdateByte()` API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the
 * critical section is stopped.
 *
 * Design: PMICDRV-623
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioUpdateByte_CS(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Modify a target bit field of width 1 without modifying other bit fields.
 *
 * Design: PMICDRV-626
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioUpdateByte_b(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value);

/**
 * @brief Identical to `Pmic_ioUpdateByte_b()` API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the critical
 * section is stopped.
 *
 * Design: PMICDRV-627
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
int32_t Pmic_ioUpdateByte_bCS(const Pmic_Handle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value);

/**
 * @brief Control whether serial communication CRC is enabled or disabled. This API
 * is a superset of `Pmic_ioCrcEnable()` and `Pmic_ioCrcDisable()`.
 *
 * Design: PMICDRV-711
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-521
 *               PMICDRV-512
 *
 * @param pmicHandle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set equal to parameter `crcEnable` upon API call success.
 *
 * @param crc8Enable [IN] CRC8 enable/disable. When set to equal to PMIC_ENABLE,
 * CRC8 will be enabled. Else, CRC8 will be disabled.
 *
 * @return Success code if CRC8 has been enabled or disabled, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioSetCrcEnableState(Pmic_Handle_t *pmicHandle, bool crc8Enable);

/**
 * @brief Enable serial communication CRC. This API is a subset of
 * `Pmic_ioSetCrcEnableState()`.
 *
 * Design: PMICDRV-712
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-521
 *               PMICDRV-512
 *
 * @param pmicHandle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set to true upon API call success.
 *
 * @return Success code if CRC8 has been enabled, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioCrcEnable(Pmic_Handle_t *pmicHandle);

/**
 * @brief Disable serial communication CRC. This API is a subset of
 * `Pmic_ioSetCrcEnableState()`.
 *
 * Design: PMICDRV-713
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-521
 *               PMICDRV-512
 *
 * @param pmicHandle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set to false upon API call success.
 *
 * @return Success code if CRC8 has been disabled, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioCrcDisable(Pmic_Handle_t *pmicHandle);

/**
 * @brief Get serial communication CRC enable/disable state.
 *
 * Design: PMICDRV-714
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551, PMICDRV-506
 *               PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-544, PMICDRV-521
 *               PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param crcEnabled [OUT] CRC8 enable status. True if CRC8 is enabled,
 * otherwise false.
 *
 * @return Success code if CRC8 enable status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioGetCrcEnableState(Pmic_Handle_t *pmicHandle, bool *crcEnabled);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_IO_H__ */
