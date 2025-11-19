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
#ifndef __PMIC_IO_H__
#define __PMIC_IO_H__

/**
 * @file pmic_io.h
 * @brief PMIC Driver Communications I/O API
 */

/**
 * @defgroup DRV_PMIC_IO_MODULE PMIC I/O Module
 * @brief Communications APIs to read from and write to PMIC registers.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include <stdint.h>

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_CrcEnableCfg
 * @name PMIC CRC Enable/Disable Configuration
 *
 *  @{
 */
#define PMIC_CRC_DISABLE    (0U)
#define PMIC_CRC_ENABLE     (1U)
/** @} */

/*==========================================================================*/
/*                          Structures and Enums                            */
/*==========================================================================*/

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Write a byte to the given PMIC `regAddr`, performing CRC on
 * communications if necessary and enabled.
 *
 * Design: PMICDRV-618
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
 *
 * @param handle  [IN] PMIC Interface Handle
 * @param regAddr [IN] Register address to write to
 * @param txData  [IN] Data to send to `regAddr`
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Identical to Pmic_ioTxByte() in terms of functionality, but a critical
 * section is started before the write. After the write, the critical section is
 * stopped.
 *
 * Design: PMICDRV-619
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
 *
 * @param handle  [IN] PMIC Interface Handle
 * @param regAddr [IN] Register address to write to
 * @param txData  [IN] Data to send to `regAddr`
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte_CS(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Read a byte from the given PMIC `regAddr`, extracting the desired
 * register data from the CRC framed data returned by the PMIC.
 *
 * Design: PMICDRV-620
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551, PMICDRV-506
 *               PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
 *
 * @param handle   [IN]  PMIC Interface Handle
 * @param regAddr  [IN]  Register address to read from
 * @param rxBuffer [OUT] Buffer to store result data in
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t *rxBuffer);

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Identical to Pmic_ioRxByte() in terms of functionality, but a critical
 * section is started before the read. After the read, the critical section is
 * stopped.
 *
 * Design: PMICDRV-621
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551, PMICDRV-506
 *               PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-517, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
 *
 * @param handle   [IN]  PMIC Interface Handle
 * @param regAddr  [IN]  Register address to read from
 * @param rxBuffer [OUT] Buffer to store result data in
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte_CS(Pmic_CoreHandle_t *handle, uint16_t regAddr, uint8_t *rxBuffer);

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
int32_t Pmic_ioUpdateByte(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

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
int32_t Pmic_ioUpdateByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to `Pmic_ioUpdateByte()` API, but only the name of the
 * bit field needs to be specified (case-sensitive).
 *
 * Design: PMICDRV-624
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
#define Pmic_ioUpdateByteByName(pmicHandle, regAddr, name, value) \
    Pmic_ioUpdateByte((Pmic_CoreHandle_t*)pmicHandle, (uint8_t)regAddr, (uint8_t)(name##_SHIFT), (uint8_t)(name##_MASK), (uint8_t)value)

/**
 * @brief Identical to `Pmic_ioUpdateByte_CS()` API, but only the name of the
 * bit field needs to be specified (case-sensitive).
 *
 * Design: PMICDRV-625
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544, PMICDRV-552
 *               PMICDRV-521, PMICDRV-505, PMICDRV-512, PMICDRV-511, PMICDRV-509
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
#define Pmic_ioUpdateByteByName_CS(pmicHandle, regAddr, name, value) \
    Pmic_ioUpdateByte_CS((Pmic_CoreHandle_t*)pmicHandle, (uint8_t)regAddr, (uint8_t)(name##_SHIFT), (uint8_t)(name##_MASK), (uint8_t)value)

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
int32_t Pmic_ioUpdateByte_b(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value);

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
int32_t Pmic_ioUpdateByte_bCS(const Pmic_CoreHandle_t *pmicHandle, uint8_t regAddr, uint8_t shift, bool value);

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Write up to 4 bytes to a linear sequence of registers starting at
 * `baseAddr`, performing CRC on communications if necessary and enabled.
 *
 * Design: PMICDRV-628
 * Architecture: PMICDRV-510, PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-549
 *               PMICDRV-551, PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-544
 *               PMICDRV-552, PMICDRV-521, PMICDRV-517, PMICDRV-512, PMICDRV-511, PMICDRV-509
 *
 * @param handle   [IN] PMIC Interface Handle
 * @param baseAddr [IN] Register address to start writing to
 * @param txData   [IN] Data to send
 * @param count    [IN] Number of bytes to write, must not be more than what can
 *                      be stored in a uint32_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxWordSeq(Pmic_CoreHandle_t *handle, uint16_t baseAddr, uint32_t txData, uint8_t count);

/**
 * @ingroup DRV_PMIC_IO_MODULE
 * @brief Read up to 4 bytes from a linear sequence of registers starting at
 * `baseAddr`, extracting the desired register data from the CRC framed data
 * returned by the PMIC.
 *
 * Design: PMICDRV-629
 * Architecture: PMICDRV-510, PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-528, PMICDRV-544
 *               PMICDRV-552, PMICDRV-521, PMICDRV-517, PMICDRV-512, PMICDRV-511, PMICDRV-509
 *
 * @param handle   [IN]  PMIC Interface Handle
 * @param baseAddr [IN]  Register address to start reading from
 * @param rxData   [OUT] Buffer to store result data in
 * @param count    [IN]  Number of bytes to read, must not be more than what can
 *                       be stored in a uint32_t.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values, see @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxWordSeq(Pmic_CoreHandle_t *handle, uint16_t baseAddr, uint32_t *rxData, uint8_t count);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__PMIC_IO_H__*/
