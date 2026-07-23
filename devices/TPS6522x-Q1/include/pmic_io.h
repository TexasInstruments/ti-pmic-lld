/******************************************************************************
 * Copyright (c) 2026 Texas Instruments Incorporated - http://www.ti.com
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
#ifndef PMIC_IO_H
#define PMIC_IO_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

#include "pmic_common.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_IoCrcCfgValidParams
 * @name PMIC IO CRC Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_IoCrcCfg_t`.
 * Set the `validParams` member of `Pmic_IoCrcCfg_t` equal to a
 * combination of these defines by using the `OR` operator.
 *
 * @{
 */
#define PMIC_CFG_IO_CRC_ENABLE_0_VALID (1U << 0)
#define PMIC_CFG_IO_CRC_ENABLE_1_VALID (1U << 1)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_IoCrcCfg
 * @name PMIC IO CRC Configuration Structure
 *
 * @brief Structure used to configure the CRC settings for PMIC IO operations.
 *
 * @param validParams Bitmap indicating which parameters are valid. For more
 * information, refer to @ref Pmic_IoCrcCfgValidParams.
 *
 * @param crcEnable0 Enable CRC for channel 0.
 *
 * @param crcEnable1 Enable CRC for channel 1.
 */
typedef struct Pmic_IoCrcCfg_s {
    uint32_t validParams;

    bool crcEnable0;
    bool crcEnable1;
} Pmic_IoCrcCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Write a single byte to a target register of the PMIC. This function
 * is only meant to be used internally by the driver, however the end-user
 * could use the API for direct register access.
 *
 * Design: PMICDRV-618
  * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509
 *               PMICDRV-511, PMICDRV-512, PMICDRV-517, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544
 *               PMICDRV-549, PMICDRV-552
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param txData [IN] Data to write to PMIC register.
 *
 * @return PMIC_ST_SUCCESS if byte has been successfully transmitted to PMIC,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @brief Write a byte to the given PMIC `regAddr`, performing CRC on communications
 * if necessary and enabled. Additionally, obtain and release a critical section
 * before/after the write.
 *
 * Design: PMICDRV-619
  * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509
 *               PMICDRV-511, PMICDRV-512, PMICDRV-517, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544, PMICDRV-552
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param txData [IN] Data to write to PMIC register.
 *
 * @return PMIC_ST_SUCCESS if byte has been successfully transmitted to PMIC,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioTxByte_CS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t txData);

/**
 * @brief Read a single byte from a target register of the PMIC. This function
 * is only meant to be used internally by the driver, however the end-user could
 * use the API for direct register access.
 *
 * Design: PMICDRV-620
 * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509, PMICDRV-511, PMICDRV-512
 *               PMICDRV-517, PMICDRV-521, PMICDRV-522, PMICDRV-526, PMICDRV-528, PMICDRV-544
 *               PMICDRV-549, PMICDRV-552
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param rxData [OUT] Pointer to variable in which PMIC register data will
 * be stored.
 *
 * @return PMIC_ST_SUCCESS if byte has been successfully obtained from PMIC,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxData);

/**
 * @brief Read a byte from the given PMIC `regAddr`, extracting the desired register
 * data from the CRC framed data returned by the PMIC. Additionally, obtain and
 * release a critical section before/after the read.
 *
 * Design: PMICDRV-621
 * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509, PMICDRV-511, PMICDRV-512
 *               PMICDRV-517, PMICDRV-521, PMICDRV-522, PMICDRV-526, PMICDRV-528, PMICDRV-544
 *               PMICDRV-552
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param regAddr [IN] PMIC register address.
 *
 * @param rxData [OUT] Pointer to variable in which PMIC register data will
 * be stored.
 *
 * @return PMIC_ST_SUCCESS if byte has been successfully obtained from PMIC,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioRxByte_CS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t *rxData);

/**
 * @brief Executes a read-modify-write routine such that the target bit field is
 * modified without modifying other bit fields.
 *
 * Design: PMICDRV-622
 * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509, PMICDRV-511, PMICDRV-512, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544, PMICDRV-549, PMICDRV-552
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
 * @return PMIC_ST_SUCCESS if read-modify-write operation was successful, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioUpdateByte(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Identical to `Pmic_ioReadModifyWrite()` API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the
 * critical section is stopped.
 *
 * Design: PMICDRV-623
 * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509, PMICDRV-511, PMICDRV-512, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544, PMICDRV-552
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
 * @return PMIC_ST_SUCCESS if read-modify-write operation was successful, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioUpdateByte_CS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t shift, uint8_t mask, uint8_t value);

/**
 * @brief Modify a target bit field of width 1 without modifying other bit fields.
 *
 * Design: PMICDRV-626
 * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509, PMICDRV-511, PMICDRV-512, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544, PMICDRV-552
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
 * @return PMIC_ST_SUCCESS if read-modify-write operation was successful, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioUpdateByte_b(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t shift, bool value);

/**
 * @brief Identical to `Pmic_ioReadModifyWrite_b()` API but starts a critical
 * section before the read, modify, and write operations. Afterwards, the critical
 * section is stopped.
 *
 * Design: PMICDRV-627
 * Architecture: PMICDRV-504, PMICDRV-505, PMICDRV-506, PMICDRV-509, PMICDRV-511, PMICDRV-512, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544, PMICDRV-552
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
 * @return PMIC_ST_SUCCESS if read-modify-write operation was successful, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioUpdateByte_bCS(const Pmic_Handle_t *handle, uint16_t regAddr, uint8_t shift, bool value);

/**
 * @brief Control whether serial communication CRC is enabled or disabled. This API
 * is a superset of `Pmic_ioCrcEnable()` and `Pmic_ioCrcDisable()`.
 *
 * Design: PMICDRV-711
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-526
 *               PMICDRV-544
 *
 * @param handle [IN/OUT] PMIC interface handle.
 *
 * @param cfg [IN] CRC configuration structure specifying which channel(s) to
 * enable or disable and whether to enable or disable CRC for each channel.
 *
 * @return PMIC_ST_SUCCESS if serial communication CRC has been enabled or disabled,
 * error code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioSetCrcEnableState(Pmic_Handle_t *handle, const Pmic_IoCrcCfg_t *cfg);

/**
 * @brief Enable serial communication CRC. This API is a subset of
 * `Pmic_ioSetCrcEnableState()`.
 *
 * Design: PMICDRV-712
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544
 *
 * @param handle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set to true upon API call success.
 *
 * @return PMIC_ST_SUCCESS if serial communication CRC has been enabled, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioCrcEnable(Pmic_Handle_t *handle);

/**
 * @brief Disable serial communication CRC. This API is a subset of
 * `Pmic_ioSetCrcEnableState()`.
 *
 * Design: PMICDRV-713
  * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521
 *               PMICDRV-522, PMICDRV-523, PMICDRV-526, PMICDRV-544
 *
 * @param handle [IN/OUT] PMIC interface handle. The crcEnable struct member
 * will be set to false upon API call success.
 *
 * @return PMIC_ST_SUCCESS if serial communication CRC has been disabled, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioCrcDisable(Pmic_Handle_t *handle);

/**
 * @brief Get serial communication CRC enable/disable state.
 *
 * Design: PMICDRV-714
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-526, PMICDRV-528
 *               PMICDRV-544
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param cfg [OUT] CRC configuration structure. Fields are populated according
 * to validParams: set PMIC_CFG_IO_CRC_ENABLE_0_VALID to read crcEnable0
 * (I2C1/SPI), set PMIC_CFG_IO_CRC_ENABLE_1_VALID to read crcEnable1 (I2C2).
 *
 * @return PMIC_ST_SUCCESS if CRC enable status has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_ioGetCrcEnableState(const Pmic_Handle_t *handle, Pmic_IoCrcCfg_t *cfg);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* PMIC_IO_H */
