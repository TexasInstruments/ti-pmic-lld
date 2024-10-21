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
 * @brief LLD-Communication header file containing communication wrapper read/write
 * API used internally by PMIC LLD.
 */
#ifndef __PMIC_IO_H__
#define __PMIC_IO_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_CommandCodes
 * @name TPS65385xx PMIC Command Codes
 *
 * @brief SPI command codes supported by the TPS65385xx PMIC.
 * @{
 */
#define PMIC_CMD_MCU_RST_RQ                 ((uint8_t)0x04U)
#define PMIC_CMD_MCU_RST_RQ_DATA            ((uint8_t)0x5AU)
#define PMIC_CMD_SAFE_EXIT                  ((uint8_t)0x9AU)
#define PMIC_CMD_SAFE_EXIT_DATA             ((uint8_t)0xA5U)
#define PMIC_CMD_SW_UNLOCK                  ((uint8_t)0xBBU)
#define PMIC_CMD_SW_UNLOCK_DATA             ((uint8_t)0x55U)
#define PMIC_CMD_SW_LOCK                    ((uint8_t)0xBDU)
#define PMIC_CMD_SW_LOCK_DATA               ((uint8_t)0xAAU)
#define PMIC_CMD_WR_DEV_CFG_1               ((uint8_t)0xB7U)
#define PMIC_CMD_WR_DEV_CFG_2               ((uint8_t)0x95U)
#define PMIC_CMD_WR_DEV_CFG_3               ((uint8_t)0x83U)
#define PMIC_CMD_WR_WD_WIN1_CFG             ((uint8_t)0xEDU)
#define PMIC_CMD_WR_WD_WIN2_CFG             ((uint8_t)0x09U)
#define PMIC_CMD_WR_WD_QUESTION_FDBCK       ((uint8_t)0x77U)
#define PMIC_CMD_WR_WD_ANSWER               ((uint8_t)0xE1U)
#define PMIC_CMD_WR_SAFETY_CFG_CRC          ((uint8_t)0x63U)
#define PMIC_CMD_WR_SAFETY_ERR_STAT_1       ((uint8_t)0xA9U)
#define PMIC_CMD_WR_SAFETY_ERR_CFG_1        ((uint8_t)0xDBU)
#define PMIC_CMD_WR_SAFETY_ERR_CFG_2        ((uint8_t)0xC9U)
#define PMIC_CMD_WR_SAFETY_CHECK_CTRL       ((uint8_t)0x93U)
#define PMIC_CMD_WR_SAFETY_FUNC_CFG         ((uint8_t)0x35U)
#define PMIC_CMD_WR_SAFETY_BIST_CTRL        ((uint8_t)0x9FU)
#define PMIC_CMD_WR_SAFETY_PWD_THR_CFG      ((uint8_t)0x99U)
#define PMIC_CMD_WR_SAFETY_ERR_PWM_HMAX     ((uint8_t)0xD8U)
#define PMIC_CMD_WR_SAFETY_ERR_PWM_HMIN     ((uint8_t)0xB0U)
#define PMIC_CMD_WR_SAFETY_ERR_PWM_LMAX     ((uint8_t)0x7EU)
#define PMIC_CMD_WR_SAFETY_ERR_PWM_LMIN     ((uint8_t)0x5FU)
#define PMIC_CMD_WR_SENS_CTRL               ((uint8_t)0x7BU)
#define PMIC_CMD_WR_SAM_CFG                 ((uint8_t)0x87U)
#define PMIC_CMD_WR_LPSAM_PERIOD            ((uint8_t)0xE4U)
#define PMIC_CMD_WR_LPSAM_IDLE              ((uint8_t)0x41U)
#define PMIC_CMD_WR_LPSAM_SAMPLE            ((uint8_t)0xDFU)
#define PMIC_CMD_RD_DEV_ID                  ((uint8_t)0x06U)
#define PMIC_CMD_RD_DEV_REV                 ((uint8_t)0x0CU)
#define PMIC_CMD_RD_DEV_CFG_1               ((uint8_t)0xAFU)
#define PMIC_CMD_RD_DEV_CFG_2               ((uint8_t)0x48U)
#define PMIC_CMD_RD_DEV_CFG_3               ((uint8_t)0x3FU)
#define PMIC_CMD_RD_DEV_STAT                ((uint8_t)0x11U)
#define PMIC_CMD_RD_SAM_STAT                ((uint8_t)0x31U)
#define PMIC_CMD_RD_SAM_SIG_STAT            ((uint8_t)0xB9U)
#define PMIC_CMD_RD_VMON_STAT_1             ((uint8_t)0x12U)
#define PMIC_CMD_RD_VMON_STAT_2             ((uint8_t)0xA6U)
#define PMIC_CMD_RD_VMON_STAT_3             ((uint8_t)0xDAU)
#define PMIC_CMD_RD_SAFETY_STAT_1           ((uint8_t)0x24U)
#define PMIC_CMD_RD_SAFETY_STAT_2           ((uint8_t)0xC5U)
#define PMIC_CMD_RD_SAFETY_STAT_3           ((uint8_t)0xA3U)
#define PMIC_CMD_RD_SAFETY_STAT_4           ((uint8_t)0xA5U)
#define PMIC_CMD_RD_SAFETY_STAT_5           ((uint8_t)0xC0U)
#define PMIC_CMD_RD_SAFETY_ERR_STAT_1       ((uint8_t)0xAAU)
#define PMIC_CMD_RD_SAFETY_ERR_STAT_2       ((uint8_t)0x4AU)
#define PMIC_CMD_RD_SPI_INV_TRAN_STAT       ((uint8_t)0xB3U)
#define PMIC_CMD_RD_SAFETY_ERR_CFG_1        ((uint8_t)0x30U)
#define PMIC_CMD_RD_SAFETY_ERR_CFG_2        ((uint8_t)0xE9U)
#define PMIC_CMD_RD_SAFETY_CHECK_CTRL       ((uint8_t)0x44U)
#define PMIC_CMD_RD_SAFETY_FUNC_CFG         ((uint8_t)0x3AU)
#define PMIC_CMD_RD_SAFETY_BIST_CTRL        ((uint8_t)0x3CU)
#define PMIC_CMD_RD_SAFETY_PWD_THR_CFG      ((uint8_t)0x39U)
#define PMIC_CMD_RD_SAFETY_ERR_PWM_HMAX     ((uint8_t)0xD7U)
#define PMIC_CMD_RD_SAFETY_ERR_PWM_HMIN     ((uint8_t)0x52U)
#define PMIC_CMD_RD_SAFETY_ERR_PWM_LMAX     ((uint8_t)0x59U)
#define PMIC_CMD_RD_SAFETY_ERR_PWM_LMIN     ((uint8_t)0x80U)
#define PMIC_CMD_RD_WD_QUESTION_FDBCK       ((uint8_t)0x78U)
#define PMIC_CMD_RD_WD_QUESTION             ((uint8_t)0x36U)
#define PMIC_CMD_RD_WD_STATUS               ((uint8_t)0x4EU)
#define PMIC_CMD_RD_WD_WIN1_CFG             ((uint8_t)0x2EU)
#define PMIC_CMD_RD_WD_WIN2_CFG             ((uint8_t)0x05U)
#define PMIC_CMD_RD_SENS_CTRL               ((uint8_t)0x56U)
#define PMIC_CMD_RD_SAM_CFG                 ((uint8_t)0xD1U)
#define PMIC_CMD_RD_LPSAM_PERIOD            ((uint8_t)0x0BU)
#define PMIC_CMD_RD_LPSAM_IDLE              ((uint8_t)0x17U)
#define PMIC_CMD_RD_LPSAM_SAMPLE            ((uint8_t)0x1CU)
/** @} */

/**
 * @anchor Pmic_invalidData
 * @name TPS65385xx PMIC Invalid Data
 *
 * @brief Used in Pmic_ioTransfer() and Pmic_ioTransfer_CS() to indicate invalid
 * write/read data.
 * @{
 */
#define PMIC_NO_WRITE_DATA                  ((uint8_t)0U)
#define PMIC_NO_READ_DATA                   (NULL)
/** @} */

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Write one byte to the PMIC and/or receive one byte from the PMIC.
 * End-users could use this API for direct register access.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param cmd [IN] PMIC command. For valid values, refer to @ref Pmic_CommandCodes.
 *
 * @param wData [IN] Write data. If @p cmd is a read command, the appropriate macro
 * defined under @ref Pmic_invalidData can be passed as input to this parameter.
 * Otherwise, the value of @p wData is ignored by the PMIC.
 *
 * @param rData [OUT] Read data. If @p cmd is a write command, the appropriate macro
 * defined under @ref Pmic_invalidData can be passed as input to this parameter.
 * Otherwise, the dereferenced value of @p rData is set to zero by the PMIC.
 *
 * @return Success code if the SPI transfer is complete, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioTransfer(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData, uint8_t *rData);

/**
 * @brief Write a single byte to the PMIC. End-users could use this API for
 * direct register access.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param cmd [IN] PMIC command. For valid values, refer to @ref Pmic_CommandCodes.
 *
 * @param wData [IN] Data to be written to the PMIC.
 *
 * @return Success code if the SPI write is complete, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
static inline int32_t Pmic_ioTxByte(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData)
{
    return Pmic_ioTransfer(pmicHandle, cmd, wData, PMIC_NO_READ_DATA);
}

/**
 * @brief Read a single byte from the PMIC. End-users could use this API for
 * direct register access.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param cmd [IN] PMIC command. For valid values, refer to @ref Pmic_CommandCodes.
 *
 * @param rData [OUT] Data received from the PMIC.
 *
 * @return Success code if the SPI read is complete, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
static inline int32_t Pmic_ioRxByte(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t *rData)
{
    return Pmic_ioTransfer(pmicHandle, cmd, PMIC_NO_WRITE_DATA, rData);
}

/**
 * @brief Identical to Pmic_ioTransfer(), however, a critical section is started
 * before the I/O transaction operation occurs. After the operation, the
 * critical section is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param cmd [IN] PMIC command. For valid values, refer to @ref Pmic_CommandCodes.
 *
 * @param wData [IN] Write data. If @p cmd is a read command, the appropriate macro
 * defined under @ref Pmic_invalidData can be passed as input to this parameter.
 * Otherwise, the value of @p wData is ignored by the PMIC.
 *
 * @param rData [OUT] Read data. If @p cmd is a write command, the appropriate macro
 * defined under @ref Pmic_invalidData can be passed as input to this parameter.
 * Otherwise, the dereferenced value of @p rData is set to zero by the PMIC.
 *
 * @return Success code if the SPI transfer is complete, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioTransfer_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData, uint8_t *rData);

/**
 * @brief Identical to Pmic_ioTxByte(), however, a critical section is started
 * before the I/O operation occurs. After the operation, the critical section
 * is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param cmd [IN] PMIC command. For valid values, refer to @ref Pmic_CommandCodes.
 *
 * @param wData [IN] Data to be written to the PMIC.
 *
 * @return Success code if the SPI write is complete, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioTxByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t wData);

/**
 * @brief Identical to Pmic_ioRxByte(), however, a critical section is started
 * before the I/O operation occurs. After the operation, the critical section
 * is stopped.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param cmd [IN] PMIC command. For valid values, refer to @ref Pmic_CommandCodes.
 *
 * @param rData [OUT] Data received from the PMIC.
 *
 * @return Success code if the SPI read is complete, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_ioRxByte_CS(const Pmic_CoreHandle_t *pmicHandle, uint8_t cmd, uint8_t *rData);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_IO_H */
