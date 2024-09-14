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
 * @defgroup DRV_PMIC_MODULE PMIC Driver
 *
 * @file pmic.h
 *
 * @brief Top level include file for PMIC LLD.
 */
#ifndef __PMIC_H__
#define __PMIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include "pmic_common.h"
#include "pmic_io.h"
#include "pmic_core.h"
#include "pmic_irq.h"

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_errorCodes
 * @name PMIC LLD Error Codes
 *
 * @brief Error codes returned by PMIC LLD APIs.
 *
 * @{
 */
#define PMIC_ST_SUCCESS                     ((int32_t)0)
#define PMIC_ST_ERR_SPI_COMM_FAIL           (-((int32_t)1))
#define PMIC_ST_ERR_INV_PARAM               (-((int32_t)2))
#define PMIC_ST_ERR_NULL_PARAM              (-((int32_t)3))
#define PMIC_ST_ERR_INV_CRC                 (-((int32_t)4))
#define PMIC_ST_ERR_REG_LOCKED              (-((int32_t)5))
#define PMIC_ST_ERR_INV_HANDLE              (-((int32_t)6))
#define PMIC_ST_ERR_FAIL                    (-((int32_t)7))
#define PMIC_ST_ERR_NOT_SUPPORTED           (-((int32_t)8))
#define PMIC_ST_WARN_NO_IRQ_REMAINING       (-((int32_t)9))
/** @} */

/**
 * @anchor Pmic_CoreCfgValidParams
 * @name PMIC LLD Configuration Valid Parameters
 *
 * @brief Indication of which parameters are valid within the Pmic_CoreCfg_t struct.
 *
 * @details For more information on the parameters, refer to @ref Pmic_CoreCfg.
 *
 * @note End-user could combine multiple valid parameters using the OR operator.
 *
 * @{
 */
#define PMIC_COMM_HANDLE_VALID              ((uint32_t)(1U << 0U))
#define PMIC_IO_TRANSFER_VALID              ((uint32_t)(1U << 1U))
#define PMIC_CRIT_SEC_START_VALID           ((uint32_t)(1U << 2U))
#define PMIC_CRIT_SEC_STOP_VALID            ((uint32_t)(1U << 3U))
#define PMIC_IRQ_RESPONSE_VALID             ((uint32_t)(1U << 4U))
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreCfg
 * @name PMIC LLD Configuration
 *
 * @brief Configuration struct holding end-user settings/parameters relating to
 * the PMIC handle.
 *
 * @attention All parameters of the struct must be set by the end-user. Otherwise,
 * an error may occur during the Pmic_init() API call.
 *
 * @note Once the user sets all struct members, the struct should be passed into
 * Pmic_init() so that the PMIC driver handle can be initialized with the user's
 * desired configurations.
 *
 * @param validParams Each bit in this variable represents whether a struct member
 * is valid. For valid values, refer to @ref Pmic_CoreCfgValidParams.
 *
 * @param commHandle Pointer to platform-specific transport layer communication handle.
 *
 * @param ioTransfer Function pointer to platform-specific SPI read/write API.
 * Platform-specific API must be capable of transmiting 24 bits and receiving 24
 * bits at once (a single SPI frame). The length parameter is specified in
 * number of bytes to be transmitted.
 *
 * @param critSecStart Function pointer to platform-specific critical section start API.
 *
 * @param critSecStop Function pointer to platform-specific critical section stop API.
 *
 * @param irqResponse Function pointer to application-specific response to detected PMIC
 * IRQ. Upon each SPI transaction, the driver checks the PMIC status for faults/IRQs; if
 * at least one is detected, this API hook will be called in response.
 */
typedef struct Pmic_CoreCfg_s
{
    uint32_t validParams;

    void *commHandle;
    int32_t (*ioTransfer)(const struct Pmic_CoreHandle_s *pmicHandle, uint32_t *txBuf, uint32_t *rxBuf, uint8_t length);
    void (*critSecStart)(void);
    void (*critSecStop)(void);
    void (*irqResponse)(void);
} Pmic_CoreCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Initialize PMIC LLD and validate the MCU connection with the TPS65385x
 * PMIC. This API should be called before calling any other TPS65385x LLD APIs.
 *
 * @param pmicCfg [IN] PMIC handle configuration struct. End-user will input
 * their settings/parameters in this struct to initialize the PMIC handle.
 *
 * @param pmicHandle [OUT] PMIC interface handle.
 *
 * @return Success code if PMIC LLD is initialized, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief De-initialize PMIC LLD. This API should be called once the end-user
 * application is complete and/or the end-user wants to close communication with
 * the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC LLD is de-initialized, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *pmicHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_H__ */
