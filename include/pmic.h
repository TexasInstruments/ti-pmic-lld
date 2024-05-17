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
#include "pmic_core.h"
#include "pmic_wdg.h"
#include "pmic_io.h"

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_errorCodes
 * @name PMIC LLD Error Codes
 *
 * @brief Error codes returned by PMIC LLD APIs
 *
 * @{
 */
#define PMIC_ST_SUCCESS                     ((int32_t)0)
#define PMIC_ST_ERR_I2C_COMM_FAIL           (-((int32_t)1))
#define PMIC_ST_ERR_INV_PARAM               (-((int32_t)2))
#define PMIC_ST_ERR_NULL_PARAM              (-((int32_t)3))
#define PMIC_ST_ERR_INV_CRC                 (-((int32_t)4))
#define PMIC_ST_ERR_REG_LOCKED              (-((int32_t)5))
#define PMIC_ST_ERR_INV_HANDLE              (-((int32_t)6))
#define PMIC_ST_ERR_FAIL                    (-((int32_t)7))
#define PMIC_ST_ERR_NOT_SUPPORTED           (-((int32_t)8))
#define PMIC_ST_WARN_NO_IRQ_REMAINING       (-((int32_t)9))
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreCfg
 * @name PMIC LLD Configuration
 *
 * @brief Configuration struct holding end-user settings/parameters relating to
 * the PMIC handle
 *
 * @attention All parameters of the struct must be set by the end-user. Otherwise,
 * an error may occur during the Pmic_init() API call
 *
 * @note Once the user sets all struct members, the struct should be passed into
 * Pmic_init() so that the PMIC driver handle can be initialized with the user's
 * desired configurations
 *
 * @param i2cAddr TPS65036x PMIC I2C address
 *
 * @param commHandle Pointer to platform-specific transport layer communication handle
 *
 * @param ioRead Function pointer to platform-specific transport layer read API
 *
 * @param ioWrite Function pointer to platform-specific transport layer write API
 *
 * @param critSecStart Function pointer to platform-specific critical section start API
 *
 * @param critSecStop Function pointer to platform-specific critical section stop API
 */
typedef struct Pmic_CoreCfg_s
{
    uint8_t i2cAddr;
    void *commHandle;
    int32_t (*ioRead)(const struct Pmic_CoreHandle_s *pmicHandle,
                      uint8_t regAddr,
                      uint8_t bufLen,
                      uint8_t *rxBuf);
    int32_t (*ioWrite)(const struct Pmic_CoreHandle_s *pmicHandle,
                       uint8_t regAddr,
                       uint8_t bufLen,
                       const uint8_t *txBuf);
    void (*critSecStart)(void);
    void (*critSecStop)(void);
} Pmic_CoreCfg_t;

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Initialize the PMIC interface handle and validate that the TPS65036x
 * PMIC is on the I2C bus. This API should be called before calling any other
 * TPS65036x LLD APIs.
 *
 * @param pmicCfg [IN] PMIC handle configuration struct. End-user will input
 * their settings/parameters in this struct to initialize the PMIC handle.
 *
 * @param pmicHandle [OUT] PMIC interface handle.
 *
 * @return Success code if PMIC handle is initialized without issue, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *pmicCfg, Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief De-initialize the PMIC interface handle when the end-user wants to
 * close communication with the PMIC.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC handle is de-initialized, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *pmicHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_H__ */
