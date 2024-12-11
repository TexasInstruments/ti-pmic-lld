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
 * @file pmic_common.h
 *
 * @brief This file contains declarations/definitions of common macros/defines,
 * data structures, and APIs used throughout PMIC LLD.
 */
#ifndef __PMIC_COMMON_H__
#define __PMIC_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

/* ========================================================================== */
/*                              Macros & Typedefs                             */
/* ========================================================================== */

/**
 * @anchor Pmic_arraySizeMacro
 * @name PMIC Array Size Macro
 *
 * @brief Macro used to find the size of an array.
 *
 * @{
 */
#define COUNT(x)                    (sizeof(x) / sizeof(x[0]))
/** @} */

/**
 * @anchor Pmic_commonDefines
 * @name PMIC LLD Common Defines
 *
 * @brief Common defines used throughout PMIC LLD.
 *
 * @{
 */
#define PMIC_CFG_DEACTIVATED        ((uint8_t)0U)
#define PMIC_CFG_ACTIVATED          ((uint8_t)1U)
#define PMIC_DISABLE                ((bool)false)
#define PMIC_ENABLE                 ((bool)true)
/** @} */

/**
 * @anchor Pmic_invalidRegAddr
 * @name PMIC Invalid Register Address Definition
 *
 * @brief Used by PMIC LLD to indicate an invalid register address.
 *
 * @{
 */
#define PMIC_INVALID_REGADDR        ((uint8_t)0xFFU)
/** @} */

/**
 * @anchor Pmic_invalidValue
 * @name PMIC Invalid Value Definition
 *
 * @brief Used by PMIC LLD to indicate an invalid value.
 *
 * @{
 */
#define PMIC_INVALID_VALUE          ((uint8_t)0x00U)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreHandle
 * @name TPS65036x PMIC Handle
 *
 * @brief Handle to TPS65036x PMIC that is used as input to all driver APIs.
 *
 * @details The handle contains information related to the TPS65036x PMIC such
 * as revision (device, NVM, silicon) and whether CRC is enabled. It also has
 * function pointers that serve to abstract platform-specific information, like
 * the transport layer write/read APIs and critical section start/stop.
 *
 * @attention Once the PMIC handle is initialized via the Pmic_init() API,
 * end-user must ensure that the handle is unmodified throughout application
 * runtime.
 *
 * @param drvInitStat Driver initialization status. When the driver is initialized,
 * the value comes out to be decimal value 1347242307, hex value 0x504D4943. When
 * converting to ASCII, the value reads "PMIC".
 *
 * @param i2cAddr PMIC device I2C address.
 *
 * @param devRev PMIC device revision identifier.
 *
 * @param nvmId 0x00 - 0xF0 are reserved for TI manufactured NVM variants. 0xF1 - 0xFF
 * are reserved for special use.
 *
 * @param nvmRev NVM revision of the IC.
 *
 * @param siliconRev PMIC silicon revision identifier. SILICON_REV[7:6] - Reserved.
 * SILICON_REV[5:3] - ALR. SILICON_REV[2:0] - Metal.
 *
 * @param crcEnable Indication of whether PMIC CRC is enabled. Used by LLD to determine
 * whether to calculate CRC during communication with PMIC.
 *
 * @param commHandle Pointer to platform-specific transport layer communication handle.
 *
 * @param ioRead Function pointer to platform-specific transport layer read API.
 *
 * @param ioWrite Function pointer to platform-specific transport layer write API.
 *
 * @param critSecStart Function pointer to platform-specific critical section start API.
 *
 * @param critSecStop Function pointer to platform-specific critical section stop API.
 *
 * @param irqResponse Function pointer to application IRQ response. Valid only when
 * servicing the PMIC WDG in Q&A mode.
 */
typedef struct Pmic_CoreHandle_s
{
    uint32_t drvInitStat;
    uint8_t i2cAddr;
    uint8_t devRev;
    uint8_t nvmId;
    uint8_t nvmRev;
    uint8_t siliconRev;
    bool crcEnable;
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
    void (*irqResponse)(void);
} Pmic_CoreHandle_t;

/*==========================================================================  */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Check whether the PMIC handle is valid.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if the PMIC handle is valid, error code otherwise. For
 * valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *pmicHandle);

/**
 * @brief Check whether a validParam is set.
 *
 * @param validParamVal [IN] Set of valid parameters.
 *
 * @param bitMask [IN] validParam to check for.
 *
 * @return True if validParam is set, false if validParam is not set.
 */
static inline bool Pmic_validParamCheck(uint32_t validParamVal, uint32_t bitMask)
{
    return ((validParamVal & bitMask) != 0U);
}

/**
 * @brief Check whether status is equal to the success code and whether a
 * parameter \p vpv (validParam value) has parameter \p bMask (bit mask) set.
 */
#define Pmic_validParamStatusCheck(vpv, bMask, status) \
    (((int32_t)status == PMIC_ST_SUCCESS) && Pmic_validParamCheck((uint32_t)vpv, (uint32_t)bMask))

/**
 * @brief Set a bit field of a register to a desired value.
 *
 * @param regVal [OUT] Pointer to variable holding register value.
 *
 * @param regFieldShift [IN] Target bit field position.
 *
 * @param regFieldMask [IN] Target bit field mask.
 *
 * @param fieldVal [IN] Desired bit field value.
 */
static inline void Pmic_setBitField(
    uint8_t *regVal, uint8_t regFieldShift, uint8_t regFieldMask, uint8_t fieldVal)
{
    *regVal = (((*regVal) & (~regFieldMask)) | ((fieldVal << regFieldShift) & regFieldMask));
}

/**
 * @brief Set a bit field of a register to a desired value, given a boolean.
 *
 * @param regVal [OUT] Pointer to variable holding register value.
 *
 * @param regFieldShift [IN] Target bit field position.
 *
 * @param regFieldMask [IN] Target bit field mask.
 *
 * @param fieldVal_b [IN] Desired bit field value. When parameter set to true,
 * bit field value will be set to 1. Otherwise, bit field value will be set to 0.
 */
static inline void Pmic_setBitField_b(
    uint8_t *regVal, uint8_t regFieldShift, uint8_t regFieldMask, bool fieldVal_b)
{
    const uint8_t fieldVal = fieldVal_b ? 1U : 0U;

    *regVal = (((*regVal) & (~regFieldMask)) | ((fieldVal << regFieldShift) & regFieldMask));
}

/**
 * @brief Get a bit field value of a register.
 *
 * @param regData [IN] Register data/value.
 *
 * @param regFieldShift [IN] Target bit field position.
 *
 * @param regFieldMask [IN] Target bit field mask.
 *
 * @return Desired bit field value.
 */
static inline uint8_t Pmic_getBitField(uint8_t regData, uint8_t regFieldShift, uint8_t regFieldMask)
{
    return ((regData & regFieldMask) >> regFieldShift);
}

/**
 * @brief Get a bit field value of a register cast as boolean.
 *
 * @param regData [IN] Register data/value.
 *
 * @param regFieldShift [IN] Target bit field position.
 *
 * @return Desired bit field value cast as a boolean.
 */
static inline bool Pmic_getBitField_b(uint8_t regData, uint8_t regFieldShift)
{
    const uint8_t bitVal = ((regData & (1U << regFieldShift)) >> regFieldShift);

    return (bitVal == 1U);
}

/**
 * @brief Start a critical section.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 */
static inline void Pmic_criticalSectionStart(const Pmic_CoreHandle_t *pmicHandle)
{
    if ((pmicHandle != NULL) && (pmicHandle->critSecStart != NULL))
    {
        pmicHandle->critSecStart();
    }
}

/**
 * @brief Stop a critical section.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 */
static inline void Pmic_criticalSectionStop(const Pmic_CoreHandle_t *pmicHandle)
{
    if ((pmicHandle != NULL) && (pmicHandle->critSecStop != NULL))
    {
        pmicHandle->critSecStop();
    }
}

/**
 * @brief Execute application-specific IRQ response.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 */
static inline void Pmic_irqResponse(const Pmic_CoreHandle_t *pmicHandle)
{
    if ((pmicHandle != NULL) && (pmicHandle->irqResponse != NULL))
    {
        pmicHandle->irqResponse();
    }
}

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_COMMON_H__ */
