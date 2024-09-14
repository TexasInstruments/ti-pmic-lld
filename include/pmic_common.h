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
#define COUNT(x)            (sizeof(x) / sizeof(x[0]))
/** @} */

/**
 * @anchor Pmic_commonDefines
 * @name PMIC LLD Common Defines
 *
 * @brief Common defines used throughout PMIC LLD.
 *
 * @{
 */
#define PMIC_DISABLE        ((bool)false)
#define PMIC_ENABLE         ((bool)true)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_CoreHandle
 * @name TPS65385x PMIC Handle
 *
 * @brief Handle to TPS65385x PMIC that is used as input to all driver APIs.
 *
 * @details The handle contains information related to the TPS65385x PMIC and API
 * hooks that serve to abstract platform-specific information, such as the transport
 * layer write/read APIs and critical section start/stop.
 *
 * @attention Once a struct of type Pmic_CoreHandle_t is initialized via the Pmic_init()
 * API, end-user must ensure members of the struct are unchanged throughout application
 * runtime.
 *
 * @param drvInitStat Driver initialization status. When the driver is initialized,
 * the value comes out to be decimal value 1347242307, hex value 0x504D4943. When
 * converting to ASCII, the value reads "PMIC".
 *
 * @param devId Binary representation of the last two digits of the part number.
 *
 * @param devRev PMIC device revision identifier.
 *
 * @param commHandle Pointer to platform-specific transport layer communication handle.
 *
 * @param ioTransfer Function pointer to platform-specific SPI read/write API.
 * Platform-specific API must be capable of transmiting 24 bits and receiving 24
 * bits at once (a single SPI frame). The length parameter is specified in
 * number of bytes transmitted.
 *
 * @param critSecStart Function pointer to platform-specific critical section start API.
 *
 * @param critSecStop Function pointer to platform-specific critical section stop API.
 *
 * @param irqResponse Function pointer to application-specific response to detected PMIC
 * IRQ. Upon each SPI transaction, the driver checks the PMIC status for faults/IRQs; if
 * at least one is detected, this API hook will be called in response.
 */
typedef struct Pmic_CoreHandle_s
{
    uint32_t drvInitStat;
    uint8_t devId;
    uint8_t devRev;
    void *commHandle;
    int32_t (*ioTransfer)(const struct Pmic_CoreHandle_s *pmicHandle, uint32_t *txBuf, uint32_t *rxBuf, uint8_t length);
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
 * @param fieldVal_b [IN] Desired bit field value. When parameter set to true,
 * bit field value will be set to 1. Otherwise, bit field value will be set to 0.
 */
static inline void Pmic_setBitField_b(
    uint8_t *regVal, uint8_t regFieldShift, bool fieldVal_b)
{
    const uint8_t fieldVal = fieldVal_b ? 1U : 0U;
    const uint8_t fieldMask = (1U << regFieldShift) & 0xFFU;

    *regVal = ((*regVal & ~fieldMask) | ((fieldVal << regFieldShift) & fieldMask));
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
