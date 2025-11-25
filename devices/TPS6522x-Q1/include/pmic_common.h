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
#ifndef __PMIC_COMMON_H__
#define __PMIC_COMMON_H__

/**
 * @file pmic_common.h

 * @brief This file contains declarations/definitions of common macros/defines,
 * data structures, and APIs used throughout PMIC LLD.
 */

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "pmic_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @anchor Pmic_arraySizeMacro
 * @name PMIC Array Size Macro
 *
 * @brief Macro used to find the size of an array.
 *
 * @{
 */
#define COUNT(x) (sizeof(x) / sizeof(x[0]))
/** @} */

/**
 * @anchor Pmic_commonDefines
 * @name PMIC LLD Common Defines
 *
 * @brief Common defines used throughout PMIC LLD.
 *
 * @{
 */
#define PMIC_CFG_DEACTIVATED ((uint8_t)0U)
#define PMIC_CFG_ACTIVATED   ((uint8_t)1U)
#define PMIC_DISABLE         ((bool)false)
#define PMIC_ENABLE          ((bool)true)
/** @} */


/**
 * @anchor Pmic_invalidValue
 * @name PMIC Invalid Value Definition
 *
 * @brief Used by PMIC LLD to indicate an invalid value.
 *
 * @{
 */
#define PMIC_INVALID_VALUE ((uint8_t)0x00U)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/**
 * @anchor Pmic_Handle
 * @name PMIC handle structure
 *
 * @brief Critical data structure used by almost all driver APIs. It abstracts
 * application/platform/OS-specific functionalities so that the driver can be
 * agnostic of those domains. It also abstracts PMIC device information.
 *
 * @attention Any instance of this data structure must not be modified by users
 * after invoking 'Pmic_init()'. Specifically, members of this structure are not
 * meant to be directly interacted with by the user. Instead, use functions from
 * other modules to interact with the handle.
 *
 * @param devId PMIC device identification.
 *
 * @param devSiRev PMIC device silicon revision.
 *
 * @param nvmCode PMIC device NVM identification.
 *
 * @param nvmRev PMIC device NVM revision.
 *
 * @param commMode Serial communication mode in which this driver will be operating
 * on. For valid values, refer to @ref Pmic_CommMode.
 *
 * @param i2cAddr0 Main I2C address. Used to access user-space registers on the
 * PMIC.
 *
 * @param i2cAddr1 Secondary I2C address. Used to access WDG-space registers on
 * the PMIC.
 *
 * @param i2cAddr2 Tertiary I2C address. Used to access NVM-space registeres on
 * the PMIC.
 *
 * @param maxLoopCnt Maximum number of iterations for loops in PMIC LLD.
 *
 * @param crcEnable Enable or disable serial communication CRC.
 *
 * @param asyncEnable Enable asynchronous serial communication operation. If set
 * to true, the driver shall use the asynchronous read/write hooks to transfer
 * data from/to the PMIC instead of synchronous hooks.
 *
 * @param commHandle0 Primary serial communication handle. Use for single I2C mode
 * or SPI mode.
 *
 * @param commHandle1 Secondary serial communication handle. Only use for dual
 * I2C mode.
 *
 * @param taskHandle Handle to the application layer task that is responsible
 * for configuring, controlling, servicing, and/or interacting with the PMIC
 * device.
 *
 * @param ioRead Function pointer to platform-specific serial communication read
 * API.
 *
 * @param ioWrite Function pointer to platform-specific serial communication write
 * API.
 *
 * @param asyncRxStart Function pointer to platform-specific asynchronous read
 * transfer start API. Typically initiates a DMA read transfer. The DMA typically
 * handles the memory->peripheral and/or peripheral->memory transfer so that the
 * CPU can enter LPM or so that the calling task can be suspended (put into a blocked
 * state).
 *
 * @param asyncTxStart Function pointer to platform-specific asynchronous write
 * transfer start API. Typically initiates a DMA write transfer. The DMA typically
 * handles the memory->peripheral and/or peripheral->memory transfer so that the
 * CPU can enter LPM or so that the calling task can be suspended (put into a blocked
 * state).
 *
 * @param asyncRxAwait Function pointer to platform-specific asynchronous read
 * transfer await API. Typically suspends the calling task so that other tasks
 * can run. After call invocation (i.e., at the end of the API routine), data
 * should be obtained from the PMIC and the task should resume.
 *
 * @param asyncTxAwait Function pointer to platform-specific asynchronous write
 * transfer await API. Typically suspends the calling task so that other tasks
 * can run. After call invocation (i.e., at the end of the API routine),
 * transmission of data to the PMIC should be completed and the task should resume.
 *
 * @param criticalSectionStart Function pointer to OS-specific critical section
 * start API. Typically takes a mutex/semaphore. Invoked by the driver when a
 * shared resource such as a communication bus is required to be used.
 *
 * @param criticalSectionStop Function pointer to OS-specific critical section
 * stop API. Typically releases a mutex/semaphore. Invoked by the driver once a
 * shared resource such as a communication bus is done being used.
 *
 * @param irqResponseCallback Optional function pointer to application-specific
 * response to an interrupt request during the servicing of the PMIC watchdog in
 * Q&A mode. The driver invokes this hook if it detects a PMIC interrupt or fault
 * when sending watchdog answer bytes to the PMIC.
 *
 * @{
 */
typedef struct Pmic_Handle_s {
    uint32_t drvInitStat;
    uint8_t devRev;
    uint8_t devSiRev;
    uint8_t nvmCode;
    uint8_t nvmRev;
    uint8_t commMode;
    uint8_t i2cAddr0;
    uint8_t i2cAddr1;
    uint8_t i2cAddr2;
    uint32_t maxLoopCnt;
    bool crcEnable;
    bool asyncEnable;
    void *commHandle0;
    void *taskHandle;
    int32_t (*ioRead)(
        const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, uint8_t *buffer, uint8_t bufLen);
    int32_t (*ioWrite)(
        const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen);
    int32_t (*asyncRxStart)(
        const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, uint8_t *buffer, uint8_t bufLen);
    int32_t (*asyncTxStart)(
        const struct Pmic_Handle_s *handle, uint8_t page, uint8_t regAddr, const uint8_t *buffer, uint8_t bufLen);
    int32_t (*asyncRxAwait)(const struct Pmic_Handle_s *handle);
    int32_t (*asyncTxAwait)(const struct Pmic_Handle_s *handle);
    void (*criticalSectionStart)(void);
    void (*criticalSectionStop)(void);
    void (*irqResponseCallback)(void);
} Pmic_Handle_t;
/** @} */

/*==========================================================================  */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Checks whether a bit in `validParams` is set. Used by driver APIs to
 * decipher whether a parameter will be processed in their routines.
 *
 * Design: PMICDRV-571
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521, PMICDRV-519
 *               PMICDRV-520
 *
 * @param validParams [IN] Indication of parameters that are valid. Each bit in
 * the variable corresponds to a structure member. If a bit is 0 in `validParams`,
 * the corresponding parameter is invalid and will not be processed by the calling
 * function. Else, if a bit is 1, the corresponding parameter is valid and will be
 * processed by the calling function.
 *
 * @param bitMask [IN] validParam to check for.
 *
 * @return True if validParam is set, false if validParam is not set.
 */
static bool Pmic_validParamCheck(uint32_t validParams, uint32_t bitMask) {
    return ((validParams & bitMask) != 0U);
}

/**
 * @brief Checks both status and valid parameters.
 *
 * Design: PMICDRV-572
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521, PMICDRV-519
 *               PMICDRV-520
 *
 * @param vpv [IN] Valid parameter value.
 *
 * @param bMask [IN] Valid parameter bit mask. used to check whether the valid parameter is set in 'vpv'.
 *
 * @param status [IN] API checks whether this parameter is equal to LLD success code.
 *
 * @return True if valid parameter is set and status is equal to LLD success code, false otherwise.
 */
static inline bool Pmic_validParamStatusCheck(uint32_t vpv, uint32_t bMask, int32_t status) {
    return ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(vpv, bMask));
}

/**
 * @brief Set a bit field of an 8-bit unsigned integer to a desired value.
 *
 * Design: PMICDRV-575
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param data [OUT] Data holding the bit field to be modified.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @param val [IN] Desired bit field value.
 */
static inline void Pmic_setBitField(uint8_t *data, uint8_t shift, uint8_t mask, uint8_t val) {
    *data = (((*data) & (~mask)) | ((val << shift) & mask));
}

/**
 * @brief Set a bit field of an 8-bit unsigned integer to a desired value, given
 * a boolean.
 *
 * Design: PMICDRV-577
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param data [OUT] Data holding the bit field to be modified.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @param val_b [IN] Desired bit field value. When parameter set to true,
 * bit field value will be set to 1. Otherwise, bit field value will be set to 0.
 */
static inline void Pmic_setBitField_b(uint8_t *data, uint8_t shift, uint8_t mask, bool val_b) {
    const uint8_t val = val_b ? 1U : 0U;
    *data = (((*data) & (~mask)) | ((val << shift) & mask));
}

/**
 * @brief Get a bit field value of an 8-bit unsigned integer.
 *
 * Design: PMICDRV-578
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param data [IN] Data to extract bit field from.
 *
 * @param shift [IN] Target bit field position.
 *
 * @param mask [IN] Target bit field mask.
 *
 * @return Desired bit field value.
 */
static inline uint8_t Pmic_getBitField(uint8_t data, uint8_t shift, uint8_t mask) {
    return ((data & mask) >> shift);
}

/**
 * @brief Get a bit field value of an 8-bit unsigned integer casted as boolean.
 *
 * Design: PMICDRV-580
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-506, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *
 * @param data [IN] Data to extract bit field from.
 *
 * @param shift [IN] Target bit field position.
 *
 * @return Desired bit field value cast as a boolean.
 */
static inline bool Pmic_getBitField_b(uint8_t data, uint8_t shift) {
    const uint8_t bitVal = ((data & (1U << shift)) >> shift);
    return (bitVal == 1U);
}

/**
 * @brief Start a critical section.
 *
 * Design: PMICDRV-573
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-502, PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *               PMICDRV-517, PMICDRV-505, PMICDRV-509
 *
 * @param handle [IN] PMIC interface handle.
 */
static inline void Pmic_criticalSectionStart(const Pmic_Handle_t *handle) {
    if ((handle != NULL) && (handle->criticalSectionStart != NULL)) {
        handle->criticalSectionStart();
    }
}

/**
 * @brief Stop a critical section.
 *
 * Design: PMICDRV-574
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-549, PMICDRV-550, PMICDRV-551
 *               PMICDRV-502, PMICDRV-506, PMICDRV-526, PMICDRV-504, PMICDRV-522, PMICDRV-521
 *               PMICDRV-517, PMICDRV-505, PMICDRV-509
 *
 * @param handle [IN] PMIC interface handle.
 */
static inline void Pmic_criticalSectionStop(const Pmic_Handle_t *handle) {
    if ((handle != NULL) && (handle->criticalSectionStop != NULL)) {
        handle->criticalSectionStop();
    }
}

/**
 * @brief Execute application-specific IRQ response.
 *
 * Design: PMICDRV-683
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-537, PMICDRV-521, PMICDRV-517
 *
 * @param handle [IN] PMIC interface handle.
 */
static inline void Pmic_irqResponseCallback(const Pmic_Handle_t *handle) {
    if ((handle != NULL) && (handle->irqResponseCallback != NULL)) {
        handle->irqResponseCallback();
    }
}

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_COMMON_H__ */
