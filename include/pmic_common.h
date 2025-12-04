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
#ifndef PMIC_COMMON_H
#define PMIC_COMMON_H

/**
 * @file pmic_common.h
 *
 * @brief APIs and macros/typedefs commonly used across PMIC LLD.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                           Macros and Defines                               */
/* ========================================================================== */

/**
 * @anchor Pmic_ErrorCodes
 * @name PMIC Error Codes
 *
 * @brief Error codes returned by PMIC APIs. Application code should check the
 * return code of PMIC LLD APIs (if any) and handle errors appropriately.
 *
 * @{
 */
#define PMIC_ST_SUCCESS               (-((int32_t)0))
#define PMIC_ST_ERR_INV_HANDLE        (-((int32_t)1))
#define PMIC_ST_ERR_NULL_PARAM        (-((int32_t)2))
#define PMIC_ST_ERR_INV_PARAM         (-((int32_t)3))
#define PMIC_ST_ERR_NULL_FPTR         (-((int32_t)4))
#define PMIC_ST_ERR_INSUFFICIENT_CFG  (-((int32_t)5))
#define PMIC_ST_ERR_SPI_COMM_FAIL     (-((int32_t)6))
#define PMIC_ST_ERR_I2C_COMM_FAIL     (-((int32_t)7))
#define PMIC_ST_ERR_DATA_IO_CRC       (-((int32_t)8))
#define PMIC_ST_ERR_NOT_SUPPORTED     (-((int32_t)9))
#define PMIC_ST_WARN_NO_IRQ_REMAINING (-((int32_t)100))
/** @} */

/**
 * @anchor Pmic_ArraySizeMacro
 * @name PMIC Array Size Macro
 *
 * @brief Macro used to find the size of a static array.
 *
 * @{
 */
#define COUNT(x) (sizeof(x)/sizeof(x[0]))
/** @} */

/**
 * @anchor Pmic_EnableDisable
 * @name PMIC Enable/Disable
 *
 * @brief Broadly used to enable or disable a feature.
 *
 * @{
 */
#define PMIC_ENABLE  ((bool)true)
#define PMIC_DISABLE ((bool)false)
/** @} */

/* ========================================================================== */
/*                          Structures and Enums                              */
/* ========================================================================== */

/**
 * @anchor Pmic_Handle
 * @name PMIC Core Handle
 *
 * @brief Handle used to abstract PMIC device information as well as
 * user-implemented functionalities that are platform/OS-specific.
 *
 * @attention This structure is a central resource used by almost all LLD APIs
 * and must be initialized via 'Pmic_init()' before it can be used by other LLD
 * APIs. End-users should not modify the contents of this structure after it has
 * been initialized.
 *
 * @param drvInitStat Driver initialization status. Used by LLD as a measure to
 * prevent corrupted handle usage.
 *
 * @param devId PMIC device identifier.
 *
 * @param nvmId PMIC NVM identifier.
 *
 * @param nvmRev PMIC NVM revision.
 *
 * @param siRev PMIC silicon revision.
 *
 * @param crcEnable Status of whether serial communication CRC is enabled. Set to true
 * if enabled, false otherwise.
 *
 * @param asyncEnable Enable asynchronous serial communication operation. If set
 * to true, the driver shall use the asynchronous read/write hooks to transfer
 * data from/to the PMIC instead of synchronous hooks.
 *
 * @param commHandle0 Primary serial communication handle.
 *
 * @param taskHandle Handle to the application layer task that is responsible
 * for configuring, controlling, servicing, and/or interacting with the PMIC
 * device.
 *
 * @param ioRead Function pointer to platform-specific synchronous serial
 * communication read API.
 *
 * @param ioWrite Function pointer to platform-specific synchronous serial
 * communication write API.
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
 */
typedef struct Pmic_Handle_s {
    uint32_t drvInitStat;
    uint8_t devId;
    uint8_t nvmId;
    uint8_t nvmRev;
    uint8_t siRev;
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

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Check whether one or multiple validParams are set.
 *
 * @param validParamVal [IN] Set of valid parameters.
 *
 * @param bitMask [IN] validParam(s) to check for.
 *
 * @return True if validParams are set, false if validParams are not set.
 */
static inline bool Pmic_validParamCheck(uint32_t validParamVal, uint32_t bitMask) {
    return ((validParamVal & bitMask) != 0U);
}

/**
 * @brief Checks whether a parameter is valid and whether the status code is
 * equal to LLD success code.
 *
 * @param vpv [IN] Valid parameter value. Each bit represents whether a parameter
 * is valid.
 *
 * @param bMask [IN] valid parameter(s) to check for.
 *
 * @param status [IN] The API checks whether the value of this parameter is
 * equal to the PMIC LLD success code.
 *
 * @return True if the status code is equal to the LLD success code and the
 * parameter is valid.
 */
#define Pmic_validParamStatusCheck(vpv, bMask, status) ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(vpv, bMask))

/**
 * @brief Start a critical section for PMIC operations if the critical
 * section start function pointer is not NULL.
 *
 * @param handle Pointer to the PMIC core handle structure.
 * @return void No return value.
 */
static inline void Pmic_criticalSectionStart(const Pmic_Handle_t *handle) {
    if ((handle != NULL) && (handle->criticalSectionStart != NULL)) {
        handle->criticalSectionStart();
    }
}

/**
 * @brief Stop a critical section for PMIC operations if the critical
 * section stop function pointer is not NULL.
 *
 * @param handle Pointer to the PMIC core handle structure.
 *
 * @return void No return value.
 */
static inline void Pmic_criticalSectionStop(const Pmic_Handle_t *handle) {
    if ((handle != NULL) && (handle->criticalSectionStop != NULL)) {
        handle->criticalSectionStop();
    }
}

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired value.
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param mask [IN] Bit field mask.
 *
 * @param value [IN] Desired bit field value to set.
 */
static inline void Pmic_setBitField(uint8_t *regData, uint8_t shift, uint8_t mask, uint8_t value) {
    *regData = ((*regData & ~mask) | ((value << shift) & mask));
}

/**
 * @brief Set the value of a bitfield based on the "NAME" of the field, rather
 * than providing individual SHIFT/MASK values. A simplified version of
 * `Pmic_setBitField()`.
 *
 * @param reg [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param name [IN] Bit field name.
 *
 * @param val [IN] Desired value to set the bit field to.
 */
#define Pmic_setBitFieldByName(reg, name, val) (Pmic_setBitField(reg, name##_SHIFT, name##_MASK, val))

/**
 * @brief Sets the bit field of an 8-bit unsigned integer to the desired boolean
 * value.
 *
 * @param regData [OUT] The API modifies the desired bit field of the value held
 * at this address.
 *
 * @param shift [IN] Bit field position.
 *
 * @param value [IN] Bit field value (either true or false).
 */
static inline void Pmic_setBitField_b(uint8_t *regData, uint8_t shift, bool value) {
    Pmic_setBitField(regData, shift, (uint8_t)(1U << shift), value ? 1U : 0U);
}

/**
 * @brief Get desired bit field of an 8-bit unsigned integer.
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @param mask [IN] Bit field mask.
 *
 * @return Value of the desired bit field.
 */
static inline uint8_t Pmic_getBitField(uint8_t regData, uint8_t shift, uint8_t mask) {
    return ((regData & mask) >> shift);
}

/**
 * @brief Retrieve the value of a bitfield based on the "NAME" of the field,
 * rather than providing individual SHIFT/MASK values. A simplified version of
 * `Pmic_getBitField()`.
 *
 * @param reg [IN] The API gets the desired bit field from this value.
 *
 * @param name [IN] Bit field name.
 *
 * @return Value of the desired bit field.
 */
#define Pmic_getBitFieldByName(reg, name) (Pmic_getBitField(reg, name##_SHIFT, name##_MASK))

/**
 * @brief Get desired bit field of an 8-bit unsigned integer, casted as boolean.
 *
 * @param regData [IN] The API gets the desired bit field from this value.
 *
 * @param shift [IN] Bit field location.
 *
 * @return Value of the desired bit field.
 */
static inline bool Pmic_getBitField_b(uint8_t regData, uint8_t shift) {
    return Pmic_getBitField(regData, shift, (uint8_t)(1U << shift)) == 1U;
}

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_COMMON_H */
