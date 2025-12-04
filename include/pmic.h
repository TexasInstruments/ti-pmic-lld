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
#ifndef PMIC_H
#define PMIC_H

/**
 * @file pmic.h
 *
 * @brief PMIC LLD entry point for initialization of PMIC driver. This module
 * contains the necessary functions and macros for initialization and
 * de-initialization of the PMIC driver to allow use of the other modules
 * defined in this driver.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"
#include "pmic_core.h"
#include "pmic_esm.h"
#include "pmic_gpio.h"
#include "pmic_io.h"
#include "pmic_wdg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

/**
 * @anchor Pmic_HandleCfgValidParams
 * @name PMIC Handle Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_HandleCfg_t`.
 * Set the `validParams` member of `Pmic_HandleCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @note The driver can be operated in different modes: synchronous and asynchronous.
 * Each mode has a different set of valid parameters. The following defines are
 * provided as a convenience: (1) PMIC_SYNC_OPERATION_VALID, (2) PMIC_ASYNC_OPERATION_VALID.
 *
 * @{
 */
#define PMIC_CRC_ENABLE_VALID             (1U << 0U)
#define PMIC_ASYNC_ENABLE_VALID           (1U << 1U)
#define PMIC_COMM_HANDLE_0_VALID          (1U << 2U)
#define PMIC_TASK_HANDLE_VALID            (1U << 3U)
#define PMIC_IO_READ_VALID                (1U << 4U)
#define PMIC_IO_WRITE_VALID               (1U << 5U)
#define PMIC_ASYNC_RX_START_VALID         (1U << 6U)
#define PMIC_ASYNC_TX_START_VALID         (1U << 7U)
#define PMIC_ASYNC_RX_AWAIT_VALID         (1U << 8U)
#define PMIC_ASYNC_TX_AWAIT_VALID         (1U << 9U)
#define PMIC_CRITICAL_SECTION_START_VALID (1U << 10U)
#define PMIC_CRITICAL_SECTION_STOP_VALID  (1U << 11U)
#define PMIC_IRQ_RESPONSE_CALLBACK_VALID  (1U << 12U)
#define PMIC_SYNC_OPERATION_VALID         (\
    PMIC_CRC_ENABLE_VALID |\
    PMIC_COMM_HANDLE_0_VALID |\
    PMIC_IO_READ_VALID |\
    PMIC_IO_WRITE_VALID |\
    PMIC_CRITICAL_SECTION_START_VALID |\
    PMIC_CRITICAL_SECTION_STOP_VALID)
#define PMIC_ASYNC_OPERATION_VALID        (\
    PMIC_CRC_ENABLE_VALID |\
    PMIC_ASYNC_ENABLE_VALID |\
    PMIC_COMM_HANDLE_0_VALID |\
    PMIC_TASK_HANDLE_VALID |\
    PMIC_ASYNC_RX_START_VALID |\
    PMIC_ASYNC_TX_START_VALID |\
    PMIC_ASYNC_RX_AWAIT_VALID |\
    PMIC_ASYNC_TX_AWAIT_VALID |\
    PMIC_CRITICAL_SECTION_START_VALID |\
    PMIC_CRITICAL_SECTION_STOP_VALID)
/** @} */

/* ========================================================================== */
/*                            Structures and Enums                            */
/* ========================================================================== */

/**
 * @anchor Pmic_HandleCfg
 * @name PMIC Handle Configuration Structure
 *
 * @brief Used to initialize a PMIC LLD handle instance. Passed as an input to
 * `Pmic_init()`. The configurations encapsulated by this structure shall be
 * used to initialize the PMIC handle. For more information regarding the PMIC
 * handle, refer to @ref Pmic_Handle.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameters, see @ref Pmic_HandleCfgValidParams.
 *
 * @param crcEnable Enable or disable serial communication CRC.
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
typedef struct Pmic_HandleCfg_s {
    uint32_t validParams;
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
} Pmic_HandleCfg_t;

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */

/**
 * @brief Initialize a PMIC handle instance for PMIC LLD. Reads the PMIC device
 * for information and stores obtained data in the handle instance.
 *
 * @param handle [OUT] PMIC interface handle.
 *
 * @param handleCfg [IN] Used to set desired PMIC handle configurations. For
 * more information on handle configurations, refer to @ref Pmic_HandleCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC handle instance has been initialized. Error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg);

/**
 * @brief De-initialize an existing PMIC handle instance.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if PMIC handle instance has been de-initialized. Error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_deinit(Pmic_Handle_t *handle);

/**
 * @brief Validate a PMIC handle instance for proper initialization and
 * construction. Utilized by all public LLD APIs that accept a handle as input
 * parameter to help prevent corrupt handle usage. Can be used in the application
 * layer to check the handle independently.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if PMIC handle instance is valid, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_checkHandle(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_H */
