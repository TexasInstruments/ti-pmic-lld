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
#ifndef PMIC_H
#define PMIC_H

/**
 * @file pmic.h
 *
 * @brief PMIC Driver initialization API/interface
 */

/**
 * @brief Application entry point for initialization of PMIC driver.
 *
 * This module contains the necessary functions and macros for initialization
 * and de-initialization of the PMIC driver to allow use of the other modules
 * defined in this driver.
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "pmic_adc.h"
#include "pmic_common.h"
#include "pmic_core.h"
#include "pmic_esm.h"
#include "pmic_fsm.h"
#include "pmic_gpio.h"
#include "pmic_irq.h"
#include "pmic_power.h"
#include "pmic_wdg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                            Macros & Typedefs                              */
/* ========================================================================= */

/**
 * @anchor Pmic_CommMode
 * @name PMIC Communication Mode
 *
 * @{
 */
#define PMIC_INTF_I2C_SINGLE (0U)
#define PMIC_INTF_I2C_DUAL   (1U)
#define PMIC_INTF_SPI        (2U)
#define PMIC_INTF_MIN        ((uint8_t)PMIC_INTF_I2C_SINGLE)
#define PMIC_INTF_MAX        ((uint8_t)PMIC_INTF_SPI)
/** @} */

/**
 * @anchor Pmic_Page
 * @name PMIC Page
 *
 * @brief Each PMIC register resides in a space within the register map. The
 * enumerations below are used to describe the space that a PMIC register belongs
 * to so that the correct handle/address/page could be utilized in user-implemented
 * R/W API hooks.
 *
 * @details The pages PMIC_PAGE_MAIN and PMIC_PAGE_WDG are commonly used. All others
 * are seldomly used.
 *
 * @{
 */
#define PMIC_PAGE_MAIN (0U)
#define PMIC_PAGE_NVM  (1U)
#define PMIC_PAGE_TRIM (2U)
#define PMIC_PAGE_SRAM (3U)
#define PMIC_PAGE_WDG  (4U)
#define PMIC_PAGE_MIN  ((uint8_t)PMIC_PAGE_MAIN)
#define PMIC_PAGE_MAX  ((uint8_t)PMIC_PAGE_WDG)
/** @} */

/**
 * @anchor Pmic_HandleCfgValidParams
 * @name PMIC Handle Configuration Structure Valid Parameters
 *
 * @brief Definitions used to indicate valid parameters of `Pmic_HandleCfg_t`.
 * Set the `validParams` member of `Pmic_HandleCfg_t` equal to a combination
 * of these defines by using the `OR` operator.
 *
 * @note The driver can be operated in different modes. Each mode has a different
 * set of valid parameters. The following defines are provided as a convenience:
 * (1) PMIC_SINGLE_I2C_OPERATION_VALID, (2) PMIC_DUAL_I2C_OPERATION_VALID, (3)
 * PMIC_SPI_OPERATION_VALID, (4) PMIC_ASYNC_SPI_OPERATION_VALID.
 *
 * @{
 */
#define PMIC_CFG_INIT_COMM_MODE_VALID              (1UL << 0U)
#define PMIC_CFG_INIT_I2C_ADDR0_VALID              (1UL << 1U)
#define PMIC_CFG_INIT_I2C_ADDR1_VALID              (1UL << 2U)
#define PMIC_CFG_INIT_I2C_ADDR2_VALID              (1UL << 3U)
#define PMIC_INIT_MAX_LOOP_CNT_VALID               (1UL << 4U)
#define PMIC_CRC_ENABLE_0_VALID                    (1UL << 5U)
#define PMIC_CFG_INIT_ASYNC_ENABLE_VALID           (1UL << 6U)
#define PMIC_CFG_INIT_COMM_HANDLE_0_VALID          (1UL << 7U)
#define PMIC_CFG_INIT_TASK_HANDLE_VALID            (1UL << 8U)
#define PMIC_CFG_INIT_IO_READ_VALID                (1UL << 9U)
#define PMIC_CFG_INIT_IO_WRITE_VALID               (1UL << 10U)
#define PMIC_CFG_INIT_ASYNC_RX_START_VALID         (1UL << 11U)
#define PMIC_CFG_INIT_ASYNC_TX_START_VALID         (1UL << 12U)
#define PMIC_CFG_INIT_ASYNC_RX_AWAIT_VALID         (1UL << 13U)
#define PMIC_CFG_INIT_ASYNC_TX_AWAIT_VALID         (1UL << 14U)
#define PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID (1UL << 15U)
#define PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID  (1UL << 16U)
#define PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID  (1UL << 17U)
#define PMIC_CFG_INIT_RETRY_CNT_VALID              (1UL << 18U)
#define PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID      (1UL << 19U)
#define PMIC_CFG_INIT_TIMER_WAIT_MS_VALID          (1UL << 20U)
#define PMIC_COMM_HANDLE_1_VALID                   (1UL << 21U)
#define PMIC_CRC_ENABLE_1_VALID                    (1UL << 22U)
#define PMIC_SINGLE_I2C_OPERATION_VALID (\
    PMIC_CFG_INIT_COMM_MODE_VALID |\
    PMIC_CFG_INIT_I2C_ADDR0_VALID |\
    PMIC_CFG_INIT_I2C_ADDR1_VALID |\
    PMIC_CRC_ENABLE_0_VALID |\
    PMIC_CFG_INIT_COMM_HANDLE_0_VALID |\
    PMIC_CFG_INIT_IO_READ_VALID |\
    PMIC_CFG_INIT_IO_WRITE_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |\
    PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID |\
    PMIC_CFG_INIT_RETRY_CNT_VALID |\
    PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID |\
    PMIC_CFG_INIT_TIMER_WAIT_MS_VALID)
#define PMIC_DUAL_I2C_OPERATION_VALID     (\
    PMIC_CFG_INIT_COMM_MODE_VALID |\
    PMIC_CFG_INIT_I2C_ADDR0_VALID |\
    PMIC_CFG_INIT_I2C_ADDR1_VALID |\
    PMIC_CRC_ENABLE_0_VALID |\
    PMIC_CRC_ENABLE_1_VALID |\
    PMIC_CFG_INIT_COMM_HANDLE_0_VALID |\
    PMIC_COMM_HANDLE_1_VALID |\
    PMIC_CFG_INIT_IO_READ_VALID |\
    PMIC_CFG_INIT_IO_WRITE_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |\
    PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID |\
    PMIC_CFG_INIT_RETRY_CNT_VALID |\
    PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID |\
    PMIC_CFG_INIT_TIMER_WAIT_MS_VALID)
#define PMIC_SPI_OPERATION_VALID          (\
    PMIC_CFG_INIT_COMM_MODE_VALID |\
    PMIC_CRC_ENABLE_0_VALID |\
    PMIC_CFG_INIT_COMM_HANDLE_0_VALID |\
    PMIC_CFG_INIT_IO_READ_VALID |\
    PMIC_CFG_INIT_IO_WRITE_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |\
    PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID |\
    PMIC_CFG_INIT_RETRY_CNT_VALID |\
    PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID |\
    PMIC_CFG_INIT_TIMER_WAIT_MS_VALID)
#define PMIC_ASYNC_SPI_OPERATION_VALID    (\
    PMIC_CFG_INIT_COMM_MODE_VALID |\
    PMIC_CRC_ENABLE_0_VALID |\
    PMIC_CFG_INIT_ASYNC_ENABLE_VALID |
    PMIC_CFG_INIT_COMM_HANDLE_0_VALID |
    PMIC_CFG_INIT_TASK_HANDLE_VALID |
    PMIC_CFG_INIT_ASYNC_RX_START_VALID |\
    PMIC_CFG_INIT_ASYNC_TX_START_VALID |\
    PMIC_CFG_INIT_ASYNC_RX_AWAIT_VALID |\
    PMIC_CFG_INIT_ASYNC_TX_AWAIT_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_START_VALID |\
    PMIC_CFG_INIT_CRITICAL_SECTION_STOP_VALID |\
    PMIC_CFG_INIT_IRQ_RESPONSE_CALLBACK_VALID |\
    PMIC_CFG_INIT_RETRY_CNT_VALID |\
    PMIC_CFG_INIT_RETRY_INTERVAL_MS_VALID |\
    PMIC_CFG_INIT_TIMER_WAIT_MS_VALID)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_HandleCfg
 * @name PMIC handle configuration structure
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
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_HandleCfgValidParams.
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
 * @param retryCnt Upon communication related errors (bus error or CRC error),
 * PMIC LLD attempts to retry the failed transaction up to `retryCnt` times before
 * returning an error code to the calling application.
 *
 * @param retryIntervalMs LLD waits this configured amount of time (milliseconds)
 * between retry attempts using the `timerWaitMs()` hook.
 *
 * @param maxLoopCnt Maximum number of iterations for loops in PMIC LLD.
 *
 * @param crcEnable0 Enable or disable serial communication CRC on I2C1 or SPI.
 *
 * @param crcEnable1 Enable or disable serial communication CRC on I2C2. Unused
 * in single I2C mode and SPI mode.
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
 * @param timerWaitMs Function pointer to platform-specific timer-based wait API.
 * Upon invocation, the user-implemented hook waits a specified period of time
 * (milliseconds) before returning control to the caller.
 *
 * @{
 */
typedef struct Pmic_HandleCfg_s {
    uint32_t validParams;
    uint8_t commMode;
    uint8_t i2cAddr0;
    uint8_t i2cAddr1;
    uint8_t i2cAddr2;
    uint32_t retryCnt;
    uint32_t retryIntervalMs;
    uint32_t maxLoopCnt;
    bool crcEnable0;
    bool crcEnable1;
    bool asyncEnable;
    void *commHandle0;
    void *commHandle1;
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
    void (*criticalSectionStart)(uint8_t resource);
    void (*criticalSectionStop)(uint8_t resource);
    void (*irqResponseCallback)(void);
    void (*timerWaitMs)(uint32_t ms);
} Pmic_HandleCfg_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */
/**
 * @brief Initialize a PMIC handle instance for PMIC LLD. Reads the PMIC device
 * for information and stores obtained data in the handle instance.
 *
 * Design: PMICDRV-568
 * Architecture: PMICDRV-500, PMICDRV-501, PMICDRV-502, PMICDRV-504, PMICDRV-506,
 *               PMICDRV-508, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-524,
 *               PMICDRV-525, PMICDRV-527, PMICDRV-528, PMICDRV-545, PMICDRV-547,
 *               PMICDRV-549, PMICDRV-551
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
 *
 * @param handle [OUT] PMIC interface handle.
 *
 * @param config [IN] Used to set desired PMIC handle configurations. For
 * more information on handle configurations, refer to @ref Pmic_HandleCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC handle instance has been initialized. Error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *config);

/**
 * @brief De-initialize the PMIC interface handle when the end-user wants to
 * close communication with the PMIC.
 *
 * Design: PMICDRV-569
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-551
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
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
 * Design: PMICDRV-570
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-520, PMICDRV-521,
 *               PMICDRV-522, PMICDRV-526, PMICDRV-545, PMICDRV-551
 *
 * @note This function does not automatically log diagnostic information upon
 * encountering errors or warnings (if any). The caller must invoke `Pmic_logStatus()`
 * to log the status if such information is desired.
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
