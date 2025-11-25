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
#ifndef PMIC_STATUS_H
#define PMIC_STATUS_H

/**
 * @file pmic_status.h
 *
 * @brief PMIC Driver status and error code definitions
 *
 * This header contains all status codes and error codes used by the PMIC
 * driver. It is included early in the include chain to ensure error codes
 * are available to all other headers and source files.
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                           Macros & Typedefs                               */
/* ========================================================================= */
/**
 * @anchor Pmic_ErrorCodes
 * @name PMIC Error Codes
 *
 * @brief Error codes returned by PMIC APIs.
 *
 * @note Application code should check all `Pmic_*` functions which return a
 * status code to verify that `PMIC_ST_SUCCESS` was returned, all other status
 * codes indicate that the requested operation did not succeed.
 *
 * **Common "User Error" Status Codes**
 *
 * The following status codes indicate an error in the expected input to an API
 * call and typically indicate a change is required in application code:
 *
 * - **PMIC_ST_ERR_INV_HANDLE**: Indicates that the `Pmic_Handle_t` passed
 *   to the API call is not valid. Ensure that the `Pmic_HandleCfg_t` has been
 *   properly configured and that `Pmic_init()` has been called.
 *
 * - **PMIC_ST_ERR_NULL_PARAM**: Indicates that a pointer type parameter needed
 *   by the API call was NULL. This should not happen under normal
 *   circumstances and likely indicates an unexpected error in application
 *   code. If the application is intentionally passing a NULL parameter, ensure
 *   that the relevant `validParam` bit is not set for that parameter.
 *
 * - **PMIC_ST_ERR_NULL_FPTR**: Like `PMIC_ST_ERR_NULL_PARAM`, but for function
 *   pointers specifically. This will generally only occur when performing
 *   `Pmic_init()` if the critical section or communications API function
 *   pointers are not set up correctly or not provided.
 *
 * - **PMIC_ST_ERR_INV_PARAM**: Indicates that one of the parameters necessary
 *   for an API call had an invalid value, refer to the documentation for the
 *   relevant function to find the valid values for each parameter.
 *
 * **Common "Communications Error" Status Codes**
 *
 * The following status codes indicate an error in the communication layer
 * between the MCU and the PMIC, and may be addressed by a retry, assuming the
 * underlying communications layer is functional.
 *
 * - **PMIC_ST_ERR_I2C_COMM_FAIL**: Indicates I2C comms. failure. Retry a limited
 *   number of times in case of spurious failure.
 *
 * - **PMIC_ST_ERR_SPI_COMM_FAIL**: Indicates SPI comms. failure. Retry a limited
 *   number of times in case of spurious failure.
 *
 * - **PMIC_ST_ERR_DATA_IO_CRC**: Indicates that the PMIC rejected the I/O
 *   request due to a CRC failure. This likely indicates an error within the
 *   PMIC driver, a misconfiguration of PMIC CRC parameters, or a spurious
 *   failure of the communications layer. Retry a limited number of times in
 *   case of spurious failure.
 *
 * **Other Status Codes**
 *
 * Other status codes are for more specific errors which may occur in one of
 * the given submodules of the PMIC driver. The user is referred to that module
 * for more detail.
 *
 * @{
 */
#define PMIC_ST_SUCCESS                                       (-((int32_t)0))
#define PMIC_ST_ERR_INV_HANDLE                                (-((int32_t)1))
#define PMIC_ST_ERR_NULL_PARAM                                (-((int32_t)2))
#define PMIC_ST_ERR_INV_PARAM                                 (-((int32_t)3))
#define PMIC_ST_ERR_INV_DEVICE                                (-((int32_t)4))
#define PMIC_ST_ERR_NULL_FPTR                                 (-((int32_t)5))
#define PMIC_ST_ERR_INSUFFICIENT_CFG                          (-((int32_t)7))
#define PMIC_ST_ERR_I2C_COMM_FAIL                             (-((int32_t)8))
#define PMIC_ST_ERR_SPI_COMM_FAIL                             (-((int32_t)9))
#define PMIC_ST_ERR_DATA_IO_CRC                               (-((int32_t)10))
#define PMIC_ST_ERR_INTF_SETUP_FAILED                         (-((int32_t)11))
#define PMIC_ST_ERR_COMM_INTF_INIT_FAIL                       (-((int32_t)12))
#define PMIC_ST_ERR_FAIL                                      (-((int32_t)13))
#define PMIC_ST_ERR_NOT_SUPPORTED                             (-((int32_t)14))
#define PMIC_ST_WARN_INV_DEVICE_ID                            (-((int32_t)40))
#define PMIC_ST_WARN_NO_IRQ_REMAINING                         (-((int32_t)41))
#define PMIC_ST_DEFAULT_DATA                                  (-((int32_t)100))
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* PMIC_STATUS_H */
