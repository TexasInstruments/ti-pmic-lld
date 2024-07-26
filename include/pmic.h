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
#ifndef __PMIC_H__
#define __PMIC_H__

/**
 * @file pmic.h
 * @brief PMIC Driver initialization API/Interface
 */

/**
 * @defgroup DRV_PMIC_MODULE PMIC Driver Entry Point
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
#include <stdint.h>

#include "pmic_core.h"
#include "pmic_io.h"
#include "pmic_irq.h"
#include "pmic_wdg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                             Macros & Typedefs                             */
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
 * - **PMIC_ST_ERR_INV_HANDLE**: Indicates that the `Pmic_CoreHandle_t` passed
 *   to the API call is not valid. Ensure that the `Pmic_CoreCfg_t` has been
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
#define PMIC_ST_ERR_INV_SUBSYSTEM                             (-((int32_t)6))
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

/**
 * @anchor Pmic_EnableDisable
 * @name PMIC Enable/Disable features control
 *
 * @{
 */
#define PMIC_ENABLE     ((bool)true)
#define PMIC_DISABLE    ((bool)false)
/** @} */

/**
 * @anchor Pmic_DeviceType
 * @name PMIC Device type
 *
 * @{
 */
#define PMIC_DEV_BB_TPS65386X (0U)
/** @} */

/**
 * @anchor Pmic_CommMode
 * @name PMIC Communication Mode
 *
 * @{
 */
#define PMIC_INTF_I2C_SINGLE (0U)
#define PMIC_INTF_I2C_DUAL   (1U)
#define PMIC_INTF_SPI        (2U)
/** @} */

/**
 * @anchor Pmic_I2CSpeedSel
 * @name PMIC Select I2C Speed
 *
 * @note Set I2C Master before switching the I2C speed to HS/Standard Mode, I2C
 * Master has to set/reset I2C1_HS/I2C2_HS bit field accordingly then only I2C
 * Master can communicate with PMIC in HS/Standard Mode
 *
 * @{
 */
#define PMIC_I2C_STANDARD_MODE  (0U)
#define PMIC_I2C_FORCED_HS_MODE (1U)
/** @} */

/**
 * @anchor Pmic_InstType
 * @name PMIC Instance Type
 *
 * @note `PMIC_NVM_INST` is valid only for reading CRC status from Page-1 using
 * NVM Slave Address, and is only valid while calling the `pFnPmicCommIoRead()`
 * API.
 *
 * @{
 */
#define PMIC_MAIN_INST  (uint32_t)(1U << 0U)
#define PMIC_QA_INST    (uint32_t)(1U << 1U)
#define PMIC_NVM_INST   (uint32_t)(1U << 2U)
/** @} */

/**
 * @anchor Pmic_ValidParamCfg
 * @name  PMIC Config Structure Param Bits
 * @brief The `validParams` values to be used when checking configuration of
 * the `Pmic_CoreCfg_t` type.
 *
 * @{
 */
#define PMIC_CFG_DEVICE_TYPE_VALID   (0U)
#define PMIC_CFG_COMM_MODE_VALID     (1U)
#define PMIC_CFG_SLAVEADDR_VALID     (2U)
#define PMIC_CFG_QASLAVEADDR_VALID   (3U)
#define PMIC_CFG_NVMSLAVEADDR_VALID  (4U)
#define PMIC_CFG_COMM_HANDLE_VALID   (5U)
#define PMIC_CFG_QACOMM_HANDLE_VALID (6U)
#define PMIC_CFG_COMM_IO_RD_VALID    (7U)
#define PMIC_CFG_COMM_IO_WR_VALID    (8U)
#define PMIC_CFG_CRITSEC_START_VALID (9U)
#define PMIC_CFG_CRITSEC_STOP_VALID  (10U)
#define PMIC_CFG_I2C1_SPEED_VALID    (11U)
#define PMIC_CFG_I2C2_SPEED_VALID    (12U)
/** @} */

/**
 * @anchor Pmic_ValidParamCfgShift
 * @name PMIC Config Structure Param Bit Shift Values
 *
 * Application can use below shifted values to set the validParam struct member
 * defined in Pmic_CoreCfg_t structure
 *
 * @{
 */
#define PMIC_CFG_DEVICE_TYPE_VALID_SHIFT    (1U << PMIC_CFG_DEVICE_TYPE_VALID)
#define PMIC_CFG_COMM_MODE_VALID_SHIFT      (1U << PMIC_CFG_COMM_MODE_VALID)
#define PMIC_CFG_SLAVEADDR_VALID_SHIFT      (1U << PMIC_CFG_SLAVEADDR_VALID)
#define PMIC_CFG_QASLAVEADDR_VALID_SHIFT    (1U << PMIC_CFG_QASLAVEADDR_VALID)
#define PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT   (1U << PMIC_CFG_NVMSLAVEADDR_VALID)
#define PMIC_CFG_COMM_HANDLE_VALID_SHIFT    (1U << PMIC_CFG_COMM_HANDLE_VALID)
#define PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT  (1U << PMIC_CFG_QACOMM_HANDLE_VALID)
#define PMIC_CFG_COMM_IO_RD_VALID_SHIFT     (1U << PMIC_CFG_COMM_IO_RD_VALID)
#define PMIC_CFG_COMM_IO_WR_VALID_SHIFT     (1U << PMIC_CFG_COMM_IO_WR_VALID)
#define PMIC_CFG_CRITSEC_START_VALID_SHIFT  (1U << PMIC_CFG_CRITSEC_START_VALID)
#define PMIC_CFG_CRITSEC_STOP_VALID_SHIFT   (1U << PMIC_CFG_CRITSEC_STOP_VALID)
#define PMIC_CFG_I2C1_SPEED_VALID_SHIFT     (1U << PMIC_CFG_I2C1_SPEED_VALID)
#define PMIC_CFG_I2C2_SPEED_VALID_SHIFT     (1U << PMIC_CFG_I2C2_SPEED_VALID)
/** @brief Helper macro to set all `validParams` necessary for configuring I2C
 * based driver. */
#define PMIC_CFG_ALL_I2C_VALID_SHIFT        (\
    PMIC_CFG_DEVICE_TYPE_VALID_SHIFT    |\
    PMIC_CFG_COMM_MODE_VALID_SHIFT      |\
    PMIC_CFG_SLAVEADDR_VALID_SHIFT      |\
    PMIC_CFG_QASLAVEADDR_VALID_SHIFT    |\
    PMIC_CFG_NVMSLAVEADDR_VALID_SHIFT   |\
    PMIC_CFG_I2C1_SPEED_VALID_SHIFT     |\
    PMIC_CFG_I2C2_SPEED_VALID_SHIFT     |\
    PMIC_CFG_COMM_IO_RD_VALID_SHIFT     |\
    PMIC_CFG_COMM_IO_WR_VALID_SHIFT     |\
    PMIC_CFG_COMM_HANDLE_VALID_SHIFT    |\
    PMIC_CFG_QACOMM_HANDLE_VALID_SHIFT  |\
    PMIC_CFG_CRITSEC_START_VALID_SHIFT  |\
    PMIC_CFG_CRITSEC_STOP_VALID_SHIFT)
/** @brief Helper macro to set all `validParams` necessary for configuring SPI
 * based driver. */
#define PMIC_CFG_ALL_SPI_VALID_SHIFT        (\
    PMIC_CFG_DEVICE_TYPE_VALID_SHIFT    |\
    PMIC_CFG_COMM_MODE_VALID_SHIFT      |\
    PMIC_CFG_COMM_IO_RD_VALID_SHIFT     |\
    PMIC_CFG_COMM_IO_WR_VALID_SHIFT     |\
    PMIC_CFG_COMM_HANDLE_VALID_SHIFT    |\
    PMIC_CFG_CRITSEC_START_VALID_SHIFT  |\
    PMIC_CFG_CRITSEC_STOP_VALID_SHIFT)
/** @} */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/
/**
 * @brief PMIC configuration structure.
 *
 * Contains various parameters which are needed to prepare PMIC driver handle
 * using Valid params like, PMIC device type, PMIC interface mode, Slave
 * address, various application defined API function pointers for LLD and
 * Critical sections.
 *
 * Application has to set the corresponding bit in validParams structure member
 * to update the driver with Pmic_CoreCfg_t structure fields.
 *
 * For Example, If the Application needs to configure the PMIC driver
 * `pmicDeviceType` member of the structure, then application has to set
 * `PMIC_CFG_DEVICE_TYPE_VALID_SHIFT` bit of `validParams` struct and then call
 * `Pmic_init()`.
 *
 * @param validParams Controls which parameters below shall be considered by
 * `Pmic_init()`, decided by the combination of @ref Pmic_ValidParamCfgShift.
 *
 * @param instType Driver instance type. For valid values, see @ref
 * Pmic_InstType. This parameter has no corresponding `validParams` shift value
 * as it is always expected to be provided.
 *
 * @param pmicDeviceType PMIC device type. For valid values, see @ref
 * Pmic_DeviceType. Valid only when `PMIC_CFG_DEVICE_TYPE_VALID` bit of
 * `validParams` is set.
 *
 * @param commMode Communications interface mode: Single I2C, Dual I2C or SPI.
 * For valid values, see @ref Pmic_CommMode. Valid only when
 * `PMIC_CFG_COMM_MODE_VALID` bit of `validParams` is set.
 *
 * @param slaveAddr Main Interface Slave Address for I2C. Valid only when
 * `PMIC_CFG_SLAVEADDR_VALID` bit of `validParams` is set. Only necessary for
 * I2C interfaces.
 *
 * @param qaSlaveAddr WDOG QA Interface Slave Address for I2C. Valid only when
 * `PMIC_CFG_QASLAVEADDR_VALID` bit of `validParams` is set. Only necessary for
 * I2C interfaces.
 *
 * @param nvmSlaveAddr NVM Slave Address for I2C. This provides only read
 * access to CRC status of Page-1 Application shall use this slave address to
 * read only CRC status. Application shall not do any write operations using
 * this slave address. Valid only when `PMIC_CFG_NVMSLAVEADDR_VALID` bit of
 * `validParams` is set. Only necessary for I2C interfaces.
 *
 * @param i2c1Speed Configures I2C1 Speed when commMode is Single or Dual I2C.
 * For valid values see, @ref Pmic_I2CSpeedSel. Valid only when
 * `PMIC_CFG_I2C1_SPEED_VALID` bit is set. Only necessary for I2C interfaces.
 *
 * @param i2c2Speed Configures I2C2 Speed when commMode is Dual I2C For valid
 * values, see @ref Pmic_I2CSpeedSel. Valid only when
 * `PMIC_CFG_I2C2_SPEED_VALID` bit is set. Only necessary for I2C interfaces.
 *
 * @param pFnPmicCommIoRead Pointer to I2C/SPI Comm LLD Read Function. Valid
 * only when `PMIC_CFG_COMM_IO_RD_VALID` bit of `validParams` is set.
 *
 * @param pFnPmicCommIoWrite Pointer to I2C/SPI Comm LLD Write Function. Valid
 * only when `PMIC_CFG_COMM_IO_WR_VALID` bit of `validParams` is set.
 *
 * @param pCommHandle Pointer to Handle for I2C1/SPI Main Interface. Valid only
 * when `PMIC_CFG_COMM_HANDLE_VALID` bit of `validParams` is set.
 *
 * @param pQACommHandle Pointer to Handle for I2C2-QA Interface. Valid only
 * when `PMIC_CFG_QACOMM_HANDLE_VALID` bit of `validParams` is set.
 *
 * @param pFnPmicCritSecStart Pointer to Pmic Critical-Section Start Function.
 * Valid only when `PMIC_CFG_CRITSEC_START_VALID` bit of `validParams` is set.
 *
 * @param pFnPmicCritSecStop Pointer to Pmic Critical-Section Stop Function.
 * Valid only when `PMIC_CFG_CRITSECSTOP_VALID` bit of `validParams` is set.
 */
typedef struct Pmic_CoreCfg_s {
    uint32_t validParams;
    uint32_t instType;
    uint8_t pmicDeviceType;
    uint8_t commMode;
    uint8_t slaveAddr;
    uint8_t qaSlaveAddr;
    uint8_t nvmSlaveAddr;
    uint8_t i2c1Speed;
    uint8_t i2c2Speed;
    void *pCommHandle;
    void *pQACommHandle;
    int32_t (*pFnPmicCommIoRead)(struct Pmic_CoreHandle_s *handle,
                                 uint8_t instType,
                                 uint16_t regAddr,
                                 uint8_t *pRxBuf,
                                 uint8_t bufLen);
    int32_t (*pFnPmicCommIoWrite)(struct Pmic_CoreHandle_s *handle,
                                  uint8_t instType,
                                  uint16_t regAddr,
                                  uint8_t *pTxBuf,
                                  uint8_t bufLen);
    void (*pFnPmicCritSecStart)(void);
    void (*pFnPmicCritSecStop)(void);
} Pmic_CoreCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/
/**
 * @ingroup DRV_PMIC_MODULE
 * @brief API to Initialize PMIC core handle for the PMIC LLD.
 *
 * This function gets device configuration from coreCfg and initializes
 * device specific information in handle after validation of given
 * params. Depends on validParams bit fields and does some basic validation on
 * PMIC interface I2C/SPI, confirming that PMIC is accessible for PMIC
 * configuration and monitor features.
 *
 * @param handle  [OUT] PMIC Interface Handle
 * @param coreCfg [IN]  PMIC Configuration Data
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_init(const Pmic_CoreCfg_t *coreCfg, Pmic_CoreHandle_t *handle);

/**
 * @ingroup DRV_PMIC_MODULE
 * @brief API to De-initialize an existing PMIC Instance.
 *
 * This function takes an existing Instance handle and closes the LLD
 * being used for this Instance.
 *
 * @param  handle  [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_deinit(Pmic_CoreHandle_t *handle);

/**
 * @ingroup DRV_PMIC_MODULE
 * @brief API to verify the proper construction of the PMIC handle instance.
 *
 * This API is primarily intended for use internal to the driver, as all public
 * functions must verify the integrity of the handle before performing
 * operations using it.
 *
 * @param  handle  [IN] PMIC Interface Handle
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_checkPmicCoreHandle(const Pmic_CoreHandle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_H__ */
