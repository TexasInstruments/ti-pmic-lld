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
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"
#include "pmic_io.h"

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
 * @details Application code should check all `Pmic_*` functions that return a
 * status code to verify that `PMIC_ST_SUCCESS` was returned, all other status
 * codes indicate that the requested operation did not succeed.
 *
 * @{
 */
#define PMIC_ST_SUCCESS                 (-((int32_t)0))
#define PMIC_ST_ERR_INV_HANDLE          (-((int32_t)1))
#define PMIC_ST_ERR_NULL_PARAM          (-((int32_t)2))
#define PMIC_ST_ERR_INV_PARAM           (-((int32_t)3))
#define PMIC_ST_ERR_INV_DEVICE          (-((int32_t)4))
#define PMIC_ST_ERR_INSUFFICIENT_CFG    (-((int32_t)7))
#define PMIC_ST_ERR_I2C_COMM_FAIL       (-((int32_t)8))
#define PMIC_ST_ERR_SPI_COMM_FAIL       (-((int32_t)9))
#define PMIC_ST_ERR_INV_COMM_CRC        (-((int32_t)10))
#define PMIC_ST_WARN_NO_IRQ_REMAINING   (-((int32_t)11))
#define PMIC_ST_ERR_NOT_SUPPORTED       (-((int32_t)12))
/** @} */

/**
 * @anchor Pmic_EnableDisable
 * @name PMIC Enable/Disable features control
 *
 * @{
 */
#define PMIC_ENABLE  ((bool)true)
#define PMIC_DISABLE ((bool)false)
/** @} */

/**
 * @anchor Pmic_DeviceType
 * @name PMIC Device type
 *
 * @{
 */
#define PMIC_DEV_CHARIOT_LP8774X (0U)
/** @} */

/**
 * @anchor Pmic_CommMode
 * @name PMIC Communication Mode
 *
 * @note For LP8774x PMIC, PMIC_INTF_SPI is the only valid instance type.
 *
 * @{
 */
#define PMIC_INTF_I2C_SINGLE (0U)
#define PMIC_INTF_I2C_DUAL   (1U)
#define PMIC_INTF_SPI        (2U)
#define PMIC_INTF_MAX        (PMIC_INTF_SPI)
/** @} */

/**
 * @anchor Pmic_I2CSpeedSel
 * @name PMIC Select I2C Speed
 *
 * @note Set I2C Master before switching the I2C speed to HS/Standard Mode, I2C
 * Master has to set/reset I2C1_HS/I2C2_HS bit field accordingly then only I2C
 * Master can communicate with PMIC in HS/Standard Mode.
 *
 * @{
 */
#define PMIC_I2C_STANDARD_MODE  (0U)
#define PMIC_I2C_FORCED_HS_MODE (1U)
#define PMIC_I2C_SPEED_SEL_MAX  (PMIC_I2C_FORCED_HS_MODE)
/** @} */

/**
 * @anchor Pmic_HandleCfgValidParams
 * @name PMIC Handle Configuration Structure Valid Parameters
 *
 * Application can use the values below to set the validParam struct member
 * defined in Pmic_HandleCfg_t structure
 *
 * @{
 */
#define PMIC_DEVICE_TYPE_VALID        (1U << 0U)
#define PMIC_COMM_MODE_VALID          (1U << 1U)
#define PMIC_SLAVE_ADDR_VALID         (1U << 2U)
#define PMIC_QA_SLAVE_ADDR_VALID      (1U << 3U)
#define PMIC_NVM_SLAVE_ADDR_VALID     (1U << 4U)
#define PMIC_I2C1_SPEED_VALID         (1U << 5U)
#define PMIC_I2C2_SPEED_VALID         (1U << 6U)
#define PMIC_CRC_ENABLE_VALID         (1U << 7U)
#define PMIC_COMM_HANDLE_VALID        (1U << 8U)
#define PMIC_QA_COMM_HANDLE_VALID     (1U << 9U)
#define PMIC_IO_READ_VALID            (1U << 10U)
#define PMIC_IO_WRITE_VALID           (1U << 11U)
#define PMIC_CRIT_SEC_START_VALID     (1U << 12U)
#define PMIC_CRIT_SEC_STOP_VALID      (1U << 13U)
#define PMIC_IRQ_RESPONSE_VALID       (1U << 14U)
#define PMIC_HANDLE_CFG_LP8774X_VALID \
    (PMIC_DEVICE_TYPE_VALID | \
     PMIC_COMM_MODE_VALID | \
     PMIC_CRC_ENABLE_VALID | \
     PMIC_COMM_HANDLE_VALID | \
     PMIC_IO_READ_VALID | \
     PMIC_IO_WRITE_VALID | \
     PMIC_CRIT_SEC_START_VALID | \
     PMIC_CRIT_SEC_STOP_VALID | \
     PMIC_IRQ_RESPONSE_VALID)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */
/**
 * @anchor Pmic_HandleCfg
 * @brief PMIC handle configuration structure.
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
 * `PMIC_CFG_DEVICE_TYPE_VALID` bit of `validParams` struct and then call
 * `Pmic_init()`.
 *
 * @note The below parameters are not relevant for LP8774x PMIC.
 * 1. qaSlaveAddr
 * 2. nvmSlaveAddr
 * 3. i2c1Speed
 * 4. i2c2Speed
 * 5. qaCommHandle
 *
 * @param validParams Controls which parameters below shall be considered by
 * `Pmic_init()`, decided by the combination of @ref Pmic_HandleCfgValidParams.
 *
 * @param deviceType PMIC device type. For valid values, see @ref Pmic_DeviceType.
 * Valid only when `PMIC_DEVICE_TYPE_VALID` bit of
 * `validParams` is set.
 *
 * @param commMode Communications interface mode: Single I2C, Dual I2C or SPI.
 * For valid values, see @ref Pmic_CommMode. Valid only when
 * `PMIC_COMM_MODE_VALID` bit of `validParams` is set. For LP8774x PMIC,
 * 'PMIC_INTF_SPI_SINGLE' is the common communication mode.
 *
 * @param slaveAddr Main Interface Slave Address for I2C. Valid only when
 * `PMIC_SLAVE_ADDR_VALID` bit of `validParams` is set. Only necessary for
 * I2C interfaces.
 *
 * @param qaSlaveAddr WDG QA Interface Slave Address for I2C. Valid only when
 * `PMIC_QA_SLAVE_ADDR_VALID` bit of `validParams` is set. Only necessary for
 * I2C interfaces.
 *
 * @param nvmSlaveAddr NVM Slave Address for I2C. This provides only read
 * access to CRC status of Page-1 Application shall use this slave address to
 * read only CRC status. Application shall not do any write operations using
 * this slave address. Valid only when `PMIC_NVM_SLAVE_ADDR_VALID` bit of
 * `validParams` is set. Only necessary for I2C interfaces.
 *
 * @param i2c1Speed Configures I2C1 Speed when commMode is Single or Dual I2C.
 * For valid values see, @ref Pmic_I2CSpeedSel. Valid only when
 * `PMIC_I2C1_SPEED_VALID` bit is set. Only necessary for I2C interfaces.
 *
 * @param i2c2Speed Configures I2C2 Speed when commMode is Dual I2C For valid
 * values, see @ref Pmic_I2CSpeedSel. Valid only when
 * `PMIC_I2C2_SPEED_VALID` bit is set. Only necessary for I2C interfaces.
 *
 * @param crcEnable Controls whether communications layer CRC is enabled or
 * disabled. If enabled the driver will enable the CRC feature in HW and perform
 * the necessary CRC calculations when communication with the PMIC. If disabled,
 * the CRC feature will be disabled in HW and no calculations will be performed
 * by the driver.
 *
 * @param commHandle Pointer to Handle for I2C1/SPI Main Interface. Valid only
 * when `PMIC_COMM_HANDLE_VALID` bit of `validParams` is set.
 *
 * @param qaCommHandle Pointer to Handle for I2C2-QA Interface. Valid only
 * when `PMIC_QA_COMM_HANDLE_VALID` bit of `validParams` is set.
 *
 * @param ioRead Pointer to I2C/SPI Comm LLD Read Function. Valid only when
 * `PMIC_IO_READ_VALID` bit of `validParams` is set.
 *
 * @param ioWrite Pointer to I2C/SPI Comm LLD Write Function. Valid only when
 * `PMIC_IO_WRITE_VALID` bit of `validParams` is set.
 *
 * @param critSecStart Pointer to PMIC Critical-Section Start Function. Valid
 * only when `PMIC_CRIT_SEC_START_VALID` bit of `validParams` is set.
 *
 * @param critSecStop Pointer to PMIC Critical-Section Stop Function. Valid only
 * when `PMIC_CRIT_SEC_STOP_VALID` bit of `validParams` is set.
 *
 * @param irqResponse Pointer to a user provided callback function that can
 * be used to support Pseudo-nINT functionality when servicing WD QA sequences.
 * While performing a WD QA sequence, the PMIC LLD will check the INT_TOP_STATUS
 * field, and if set will call this function to notify the user that an
 * interrupt is pending. Valid only when `PMIC_IRQ_RESPONSE_VALID` bit of
 * `validParams` is set.
 */
typedef struct Pmic_HandleCfg_s {
    uint32_t validParams;
    uint8_t deviceType;
    uint8_t commMode;
    uint8_t slaveAddr;
    uint8_t qaSlaveAddr;
    uint8_t nvmSlaveAddr;
    uint8_t i2c1Speed;
    uint8_t i2c2Speed;
    bool crcEnable;
    void *commHandle;
    void *qaCommHandle;
    int32_t (*ioRead)(
        const struct Pmic_Handle_s *pmicHandle, uint16_t regAddr, uint8_t *rxBuf, uint8_t bufLen);
    int32_t (*ioWrite)(
        const struct Pmic_Handle_s *pmicHandle, uint16_t regAddr, const uint8_t *txBuf, uint8_t bufLen);
    void (*critSecStart)(void);
    void (*critSecStop)(void);
    void (*irqResponse)(void);
} Pmic_HandleCfg_t;

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Initialize PMIC handle for PMIC LLD. This API also validates the
 * handle, returning an error if the handle is invalid at end of invocation.
 *
 * @details This API can set the following configurations for the PMIC handle
 * 1. PMIC device type (validParams: PMIC_DEVICE_TYPE_VALID)
 * 2. Communication mode (validParams: PMIC_COMM_MODE_VALID)
 * 3. I2C slave address (validParams: PMIC_SLAVE_ADDR_VALID)
 * 4. I2C Q&A slave address (validParams: PMIC_QA_SLAVE_ADDR_VALID)
 * 5. I2C NVM slave address (validParams: PMIC_NVM_SLAVE_ADDR_VALID)
 * 6. I2C1 speed (validParams: PMIC_I2C1_SPEED_VALID)
 * 7. I2C2 speed (validParams: PMIC_I2C2_SPEED_VALID)
 * 8. CRC enable (validParams: PMIC_CRC_ENABLE_VALID)
 * 9. Communication handle (validParams: PMIC_COMM_HANDLE_VALID)
 * 10. Q&A communication handle (validParams: PMIC_QA_COMM_HANDLE_VALID)
 * 11. Serial communication write hook (validParams: PMIC_IO_READ_VALID)
 * 12. Serial communication read hook (validParams: PMIC_IO_WRITE_VALID)
 * 13. Critical section start hook (validParams: PMIC_CRIT_SEC_START_VALID)
 * 14. Critical section stop hook (validParams: PMIC_CRIT_SEC_STOP_VALID)
 * 15. IRQ response hook (validParams: PMIC_IRQ_RESPONSE_VALID)
 *
 * @param handle [OUT] PMIC interface handle.
 *
 * @param handleCfg [IN] Desired PMIC handle configurations.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * valid values @ref Pmic_ErrorCodes.
 */
int32_t Pmic_init(Pmic_Handle_t *handle, const Pmic_HandleCfg_t *handleCfg);

/**
 * @brief De-initialize an existing PMIC handle instance.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the PMIC handle has been de-initialized
 * successfully, error code otherwise. For valid success/error codes,
 * refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_deinit(Pmic_Handle_t *handle);

/**
 * @brief Check whether the PMIC handle is valid. Meant to be used internally
 * by PMIC LLD to validate that the PMIC handle is ready for correct usage.
 * However, end-user could independently use this API to validate the PMIC
 * handle in their application.
 *
 * @details The following members must be non-NULL or a specific value for the
 * handle to to be valid
 * 1. drvInitStat (specific value)
 * 2. deviceType (specific value)
 * 3. commMode (specific value)
 * 4. commHandle (non-NULL)
 * 5. ioRead (non-NULL)
 * 6. ioWrite (non-NULL)
 * 7. critSecStart (non-NULL)
 * 8. critSecStop (non-NULL)
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the PMIC handle is valid, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_checkPmicHandle(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_H */
