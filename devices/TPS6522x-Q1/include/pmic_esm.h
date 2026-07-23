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
#ifndef PMIC_ESM_H
#define PMIC_ESM_H

/**
 * @file pmic_esm.h
 *
 * @brief PMIC error signal monitor (ESM) interface. Contains APIs, macros/defines,
 * and data structures used to configure, control, and interact with the PMIC ESM.
 */

/* ========================================================================= */
/*                              Include Files                                */
/* ========================================================================= */

#include "pmic_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmMode
 * @name PMIC ESM Mode
 *
 * @brief Range of values for ESM mode.
 *
 * @{
 */
#define PMIC_ESM_MODE_LEVEL (0U)
#define PMIC_ESM_MODE_PWM   (1U)
#define PMIC_ESM_MODE_MIN   ((uint8_t)PMIC_ESM_MODE_LEVEL)
#define PMIC_ESM_MODE_MAX   ((uint8_t)PMIC_ESM_MODE_PWM)
/** @} */

/**
 * @anchor Pmic_EsmErrCntThr
 * @name PMIC ESM Error Counter Threshold
 *
 * @brief Range of values for ESM error counter threshold.
 *
 * @{
 */
#define PMIC_ESM_ERR_CNT_THR_MIN ((uint8_t)0x0U)
#define PMIC_ESM_ERR_CNT_THR_MAX ((uint8_t)0xFU)
/** @} */

/**
 * @anchor Pmic_EsmCfgValidParams
 * @name PMIC ESM Configuration Structure Valid Parameters
 *
 * @brief Range of values for ESM error counter threshold.
 *
 * @{
 */
#define PMIC_CFG_ESM_MODE_VALID                   (1UL << 0U)
#define PMIC_CFG_ESM_ERR_CNT_THR_VALID            (1UL << 1U)
#define PMIC_CFG_ESM_DELAY1_VALID                 (1UL << 2U)
#define PMIC_CFG_ESM_DELAY2_VALID                 (1UL << 3U)
#define PMIC_CFG_ESM_LMIN_VALID                   (1UL << 4U)
#define PMIC_CFG_ESM_LMAX_VALID                   (1UL << 5U)
#define PMIC_CFG_ESM_HMIN_VALID                   (1UL << 6U)
#define PMIC_CFG_ESM_HMAX_VALID                   (1UL << 7U)
#define PMIC_CFG_ESM_CLR_EN_DRV_ON_FAIL_INT_VALID (1UL << 8U)
/** @} */

/**
 * @anchor Pmic_EsmStatusValidParams
 * @name PMIC ESM Status Structure Valid Parameters
 *
 * @brief Valid parameter bits for ESM status structure.
 *
 * @{
 */
#define PMIC_ESM_RST_INT_VALID  (1UL << 0U)
#define PMIC_ESM_FAIL_INT_VALID (1UL << 1U)
#define PMIC_ESM_PIN_INT_VALID  (1UL << 2U)
#define PMIC_ESM_STATUS_ALL_VALID (PMIC_ESM_RST_INT_VALID | PMIC_ESM_FAIL_INT_VALID | PMIC_ESM_PIN_INT_VALID)
/** @} */

/* ========================================================================== */
/*                           Structures and Enums                             */
/* ========================================================================== */

/**
 * @anchor Pmic_EsmCfg
 * @name PMIC ESM Configuration Structure
 *
 * @brief Structure used to set and get ESM configurations.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. Specifically, if a bit is set to 1 in this variable, the corresponding
 * structure member is valid and will be considered by the driver API that is using
 * this data structure. Otherwise, if a bit is set to 0, the corresponding structure
 * member is invalid and will not be considered by the driver API that is using
 * this data structure. For possible valid parameter values, refer to
 * @ref Pmic_EsmCfgValidParams.
 *
 * @param mode ESM mode of operation. For valid values, refer to @ref Pmic_EsmMode.
 *
 * @param errCntThr ESM error counter threshold. For valid values, refer to
 * @ref Pmic_EsmErrCntThr.
 *
 * @param delay1 ESM delay 1. All values within the range [0x0, 0xFF] are valid.
 * To convert to a human-readable time duration, refer to the device data sheet.
 *
 * @param delay2 ESM delay 2. All values within the range [0x0, 0xFF] are valid.
 * To convert to a human-readable time duration, refer to the device data sheet.
 *
 * @param lmin Valid only in PWM mode. Minimum time in which the PWM signal can
 * be low. All values within the range [0x0, 0xFF] are valid.
 *
 * @param lmax Valid only in PWM mode. Maximum time in which the PWM signal can
 * be low. All values within the range [0x0, 0xFF] are valid.
 *
 * @param hmin Valid only in PWM mode. Minimum time in which the PWM signal can
 * be high. All values within the range [0x0, 0xFF] are valid.
 *
 * @param hmax Valid only in PWM mode. Maximum time in which the PWM signal can
 * be high. All values within the range [0x0, 0xFF] are valid.
 *
 * @param clrEnDrvOnFailInt Control whether EN_DRV is cleared when ESM_MCU_FAIL_INT
 * occurs.
 *
 * @{
 */
typedef struct Pmic_EsmCfg_s {
    uint32_t validParams;

    uint8_t mode;
    uint8_t errCntThr;

    uint8_t delay1;
    uint8_t delay2;

    uint8_t lmin;
    uint8_t lmax;
    uint8_t hmin;
    uint8_t hmax;

    bool clrEnDrvOnFailInt;
} Pmic_EsmCfg_t;
/** @} */

/**
 * @anchor Pmic_EsmStatus
 * @name PMIC ESM Status Structure
 *
 * @brief Structure used to get and clear ESM status/interrupt flags.
 *
 * @param validParams Each bit in this variable corresponds to a member in this
 * structure. For possible valid parameter values, refer to
 * @ref Pmic_EsmStatusValidParams.
 *
 * @param rstInt ESM MCU reset interrupt flag.
 *
 * @param failInt ESM MCU fail interrupt flag.
 *
 * @param pinInt ESM MCU pin interrupt flag.
 *
 * @{
 */
typedef struct Pmic_EsmStatus_s {
    uint32_t validParams;

    bool rstInt;
    bool failInt;
    bool pinInt;
} Pmic_EsmStatus_t;
/** @} */

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

/**
 * @brief Enable or disable the PMIC ESM.
 *
 * Design: PMICDRV-686
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param enable [IN] `PMIC_ENABLE` - enable the ESM; `PMIC_DISABLE` - disable
 * the ESM.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM has been enabled/disabled, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmSetEnableState(const Pmic_Handle_t *handle, bool enable);

/**
 * @brief Get the enable state of the PMIC ESM.
 *
 * Design: PMICDRV-687
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isEnabled [OUT] `PMIC_ENABLE` - ESM is enabled; `PMIC_DISABLE` - ESM is
 * disabled.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM enable state has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled);

/**
 * @brief Start or stop the PMIC ESM.
 *
 * Design: PMICDRV-593
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param start [IN] This parameter set to true will start the ESM. Otherwise,
 * if the value is set false, the ESM will stop.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM has started or stopped, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmSetStartState(const Pmic_Handle_t *handle, bool start);

/**
 * @brief Get start state of the PMIC ESM.
 *
 * Design: PMICDRV-596
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param started [OUT] This parameter returned as true means the ESM is started.
 * Otherwise, if the value returned is false, the ESM is stopped.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM start state has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetStartState(const Pmic_Handle_t *handle, bool *started);

/**
 * @brief Set PMIC ESM configurations.
 *
 * Design: PMICDRV-597
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmCfg [IN] Desired PMIC ESM configurations to set. For more information
 * on ESM configurations, refer to @ref Pmic_EsmCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM configurations have been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmSetCfg(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM configurations.
 *
 * Design: PMICDRV-598
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmCfg [OUT] PMIC ESM configurations obtained from the PMIC. For more
 * information on ESM configurations, refer to @ref Pmic_EsmCfg.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM configurations have been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetCfg(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg);

/**
 * @brief Get PMIC ESM error counter.
 *
 * Design: PMICDRV-688
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-528, PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param esmErrCnt [OUT] PMIC ESM error counter value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM error counter value has been obtained
 * from the PMIC, error code otherwise. For valid success/error codes, refer
 * to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmGetErrCnt(const Pmic_Handle_t *handle, uint8_t *esmErrCnt);

/**
 * @brief Start PMIC ESM monitoring.
 *
 * Design: PMICDRV-594
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523,
 *               PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM has been started, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmStart(const Pmic_Handle_t *handle);

/**
 * @brief Stop PMIC ESM monitoring.
 *
 * Design: PMICDRV-595
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-523,
 *               PMICDRV-539
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if PMIC ESM has been stopped, error code otherwise.
 * For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_esmStop(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_ESM_H */
