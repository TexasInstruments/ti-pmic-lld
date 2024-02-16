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
 * \file   pmic_esm.h
 *
 * \brief  PMIC Low Level Driver API/interface file for ESM API
 */

#ifndef PMIC_ESM_H_
#define PMIC_ESM_H_
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic.h"
#include "pmic_core.h"
#include "pmic_core_priv.h"
#include "pmic_io_priv.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 *  \anchor Pmic_EsmTypes
 *  \name PMIC ESM types
 *
 *  @{
 */
#define PMIC_ESM_MODE_MCU (bool)false
#define PMIC_ESM_MODE_SOC (bool)true

/**
 *  \anchor Pmic_EsmStates
 *  \name PMIC ESM Start/Stop state
 *
 *  @{
 */
#define PMIC_ESM_STOP (bool)false
#define PMIC_ESM_START (bool)true
/*  @} */

/**
 *  \anchor Pmic_EsmToggle
 *  \name PMIC ESM Enable/Disable
 *
 *  @{
 */
#define PMIC_ESM_DISABLE (bool)false
#define PMIC_ESM_ENABLE (bool)true
/*  @} */

/**
 *  \anchor Pmic_EsmMode
 *  \name PMIC ESM Modes
 *
 *  @{
 */
#define PMIC_ESM_LEVEL_MODE (bool)false
#define PMIC_ESM_PWM_MODE (bool)true
/*  @} */
/**
 *  \anchor Pmic_EsmEnDrvSel
 *  \name PMIC ESM EN DRV Clear CFG
 *
 *  @{
 */
#define PMIC_ESM_ERR_EN_DRV_CLEAR_DISABLE (bool)false
#define PMIC_ESM_ERR_EN_DRV_CLEAR_ENABLE (bool)true
/*  @} */

/**
 *  \anchor Pmic_EsmIntr
 *  \name PMIC ESM Interrupt enable/disable
 *
 *  @{
 */
#define PMIC_ESM_INTERRUPT_DISABLE (bool)false
#define PMIC_ESM_INTERRUPT_ENABLE (bool)true
/*  @} */

#define PMIC_ESM_VAL_1 (1U)

/** \brief validParams value used to set/get ESM delay-1 time interval */
#define PMIC_ESM_CFG_DELAY1_VALID (0x00U)
/** \brief validParams value used to set/get ESM delay-2 time interval  */
#define PMIC_ESM_CFG_DELAY2_VALID (0x01U)
/** \brief validParams value used to set/get ESM Error count Threshold value */
#define PMIC_ESM_CFG_ERR_CNT_THR_VALID (0x02U)
/** \brief validParams value used to set/get ESM Maximum high-pulse
 *         time-threshold value  */
#define PMIC_ESM_CFG_HMAX_VALID (0x03U)
/** \brief validParams value used to set/get ESM Minimum high-pulse
 *         time-threshold value  */
#define PMIC_ESM_CFG_HMIN_VALID (0x04U)
/** \brief validParams value used to set/get ESM Maximum low-pulse
 *         time-threshold value */
#define PMIC_ESM_CFG_LMAX_VALID (0x05U)
/** \brief validParams value used to set/get  ESM Minimum low-pulse
 *         time-threshold value */
#define PMIC_ESM_CFG_LMIN_VALID (0x06U)
/** \brief validParams value used to set/get ESM ENABLE_DRV clear configuration
 */
#define PMIC_ESM_CFG_EN_DRV_VALID (0x07U)
/** \brief validParams value used to set/get ESM mode */
#define PMIC_ESM_CFG_MODE_VALID (0x08U)

#define PMIC_ESM_CFG_DELAY1_VALID_SHIFT (0x01U << PMIC_ESM_CFG_DELAY1_VALID)
#define PMIC_ESM_CFG_DELAY2_VALID_SHIFT (0x01U << PMIC_ESM_CFG_DELAY2_VALID)
#define PMIC_ESM_CFG_ERR_CNT_THR_VALID_SHIFT                                   \
  (0x01U << PMIC_ESM_CFG_ERR_CNT_THR_VALID)
#define PMIC_ESM_CFG_HMAX_VALID_SHIFT (0x01U << PMIC_ESM_CFG_HMAX_VALID)
#define PMIC_ESM_CFG_HMIN_VALID_SHIFT (0x01U << PMIC_ESM_CFG_HMIN_VALID)
#define PMIC_ESM_CFG_LMAX_VALID_SHIFT (0x01U << PMIC_ESM_CFG_LMAX_VALID)
#define PMIC_ESM_CFG_LMIN_VALID_SHIFT (0x01U << PMIC_ESM_CFG_LMIN_VALID)
#define PMIC_ESM_CFG_EN_DRV_VALID_SHIFT (0x01U << PMIC_ESM_CFG_EN_DRV_VALID)
#define PMIC_ESM_CFG_MODE_VALID_SHIFT (0x01U << PMIC_ESM_CFG_MODE_VALID)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/**
 * \brief  PMIC ESM Configuration structure
 *         Note: validParams is input param for all Set and Get APIs. other
 *         params except validParams is input param for Set APIs and output
 *         param for Get APIs
 *
 * \param   validParams          Selection of structure parameters to be set,
 *                               from the combination of \ref Pmic_EsmCflag
 *                               and the corresponding member value must be
 *                               updated.
 * \param   esmDelay1_us         ESM delay-1 time interval in micro seconds.
 *                               To get more effective results, user has to
 *                               program esmDelay1 with multiples of 2048.
 *                               The valid range is (0, 2048, 4096, 6144,
 *                               8192, ......., 522240).
 *                               Valid only when PMIC_ESM_CFG_DELAY1_VALID
 *                               bit is set
 * \param   esmDelay2_us         ESM delay-2 time interval in micro seconds.
 *                               To get more effective results, user has to
 *                               program esmDelay2 with multiples of 2048.
 *                               The valid range is (0, 2048, 4096, 6144,
 *                               8192, ......., 522240).
 *                               Valid only when PMIC_ESM_CFG_DELAY2_VALID
 *                               bit is set
 * \param   esmHmax_us           ESM Maximum high-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmHmax with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_HMAX_VALID
 *                               bit is set
 * \param   esmHmin_us           ESM Minimum high-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmHmin with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_HMIN_VALID
 *                               bit is set
 * \param   esmLmax_us           ESM Maximum low-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmLmax with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_LMAX_VALID
 *                               bit is set
 * \param   esmLmin_us           ESM Minimum low-pulse time-threshold value in
 *                               micro seconds.
 *                               To get more effective results, user has to
 *                               program esmLmin with multiples of 15.
 *                               The valid range is (15, 30, 45, 60, 75
 *                              ....., 3840).
 *                               Valid only when PMIC_ESM_CFG_LMIN_VALID
 *                               bit is set
 * \param   esmErrCntThr         ESM Error count Threshold value.
 *                               Valid only when PMIC_ESM_CFG_ERR_CNT_THR_VALID
 *                               bit is set
 * \param   esmEnDrv             ESM ENABLE_DRV clear configuration.
 *                               Valid values: \ref Pmic_EsmEnDrvSel.
 *                              #endif Valid only when PMIC_ESM_CFG_EN_DRV_VALID
 *                               bit is set
 * \param   esmMode              ESM mode select.
 *                               Valid values: \ref Pmic_EsmMode.
 *                               Valid only when PMIC_ESM_CFG_MODE_VALID
 *                               bit is set
 */
typedef struct Pmic_EsmCfg_s {
  uint32_t validParams;
  uint32_t esmDelay1_us;
  uint32_t esmDelay2_us;
  uint16_t esmHmax_us;
  uint16_t esmHmin_us;
  uint16_t esmLmax_us;
  uint16_t esmLmin_us;
  uint8_t esmErrCntThr;
  bool esmEnDrv;
  bool esmMode;
} Pmic_EsmCfg_t;

/**
 * \brief   PMIC ESM Interrupt Configuration Structure.
 *
 * \param   esmPinIntr             ESM Pin Interrupt configuration.
 *                                 Valid values: \ref Pmic_EsmIntr.
 * \param   esmFailIntr            ESM Fail Interrupt configuration.
 *                                 Valid values: \ref Pmic_EsmIntr.
 * \param   esmRstIntr             ESM Reset Interrupt configuration.
 *                                 Valid values: \ref Pmic_EsmIntr.
 */
typedef struct Pmic_EsmIntrCfg_s {
  bool esmPinIntr;
  bool esmFailIntr;
  bool esmRstIntr;
} Pmic_EsmIntrCfg_t;

/*==========================================================================*/
/*                         Function Declarations                            */
/*==========================================================================*/

int32_t Pmic_esmStart(Pmic_CoreHandle_t *pPmicCoreHandle, const bool esmType,
                      const bool esmState);

int32_t Pmic_esmEnable(Pmic_CoreHandle_t *pPmicCoreHandle, const bool esmType,
                       const bool esmToggle);

int32_t Pmic_esmSetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const bool esmType,
                                 const Pmic_EsmCfg_t esmCfg);

int32_t Pmic_esmGetConfiguration(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 const bool esmType, Pmic_EsmCfg_t *pEsmCfg);

int32_t Pmic_esmGetErrCnt(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const bool esmType, uint8_t *pEsmErrCnt);

int32_t Pmic_esmGetStatus(Pmic_CoreHandle_t *pPmicCoreHandle,
                          const bool esmType, bool *pEsmState);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_ESM_H_ */
