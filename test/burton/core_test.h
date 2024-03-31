/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
 * \file core_test.h
 * \author John Bui (j-bui1@ti.com)
 * \brief Header file for PMIC Core Unity testing containing all test prototypes.
 * \version 1.0
 */
#ifndef CORE_TEST_H
#define CORE_TEST_H

#include "common_test.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 *  \brief  Pmic_getRecoveryCntCfg: Test whether API can set recovery count threshold value
 */
void test_core_setRecoveryCntCfg_thrVal(void);

/**
 *  \brief  Pmic_getRecoveryCntCfg: Test whether API can clear the recovery counter
 */
void test_core_setRecoveryCntCfg_clrCnt(void);

/**
 *  \brief  Pmic_setScratchPadValue: Test whether API can set the value of all scratchpad registers
 */
void test_core_setScratchPadValue_allScratchPads(void);

/**
 *  \brief  Pmic_setUserSpareValue: Test whether API can set the value of all valid user spares and
 *                                  be able to handle errors for invalid user spares
 */
void test_core_setUserSpareValue_allUserSpares(void);

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can enable/disable spread spectrum
 */
void test_core_setCommonCtrlCfg_spreadSpectrumEn(void);

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test error handling for when user wants to configure the Skip
 *                                    EEPROM Default Load Enable (TPS6522x does not support this feature)
 */
void test_core_setCommonCtrlCfg_skipEepromDefaultLoadEn(void);

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test error handling for when user wants to configure the EEPROM
 *                                    Default Load (TPS6522x does not support this feature)
 */
void test_core_setCommonCtrlCfg_eepromDefaultLoad(void);

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can configure EN_DRV
 */
void test_core_setCommonCtrlCfg_enDrv(void);

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can lock/unlock registers
 */
void test_core_setCommonCtrlCfg_regLock(void);

/**
 *  \brief  Pmic_setCommonCtrlConfig: Test whether API can configure the spread spectrum depth
 */
void test_core_setCommonCtrlCfg_spreadSpectrumDepth(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure AMUXOUT_EN
 *                                  (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_amuxOutRefOutEn(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure CLKMON_EN
 *                                  (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_clkMonEn(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure
 *                                  SYNCCLKOUT_FREQ_SEL (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_syncClkOutFreqSel(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test whether API can configure the external clock mode (SEL_EXT_CLK)
 */
void test_core_setMiscCtrlCfg_extClkSel(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test whether API can configure external clock frequency
 */
void test_core_setMiscCtrlCfg_syncClkInFreq(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test API error handling for when user wants to configure
 *                                  NRSTOUT_SOC signal (TPS6522x does not support this feature)
 */
void test_core_setMiscCtrlCfg_nRstOutSocSignal(void);

/**
 *  \brief  Pmic_setMiscCtrlConfig: Test whether API can configure NRSTOUT signal
 */
void test_core_setMiscCtrlCfg_nRstOutSignal(void);

/**
 *  \brief  Pmic_setBatteryCtrlConfig: Test API error handling for when user wants to set the
 *                                     battery configurations (TPS6522x does not support this
 *                                     feature)
 */
void test_core_setBatteryCtrlCfg_invalidDevice(void);

/**
 *  \brief  Pmic_getBatteryCtrlConfig: Test API error handling for when user wants to get the
 *                                     battery configurations (TPS6522x does not support this
 *                                     feature)
 */
void test_core_getBatteryCtrlCfg_invalidDevice(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test API error handling for when user wants to set the
 *                                  SPMI low power mode control (TPS6522x does not support this
 *                                  feature)
 */
void test_core_getCommonCtrlStat_spmiLpmStat(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the FORCE_EN_DRV_LOW status
 */
void test_core_getCommonCtrlStat_forceEnDrvLowStat(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test API error handling for when user wants to get the
 *                                  backup battery end of charge indication (TPS6522x does
 *                                  not support this feature)
 */
void test_core_getCommonCtrlStat_bbEndOfChargeIndication(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the register lock/unlock status
 */
void test_core_getCommonCtrlStat_regLockStat(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the external clock validity status
 */
void test_core_getCommonCtrlStat_extClkValidity(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test whether API can get the startup pin status
 */
void test_core_getCommonCtrlStat_startupPin(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the EN_DRV pin
 *                                  status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_enDrvPin(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the NRSTOUT_SOC
 *                                  pin status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_nRstOutSocPin(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the NRSTOUT pin
 *                                  status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_nRstOutPin(void);

/**
 *  \brief  Pmic_getCommonCtrlStat: Test error handling for when user wants to get the nINT pin
 *                                  status (TPS6522x does not support this feature)
 */
void test_core_getCommonCtrlStat_nIntPin(void);

/**
 *  \brief  Pmic_getDeviceInfo: Test whether API can get the PMIC device information
 */
void test_core_getDeviceInfo(void);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* CORE_TEST_H */
