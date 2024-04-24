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
 *  @file   pmic_power_test.h
 *
 *  @brief  Header file for PMIC POWER Unit Tests
 */

#ifndef __PMIC_POWER_TEST_H__
#define __PMIC_POWER_TEST_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_power.h"
#include "pmic_power_priv.h"
#include "pmic_power_tps65386x_priv.h"
#include "pmic_test_common.h"

/* ========================================================================= */
/*                             Macros & Typedefs                             */
/* ========================================================================= */

#define PMIC_TPS65386X_LDO_MAX PMIC_POWER_LDO_MAX
#define PMIC_TPS65386X_BUCK1_4_CURRENT_LIMIT_MAX                               \
  PMIC_BB_REGULATOR_BUCK_CURRENT_LIMIT_5A5
#define PMIC_TPS65386X_BUCK5_CURRENT_LIMIT_MAX                                 \
  PMIC_BB_REGULATOR_BUCK_CURRENT_LIMIT_3A5
#define PMIC_TPS65386X_BUCK_SLEW_RATE_MAX PMIC_BB_REGULATOR_BUCK_SLEW_RATE_0MV31
#define PMIC_TPS65386X_REGULATOR_LDO_PLDN_VAL_MAX                              \
  PMIC_TPS65386X_REGULATOR_LDO_PLDN_VAL_500OHM
#define PMIC_TPS65386X_REGULATOR_LDO_MAX_VOLTAGE (3300U)
#define PMIC_TPS65386X_POWER_RAIL_SEL_MAX PMIC_TPS65386X_POWER_RAIL_SEL_OTHER
#define PMIC_TPS65386X_REGULATOR_LDO_RV_TIMEOUT_MAX                            \
  PMIC_TPS65386X_REGULATOR_LDO_RV_TIMEOUT_16MS

#define PMIC_POWER_NUM_OF_TESTCASES                                            \
  (sizeof(pmic_power_tests) / sizeof(pmic_power_tests[0]))

#define PMIC_UT_TPS65386X_REGULATOR_BUCK_MAX_VOLTAGE (3340U)
#define PMIC_UT_LP8764X_REGULATOR_BUCK_MAX_VOLTAGE (3340U)
#define PMIC_UT_TPS65386X_REGULATOR_BUCK_MIN_VOLTAGE (300U)
#define PMIC_UT_LP8764X_REGULATOR_BUCK_MIN_VOLTAGE (300U)
#define PMIC_UT_TPS65386X_POWER_LDO1_2_3_MIN_VOLTAGE (600U)
#define PMIC_UT_TPS65386X_POWER_LDO4_MIN_VOLTAGE (1200U)

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void test_power_setBuckBst_PGood_Cfg(void);
void test_power_setBuckBst_SSEn_Cfg(void);
void test_power_setBuckBst_StbyLvl_Cfg(void);
void test_power_setBuckBst_Level_Cfg(void);
void *test_pmic_power(void *args);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_POWER_TEST_H__ */
