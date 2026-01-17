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
#ifndef ESM_TEST_H
#define ESM_TEST_H



/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include "../platform.h"
#include "pmic_esm.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/**
 * @brief Convenience macro for common ESM configuration parameters
 */
#define PMIC_ESM_CFG_VALID \
    (PMIC_CFG_ESM_ENABLE_VALID | \
     PMIC_CFG_ESM_MODE_VALID | \
     PMIC_CFG_ESM_ERR_THR_VALID | \
     PMIC_CFG_ESM_POLARITY_VALID | \
     PMIC_CFG_ESM_DEGLITCH_VALID | \
     PMIC_CFG_ESM_TIME_BASE_VALID | \
     PMIC_CFG_ESM_HMAX_VALID | \
     PMIC_CFG_ESM_HMIN_VALID | \
     PMIC_CFG_ESM_LMAX_VALID | \
     PMIC_CFG_ESM_LMIN_VALID)

/**
 * @brief Convenience macro for ESM status valid params - use actual status names
 */
#define PMIC_ESM_ERR_VALID_SHIFT PMIC_ESM_ERR_VALID
#define PMIC_ESM_DELAY1_ERR_VALID_SHIFT PMIC_ESM_DELAY1_ERR_VALID
#define PMIC_ESM_DELAY2_ERR_VALID_SHIFT PMIC_ESM_DELAY2_ERR_VALID
#define PMIC_ESM_ERR_CNT_VALID_SHIFT PMIC_ESM_ERR_CNT_VALID
#define PMIC_CFG_ESM_DELAY1_VALID_SHIFT PMIC_CFG_ESM_DELAY1_VALID
#define PMIC_CFG_ESM_DELAY2_VALID_SHIFT PMIC_CFG_ESM_DELAY2_VALID

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void esm_test(void *args);

/* ========================================================================== */
/*                       Positive Test Declarations                           */
/* ========================================================================== */

/* esmSetStartState and esmGetStartState Tests */
/**
 * @brief Test: Set and get ESM start state.
 */
void test_pos_esm_esmSetGetStartState(void);

/* esmStart Tests */
/**
 * @brief Test: Start ESM using Pmic_esmStart().
 */
void test_pos_esm_esmStart(void);

/* esmStop Tests */
/**
 * @brief Test: Stop ESM using Pmic_esmStop().
 */
void test_pos_esm_esmStop(void);

/* esmSetCfg Tests */
/**
 * @brief Test: Set and get ESM enable configuration.
 */
void test_pos_esm_esmSetCfg_enable(void);

/**
 * @brief Test: Set and get ESM mode configuration.
 */
void test_pos_esm_esmSetCfg_mode(void);

/**
 * @brief Test: Set and get ESM error threshold configuration.
 */
void test_pos_esm_esmSetCfg_errThr(void);

/**
 * @brief Test: Set and get ESM polarity configuration.
 */
void test_pos_esm_esmSetCfg_polarity(void);

/**
 * @brief Test: Set and get ESM deglitch configuration.
 */
void test_pos_esm_esmSetCfg_deglitch(void);

/**
 * @brief Test: Set and get ESM time base configuration.
 */
void test_pos_esm_esmSetCfg_timeBase(void);

/**
 * @brief Test: Set and get ESM delay1 configuration.
 */
void test_pos_esm_esmSetCfg_delay1(void);

/**
 * @brief Test: Set and get ESM delay2 configuration.
 */
void test_pos_esm_esmSetCfg_delay2(void);

/**
 * @brief Test: Set and get ESM hmax configuration.
 */
void test_pos_esm_esmSetCfg_hmax(void);

/**
 * @brief Test: Set and get ESM hmin configuration.
 */
void test_pos_esm_esmSetCfg_hmin(void);

/**
 * @brief Test: Set and get ESM lmax configuration.
 */
void test_pos_esm_esmSetCfg_lmax(void);

/**
 * @brief Test: Set and get ESM lmin configuration.
 */
void test_pos_esm_esmSetCfg_lmin(void);

/**
 * @brief Test: Set and get multiple ESM configuration parameters simultaneously.
 */
void test_pos_esm_esmSetCfg_multiple(void);

/* esmGetStatus Tests */
/**
 * @brief Test: Get ESM error status.
 */
void test_pos_esm_esmGetStatus_esmErr(void);

/**
 * @brief Test: Get ESM delay1 error status.
 */
void test_pos_esm_esmGetStatus_delay1Err(void);

/**
 * @brief Test: Get ESM delay2 error status.
 */
void test_pos_esm_esmGetStatus_delay2Err(void);

/**
 * @brief Test: Get ESM error count status.
 */
void test_pos_esm_esmGetStatus_errCnt(void);

/* esmClrStatus Tests */
/**
 * @brief Test: Clear ESM status flags.
 */
void test_pos_esm_esmClrStatus(void);

/* ========================================================================== */
/*                       Negative Test Declarations                           */
/* ========================================================================== */

/* esmSetStartState Tests */
/**
 * @brief Test: Pmic_esmSetStartState() with NULL handle.
 */
void test_neg_esm_esmSetStartState_nullHandle(void);

/* esmGetStartState Tests */
/**
 * @brief Test: Pmic_esmGetStartState() with NULL handle.
 */
void test_neg_esm_esmGetStartState_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetStartState() with NULL start pointer.
 */
void test_neg_esm_esmGetStartState_nullPointer(void);

/* esmStart Tests */
/**
 * @brief Test: Pmic_esmStart() with NULL handle.
 */
void test_neg_esm_esmStart_nullHandle(void);

/* esmStop Tests */
/**
 * @brief Test: Pmic_esmStop() with NULL handle.
 */
void test_neg_esm_esmStop_nullHandle(void);

/* esmSetCfg Tests */
/**
 * @brief Test: Pmic_esmSetCfg() with NULL handle.
 */
void test_neg_esm_esmSetCfg_nullHandle(void);

/**
 * @brief Test: Pmic_esmSetCfg() with NULL configuration pointer.
 */
void test_neg_esm_esmSetCfg_nullPointer(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid validParams (0).
 */
void test_neg_esm_esmSetCfg_invalidParams(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid mode value.
 */
void test_neg_esm_esmSetCfg_invalidMode(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid error threshold value.
 */
void test_neg_esm_esmSetCfg_invalidErrThr(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid polarity value.
 */
void test_neg_esm_esmSetCfg_invalidPolarity(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid deglitch value.
 */
void test_neg_esm_esmSetCfg_invalidDeglitch(void);

/**
 * @brief Test: Pmic_esmSetCfg() with invalid time base value.
 */
void test_neg_esm_esmSetCfg_invalidTimeBase(void);

/* esmGetCfg Tests */
/**
 * @brief Test: Pmic_esmGetCfg() with NULL handle.
 */
void test_neg_esm_esmGetCfg_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetCfg() with NULL configuration pointer.
 */
void test_neg_esm_esmGetCfg_nullPointer(void);

/**
 * @brief Test: Pmic_esmGetCfg() with invalid validParams (0).
 */
void test_neg_esm_esmGetCfg_invalidParams(void);

/* esmGetStatus Tests */
/**
 * @brief Test: Pmic_esmGetStatus() with NULL handle.
 */
void test_neg_esm_esmGetStatus_nullHandle(void);

/**
 * @brief Test: Pmic_esmGetStatus() with NULL status pointer.
 */
void test_neg_esm_esmGetStatus_nullPointer(void);

/**
 * @brief Test: Pmic_esmGetStatus() with invalid validParams (0).
 */
void test_neg_esm_esmGetStatus_invalidParams(void);

/* esmClrStatus Tests */
/**
 * @brief Test: Pmic_esmClrStatus() with NULL handle.
 */
void test_neg_esm_esmClrStatus_nullHandle(void);

/**
 * @brief Test: Pmic_esmClrStatus() with NULL status pointer.
 */
void test_neg_esm_esmClrStatus_nullPointer(void);

/**
 * @brief Test: Pmic_esmClrStatus() with invalid validParams (0).
 */
void test_neg_esm_esmClrStatus_invalidParams(void);

/**
 * @brief Test: Pmic_esmClrStatus() with unsupported ERR_CNT parameter.
 */
void test_neg_esm_esmClrStatus_unsupportedErrCnt(void);

/* ========================================================================== */
/*                         Unity Framework Functions                          */
/* ========================================================================== */

/* setUp() and tearDown() are defined in test_runner.c, not here */

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* ESM_TEST_H */
