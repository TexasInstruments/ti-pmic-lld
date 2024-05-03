/****************************************************************************
 *  Copyright (C) 2024 Texas Instruments Incorporated
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
 ****************************************************************************/

/**
 *  @file  pmic_fsmRotationalOnOff_test.h
 *
 *  @brief  This file contains all the testing related files APIs for the PMIC core
 *          and basic register lock/unlock and other miscellaneous tests.
 */
#ifndef __PMIC_LOWIQTIMER_TEST_H__
#define __PMIC_LOWIQTIMER_TEST_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */

#include "pmic_test_common.h"
#include "pmic_low_iq_timer.h"
#include "regmap/timer.h"
#include "pmic_fsmRotationalOnOff_test.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void test_pmic_set_tmrconfig(uint32_t tmr_cfg);
void test_pmic_get_tmrconfig();
void test_pmic_set_timerprescale();
void test_pmic_get_timerprescale();
void test_pmic_timer_clear();
void test_pmic_get_timer_cnt();
void *test_pmic_lowIQTimer(void *args);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_LOWIQTIMER_TEST_H__ */
