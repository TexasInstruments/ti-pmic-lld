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
#ifndef __IO_TEST_H__
#define __IO_TEST_H__

/**
 * @file io_test.h
 * @brief Contains macros/defines and test declarations specific to testing the
 * IO module.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "platform.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

void io_test(void *args);

void test_negative_Pmic_ioTxByte_nullParam_handle(void);
void test_negative_Pmic_ioRxByte_nullParam_handle(void);
void test_negative_Pmic_ioRxByte_nullParam_rxBuffer(void);
void test_negative_Pmic_ioTxByte_CS_nullParam_handle(void);
void test_negative_Pmic_ioRxByte_CS_nullParam_handle(void);
void test_negative_Pmic_ioRxByte_CS_nullParam_rxBuffer(void);
void test_negative_Pmic_ioGetCrcEnableState_nullParam_handle(void);
void test_negative_Pmic_ioGetCrcEnableState_nullParam_isEnabled(void);
void test_negative_Pmic_ioSetCrcEnableState_nullParam_handle(void);
void test_negative_Pmic_ioCrcEnable_nullParam_handle(void);
void test_negative_Pmic_ioCrcDisable_nullParam_handle(void);
void test_positive_Pmic_ioTxByte_Pmic_ioRxByte_writeReadScratchpadReg1To4(void);
void test_positive_Pmic_ioTxByte_CS_Pmic_ioRxByte_CS_writeReadScratchpadReg1To4(void);
void test_positive_setGetCrcEnableState(void);
void test_positive_enableDisableCrc(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /*__IO_TEST_H__*/
