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
 *  @file  pmic_test_common.c
 *
 *  @brief  This file contains all the testing related files APIs for the common tests.
 */

#ifndef __PMIC_TEST_COMMON_H__
#define __PMIC_TEST_COMMON_H__

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* COMMON INCLUDES */
#include <string.h>
#include <stdbool.h>

/* PMIC INCLUDES */
#include "pmic.h"
#include "pmic_core.h"
#include "private/pmic_io_priv.h"

/* DEVICE INCLUDES */
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define PMIC_MCSPI_MSGSIZE              (1U)
#define SemaphoreP_OK                   (0U)
#define SemaphoreP_WAIT_FOREVER         (~((uint32_t)0U))

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

SemaphoreP_Object gpmicCoreObj;

uint32_t gPmicMcspiTxBuffer[PMIC_MCSPI_MSGSIZE];
uint32_t gPmicMcspiRxBuffer[PMIC_MCSPI_MSGSIZE];

uint8_t Pmic_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat);
int32_t PMIC_mcspiReadRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t* data);
int32_t PMIC_mcspiWriteRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t data);

void mcspi_mux_pmic(void);
void delay(uint32_t milliseconds);
void test_pmic_criticalSectionStartFn(void);
void test_pmic_criticalSectionStopFn(void);
int32_t test_pmic_appInit(Pmic_CoreHandle_t **pmicCoreHandle,
                          Pmic_CoreCfg_t     *pmicConfigData);
static void test_pmic_SemaphoreDeInit(void);
int32_t test_pmic_regRead(Pmic_CoreHandle_t  *pmicCorehandle,
                          uint8_t             instType,
                          uint16_t            regAddr,
                          uint8_t            *pBuf,
                          uint8_t             bufLen);
int32_t test_pmic_regWrite(Pmic_CoreHandle_t  *pmicCorehandle,
                           uint8_t             instType,
                           uint16_t            regAddr,
                           uint8_t            *pBuf,
                           uint8_t             bufLen);
void test_check_lock_config_reg(Pmic_CoreHandle_t  *pmicCorehandle);
void test_check_tmr_cnt_config_reg(Pmic_CoreHandle_t  *pmicCorehandle);
void test_pmic_LockUnlock(Pmic_CoreHandle_t  *pmicCorehandle, int unlock);
void test_pmic_CNT_LockUnlock(Pmic_CoreHandle_t  *pmicCorehandle, int unlock);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __PMIC_TEST_COMMON_H__ */

