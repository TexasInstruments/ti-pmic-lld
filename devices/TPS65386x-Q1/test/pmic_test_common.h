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



#ifndef PMIC_TEST_COMMON_H
#define PMIC_TEST_COMMON_H

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
/* COMMON INCLUDES */
#include <string.h>
#include <stdbool.h>

/* PMIC INCLUDES */
#include "pmic.h"
#include "pmic_core.h"
#include "pmic_io.h"

/* DEVICE INCLUDES - hardware only */
#ifndef BUILD_MOCK
#include <kernel/dpl/DebugP.h>
#include "ti_drivers_config.h"
#include "ti_board_open_close.h"
#include "ti_drivers_open_close.h"
#else
/* Mock build - use printf for debug output */
#include <stdio.h>
#define DebugP_log(...) printf(__VA_ARGS__)
#endif

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
#define PMIC_MCSPI_MSGSIZE              (1U)
#define SemaphoreP_OK                   (0U)
#define SemaphoreP_WAIT_FOREVER         (~((uint32_t)0U))

#ifdef BUILD_MOCK
/* Mock semaphore for single-threaded testing */
typedef struct {
    uint32_t dummy;
} SemaphoreP_Object;

static inline int32_t SemaphoreP_constructMutex(SemaphoreP_Object *obj) {
    (void)obj;
    return SemaphoreP_OK;
}

static inline int32_t SemaphoreP_pend(SemaphoreP_Object *obj, uint32_t timeout) {
    (void)obj; (void)timeout;
    return SemaphoreP_OK;
}

static inline void SemaphoreP_post(SemaphoreP_Object *obj) {
    (void)obj;
}

static inline void SemaphoreP_destruct(SemaphoreP_Object *obj) {
    (void)obj;
}
#endif

/*==========================================================================*/
/*                         Structures and Enums                             */
/*==========================================================================*/

/* ========================================================================== */
/*                          Function Declarations                             */
/* ========================================================================== */

SemaphoreP_Object gpmicCoreObj;

uint32_t gPmicMcspiTxBuffer[PMIC_MCSPI_MSGSIZE];
uint32_t gPmicMcspiRxBuffer[PMIC_MCSPI_MSGSIZE];

uint8_t IO_calcCRC8(uint8_t cmd, uint8_t rdwr, uint8_t dat);

#ifndef BUILD_MOCK
int32_t PMIC_mcspiReadRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t* data);
int32_t PMIC_mcspiWriteRegister(MCSPI_Handle handle, MCSPI_Transaction *spiTransaction, uint8_t cmd, uint8_t data);
void mcspi_mux_pmic(void);
void delay(uint32_t milliseconds);
#endif

void test_pmic_criticalSectionStartFn(uint8_t resource);
void test_pmic_criticalSectionStopFn(uint8_t resource);
int32_t test_pmic_appInit(Pmic_Handle_t **pmicCoreHandle,
                          Pmic_HandleCfg_t     *pmicConfigData);
static void test_pmic_SemaphoreDeInit(void);
int32_t test_pmic_regRead(const Pmic_Handle_t *handle,
                          uint8_t             page,
                          uint8_t             regAddr,
                          uint8_t            *buffer,
                          uint8_t             bufLen);
int32_t test_pmic_regWrite(const Pmic_Handle_t *handle,
                           uint8_t             page,
                           uint8_t             regAddr,
                           const uint8_t      *buffer,
                           uint8_t             bufLen);
void test_check_lock_config_reg(Pmic_Handle_t  *pmicCorehandle);
void test_check_tmr_cnt_config_reg(Pmic_Handle_t  *pmicCorehandle);
void test_pmic_LockUnlock(Pmic_Handle_t  *pmicCorehandle, int unlock);
void test_pmic_CNT_LockUnlock(Pmic_Handle_t  *pmicCorehandle, int unlock);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PMIC_TEST_COMMON_H */

