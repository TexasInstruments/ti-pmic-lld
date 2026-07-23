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
/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define PMIC_MAX_DIAGNOSTIC_ID ((PMIC_ST_ID_ERROR_MAX > PMIC_ST_ID_WARNING_MAX) ? PMIC_ST_ID_ERROR_MAX : PMIC_ST_ID_WARNING_MAX)

/* ========================================================================== */
/*                          Structures and Enums                              */
/* ========================================================================== */

/**
 * @anchor Pmic_Diagnostic
 * @name PMIC Diagnostic Structure
 *
 * @brief Structure used to hold PMIC LLD system diagnostic information.
 *
 * @param errCnt Each index of this array maps to a specific error ID. The data
 * held at each index represents the count of occurrences for that error ID.
 *
 * @param warnCnt Each index of this array maps to a specific warning ID. The data
 * held at each index represents the count of occurrences for that warning ID.
 *
 * @param retryCnt Number of times a failed operation has been retried.
 *
 * @param errCntOverflow Each index of this array maps to a specific error ID.
 * The data held at each index represents whether the corresponding error counter
 * has reached the overflow threshold and has wrapped around to zero.
 *
 * @param warnCntOverflow Each index of this array maps to a specific warning
 * ID. The data held at each index represents whether the corresponding warning
 * counter has reached the overflow threshold and has wrapped around to zero.
 *
 * @param retryCntOverflow Flag to indicate if the retry counter has reached the
 * overflow threshold and has wrapped around to zero.
 */
typedef struct SystemDiagnostics_s {
    uint32_t errCnt[PMIC_ST_ID_ERROR_MAX + 1U];
    uint32_t warnCnt[PMIC_ST_ID_WARNING_MAX + 1U];
    uint32_t retryCnt;

    bool errCntOverflow[PMIC_ST_ID_ERROR_MAX + 1U];
    bool warnCntOverflow[PMIC_ST_ID_WARNING_MAX + 1U];
    bool retryCntOverflow;
} SystemDiagnostics_t;

/* ========================================================================== */
/*                           Variables and Data                               */
/* ========================================================================== */

/**
 * @brief Driver system diagnostics.
 *
 * @attention It shall be guaranteed that all accesses of this structure are
 * protected by a critical section.
 */
static SystemDiagnostics_t sysDiagnostics = {0U};

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
void Pmic_criticalSectionStart(const Pmic_Handle_t *handle, uint8_t resource) {
    if ((handle != (void *)0U) && (handle->criticalSectionStart != (void *)0U)) {
        handle->criticalSectionStart(resource);
    }
}

void Pmic_criticalSectionStop(const Pmic_Handle_t *handle, uint8_t resource) {
    if ((handle != (void *)0U) && (handle->criticalSectionStop != (void *)0U)) {
        handle->criticalSectionStop(resource);
    }
}

void Pmic_timerWaitMs(const Pmic_Handle_t *handle, uint32_t ms) {
    if ((handle != (void *)0U) && (handle->timerWaitMs != (void *)0U)) {
        handle->timerWaitMs(ms);
    }
}

void Pmic_irqResponseCallback(const Pmic_Handle_t *handle) {
    if ((handle != (void *)0U) && (handle->irqResponseCallback != (void *)0U)) {
        handle->irqResponseCallback();
    }
}

/* ========================================================================== */
/*                         Static Function Definitions                        */
/* ========================================================================== */

static inline uint16_t getStatusCodeType(int32_t status) {
    uint32_t shifted = (uint32_t)status >> PMIC_ST_TYPE_SHIFT;
    return (uint16_t)(shifted & 0xFFFFU);
}

static inline uint16_t getStatusCodeId(int32_t status) {
    return (uint16_t)((uint32_t)status & 0xFFFFU);
}

/*
 * @brief Validates that a statusId is within bounds for error counters.
 * @param statusId The status ID to validate
 * @return true if statusId is valid for error arrays, false otherwise
 */
static inline bool isValidErrorId(uint16_t statusId) {
    return (statusId <= PMIC_ST_ID_ERROR_MAX);
}

/*
 * @brief Validates that a statusId is within bounds for warning counters.
 * @param statusId The status ID to validate
 * @return true if statusId is valid for warning arrays, false otherwise
 */
static inline bool isValidWarningId(uint16_t statusId) {
    return (statusId <= PMIC_ST_ID_WARNING_MAX);
}

/*
 * REQUIREMENT:
 * Status type and ID must already be validated in parent function.
 * Critical section must already be started before this function is called.
 * @return PMIC_ST_SUCCESS on success, PMIC_ST_ERR_INV_STATUS_ID if statusId is out of bounds
 */
static inline int32_t incrementErrCnt(int32_t status) {
    const uint16_t statusId = getStatusCodeId(status);
    if (!isValidErrorId(statusId)) {
        return PMIC_ST_ERR_INV_STATUS_ID;
    }
    if (sysDiagnostics.errCnt[statusId] >= PMIC_ERR_CNT_OVERFLOW_THR) {
        sysDiagnostics.errCntOverflow[statusId] = (bool)true;
        sysDiagnostics.errCnt[statusId] = 0U;
    } else {
        sysDiagnostics.errCnt[statusId]++;
    }
    return PMIC_ST_SUCCESS;
}

static inline void copyDiagnostic(const Pmic_Diagnostic_t *src, Pmic_Diagnostic_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_Diagnostic_t));
}

/*
 * REQUIREMENT:
 * Status type and ID must already be validated in parent function.
 * Critical section must already be started before this function is called.
 * @return PMIC_ST_SUCCESS on success, PMIC_ST_ERR_INV_STATUS_ID if statusId is out of bounds
 */
static inline int32_t incrementWarnCnt(int32_t status) {
    const uint16_t statusId = getStatusCodeId(status);
    if (!isValidWarningId(statusId)) {
        return PMIC_ST_ERR_INV_STATUS_ID;
    }
    if (sysDiagnostics.warnCnt[statusId] >= PMIC_WARN_CNT_OVERFLOW_THR) {
        sysDiagnostics.warnCntOverflow[statusId] = (bool)true;
        sysDiagnostics.warnCnt[statusId] = 0U;
    } else {
        sysDiagnostics.warnCnt[statusId]++;
    }
    return PMIC_ST_SUCCESS;
}

static int32_t statusCodeCheck(int32_t status) {
    const uint16_t statusType = getStatusCodeType(status);
    const uint16_t statusId = getStatusCodeId(status);


    switch (statusType) {
        case PMIC_ST_TYPE_SUCCESS:
            if (statusId > PMIC_ST_ID_SUCCESS_MAX) {
                return PMIC_ST_ERR_INV_STATUS_ID;
            }
            break;
        case PMIC_ST_TYPE_ERROR:
            if (statusId > PMIC_ST_ID_ERROR_MAX) {
                return PMIC_ST_ERR_INV_STATUS_ID;
            }
            break;
        case PMIC_ST_TYPE_WARNING:
            if (statusId > PMIC_ST_ID_WARNING_MAX) {
                return PMIC_ST_ERR_INV_STATUS_ID;
            }
            break;
        default:
            return PMIC_ST_ERR_INV_STATUS_TYPE;
            break;
    }

    return PMIC_ST_SUCCESS;
}

static int32_t getDiagnosticEntry(Pmic_Diagnostic_t *diag) {
    const uint16_t statusType = getStatusCodeType(diag->code);
    const uint16_t statusId = getStatusCodeId(diag->code);

    if (diag->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (statusType == PMIC_ST_TYPE_ERROR) {
        if (!isValidErrorId(statusId)) {
            return PMIC_ST_ERR_INV_STATUS_ID;
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_CNT_VALID)) {
            diag->cnt = sysDiagnostics.errCnt[statusId];
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_FLAG_VALID)) {
            diag->flag = sysDiagnostics.errCntOverflow[statusId];
        }
    } else if (statusType == PMIC_ST_TYPE_WARNING) {
        if (!isValidWarningId(statusId)) {
            return PMIC_ST_ERR_INV_STATUS_ID;
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_CNT_VALID)) {
            diag->cnt = sysDiagnostics.warnCnt[statusId];
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_FLAG_VALID)) {
            diag->flag = sysDiagnostics.warnCntOverflow[statusId];
        }
    } else {
        return PMIC_ST_ERR_INV_STATUS_TYPE;
    }

    return PMIC_ST_SUCCESS;
}

static int32_t clrDiagnosticEntry(const Pmic_Diagnostic_t *diag) {
    const uint16_t statusType = getStatusCodeType(diag->code);
    const uint16_t statusId = getStatusCodeId(diag->code);

    if (diag->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (statusType == PMIC_ST_TYPE_ERROR) {
        if (!isValidErrorId(statusId)) {
            return PMIC_ST_ERR_INV_STATUS_ID;
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_CNT_VALID)) {
            sysDiagnostics.errCnt[statusId] = 0U;
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_FLAG_VALID)) {
            sysDiagnostics.errCntOverflow[statusId] = (bool)false;
        }
    } else if (statusType == PMIC_ST_TYPE_WARNING) {
        if (!isValidWarningId(statusId)) {
            return PMIC_ST_ERR_INV_STATUS_ID;
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_CNT_VALID)) {
            sysDiagnostics.warnCnt[statusId] = 0U;
        }
        if (Pmic_validParamCheck(diag->validParams, PMIC_COMMON_DIAGNOSTIC_FLAG_VALID)) {
            sysDiagnostics.warnCntOverflow[statusId] = (bool)false;
        }
    } else {
        return PMIC_ST_ERR_INV_STATUS_TYPE;
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_logStatus(const Pmic_Handle_t *handle, int32_t status) {
    int32_t statusCheck = 0;
    int32_t incrementResult = 0;
    uint16_t statusType = 0U;

    // If status is success, no need to log
    if (status == PMIC_ST_SUCCESS) {
        return status;
    }

    statusCheck = statusCodeCheck(status);

    // Do not log if critical section functions are not available
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        if (statusCheck != PMIC_ST_SUCCESS) {
            return statusCheck;
        } else {
            return status;
        }
    }

    statusType = getStatusCodeType(status);
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);

    // If status isn't valid (invalid type or ID), increment error counter and return
    if (statusCheck != PMIC_ST_SUCCESS) {
        incrementResult = incrementErrCnt(statusCheck);
        Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
        if (incrementResult != PMIC_ST_SUCCESS) {
            return incrementResult;
        }
        return statusCheck;
    }

    // Update diagnostic information for error type
    if (statusType == PMIC_ST_TYPE_ERROR) {
        incrementResult = incrementErrCnt(status);
        if (incrementResult != PMIC_ST_SUCCESS) {
            Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
            return incrementResult;
        }
    }

    // Update diagnostic information for warning type
    if (statusType == PMIC_ST_TYPE_WARNING) {
        incrementResult = incrementWarnCnt(status);
        if (incrementResult != PMIC_ST_SUCCESS) {
            Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
            return incrementResult;
        }
    }

    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
    return status;
}

int32_t Pmic_getDiagnostic(const Pmic_Handle_t *handle, Pmic_Diagnostic_t *diagnostic) {
    Pmic_Diagnostic_t localDiagnostic = (Pmic_Diagnostic_t){0};
    int32_t statusCheck = 0;
    int32_t retVal = 0;

    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    if (diagnostic == NULL) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (diagnostic->validParams == 0U) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    copyDiagnostic(diagnostic, &localDiagnostic);

    statusCheck = statusCodeCheck(localDiagnostic.code);
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);

    if (statusCheck != PMIC_ST_SUCCESS) {
        (void)incrementErrCnt(statusCheck);
        Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
        return statusCheck;
    }

    retVal = getDiagnosticEntry(&localDiagnostic);
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    if (retVal != PMIC_ST_SUCCESS) {
        return Pmic_logStatus(handle, retVal);
    }

    copyDiagnostic(&localDiagnostic, diagnostic);
    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

static int32_t validateDiagnosticsArgs(const Pmic_Handle_t *handle,
                                       const Pmic_Diagnostic_t *diagnostic,
                                       uint8_t numDiagnostics)
{
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) ||
        (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }
    if (diagnostic == NULL) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }
    if (numDiagnostics == 0U) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }
    if (numDiagnostics > (PMIC_MAX_DIAGNOSTIC_ID + 1U)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_getDiagnostics(const Pmic_Handle_t *handle, Pmic_Diagnostic_t diagnostic[], uint8_t numDiagnostics) {
    Pmic_Diagnostic_t localDiagnostics[PMIC_MAX_DIAGNOSTIC_ID + 1U] = {0};
    int32_t entryStatus = 0;
    int32_t status = validateDiagnosticsArgs(handle, diagnostic, numDiagnostics);

    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    for (uint8_t i = 0U; i < numDiagnostics; i++) {
        copyDiagnostic(&diagnostic[i], &localDiagnostics[i]);
    }

    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    for (uint8_t i = 0U; i < numDiagnostics; i++) {
        entryStatus = getDiagnosticEntry(&localDiagnostics[i]);
        if (entryStatus != PMIC_ST_SUCCESS) {
            (void)incrementErrCnt(entryStatus);
            Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
            return entryStatus;
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    for (uint8_t i = 0U; i < numDiagnostics; i++) {
        copyDiagnostic(&localDiagnostics[i], &diagnostic[i]);
    }

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_clrDiagnostic(const Pmic_Handle_t *handle, const Pmic_Diagnostic_t *diagnostic) {
    Pmic_Diagnostic_t localDiagnostic = (Pmic_Diagnostic_t){0};
    int32_t statusCheck = 0;
    int32_t retVal = 0;

    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    if (diagnostic == NULL) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (diagnostic->validParams == 0U) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    copyDiagnostic(diagnostic, &localDiagnostic);

    statusCheck = statusCodeCheck(localDiagnostic.code);
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);

    if (statusCheck != PMIC_ST_SUCCESS) {
        (void)incrementErrCnt(statusCheck);
        Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
        return statusCheck;
    }

    retVal = clrDiagnosticEntry(&localDiagnostic);
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    if (retVal != PMIC_ST_SUCCESS) {
        return Pmic_logStatus(handle, retVal);
    }

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_clrDiagnostics(const Pmic_Handle_t *handle, const Pmic_Diagnostic_t diagnostic[], uint8_t numDiagnostics) {
    Pmic_Diagnostic_t localDiagnostics[PMIC_MAX_DIAGNOSTIC_ID + 1U] = {0};
    int32_t entryStatus = 0;
    int32_t status = validateDiagnosticsArgs(handle, diagnostic, numDiagnostics);

    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    for (uint8_t i = 0U; i < numDiagnostics; i++) {
        copyDiagnostic(&diagnostic[i], &localDiagnostics[i]);
    }

    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    for (uint8_t i = 0U; i < numDiagnostics; i++) {
        entryStatus = clrDiagnosticEntry(&localDiagnostics[i]);
        if (entryStatus != PMIC_ST_SUCCESS) {
            (void)incrementErrCnt(entryStatus);
            Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);
            return entryStatus;
        }
    }
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_clrDiagnosticsAll(const Pmic_Handle_t *handle) {
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);

    // Reset all error counters and flags
    (void)memset(sysDiagnostics.errCnt, 0, sizeof(sysDiagnostics.errCnt));
    (void)memset(sysDiagnostics.errCntOverflow, 0, sizeof(sysDiagnostics.errCntOverflow));

    // Reset all warning counters and flags
    (void)memset(sysDiagnostics.warnCnt, 0, sizeof(sysDiagnostics.warnCnt));
    (void)memset(sysDiagnostics.warnCntOverflow, 0, sizeof(sysDiagnostics.warnCntOverflow));

    // Reset retry counter and associated flag
    sysDiagnostics.retryCnt = 0U;
    sysDiagnostics.retryCntOverflow = (bool)false;

    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_getRetryCnt(const Pmic_Handle_t *handle, uint32_t *retryCnt) {
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    if (retryCnt == NULL) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    // Extract retry count
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    *retryCnt = sysDiagnostics.retryCnt;
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_getRetryCntOverflow(const Pmic_Handle_t *handle, bool *reachedThreshold) {
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    if (reachedThreshold == NULL) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    // Extract retry threshold flag
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    *reachedThreshold = sysDiagnostics.retryCntOverflow;
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_clrRetryCnt(const Pmic_Handle_t *handle) {
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    // Clear retry counter and flag
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    sysDiagnostics.retryCnt = 0U;
    sysDiagnostics.retryCntOverflow = (bool)false;
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_incrementRetryCnt(const Pmic_Handle_t *handle) {
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    // Increment retry counter. If threshold is reached, reset counter and set flag
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    if (sysDiagnostics.retryCnt >= PMIC_RETRY_CNT_OVERFLOW_THR) {
        sysDiagnostics.retryCntOverflow = (bool)true;
        sysDiagnostics.retryCnt = 0U;
    } else {
        sysDiagnostics.retryCnt++;
    }
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}

int32_t Pmic_clrRetryCntOverflow(const Pmic_Handle_t *handle) {
    if ((handle == NULL) || (handle->criticalSectionStart == NULL) || (handle->criticalSectionStop == NULL)) {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_HANDLE);
    }

    // Clear retry counter flag
    Pmic_criticalSectionStart(handle, PMIC_DIAGNOSTIC);
    sysDiagnostics.retryCntOverflow = (bool)false;
    Pmic_criticalSectionStop(handle, PMIC_DIAGNOSTIC);

    return Pmic_logStatus(handle, PMIC_ST_SUCCESS);
}
