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
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"
#include "pmic_power.h"
#include "regmap/power.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */
// While the UV/OV thresholds have different thresholds depending on whether it
// is for Buck, LDO, LSW, or VCCA, the actual values that go in the register are
// all 2-bit values and have the same range of supported values.
#define UV_OV_THR_MIN_VALUE   ((uint8_t)0U)
#define UV_OV_THR_MAX_VALUE   ((uint8_t)3U)

// Min and max voltages supported for BUCK1/2/3 VSET
#define BUCK_VSET_MIN_MV      ((uint16_t)900U)
#define BUCK_VSET_MAX_MV      ((uint16_t)1900U)
#define BUCK_VSET_STEP_MV     ((uint16_t)20U)

// Min and max voltages supported for LDO_LS1_VMON1, LS2_VMON2, and VCCA_VMON
// PG_LEVEL
#define VMON_PG_LEVEL_MIN_MV  ((uint16_t)600U)
#define VMON_PG_LEVEL_MAX_MV  ((uint16_t)3400U)
#define VMON_PG_LEVEL_STEP_MV ((uint16_t)25U)

// Register mask and shift values for Buck VSET, and VMON PGSET fields. Bucks
// get the whole register, VMONs get all but the highest bit.
#define VSET_PGSET_SHIFT      (0U)
#define BUCK_VSET_MASK        (0xFFU)
#define VMON_PGSET_MASK       (0x7FU)

/* Power resource field width - each resource uses 2-bit field in registers */
#define PWR_RSRC_FIELD_WIDTH_BITS  ((uint8_t)2U)

typedef struct SetResourceProcessor_s {
    uint16_t validParam;
    int32_t (*fptr)(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config);
} SetResourceProcessor_t;

typedef struct GetResourceProcessor_s {
    uint16_t validParam;
    int32_t (*fptr)(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config);
} GetResourceProcessor_t;

static const uint8_t PwrResourceBucks[] = {
    (uint8_t)PMIC_PWR_RSRC_BUCK1,
    (uint8_t)PMIC_PWR_RSRC_BUCK2,
    (uint8_t)PMIC_PWR_RSRC_BUCK3,
};

static const uint8_t PwrResourceVmonCapable[] = {
    (uint8_t)PMIC_PWR_RSRC_BUCK1,
    (uint8_t)PMIC_PWR_RSRC_BUCK2,
    (uint8_t)PMIC_PWR_RSRC_BUCK3,
    (uint8_t)PMIC_PWR_RSRC_LDO_LS1_VMON1,
    (uint8_t)PMIC_PWR_RSRC_LS2_VMON2,
    (uint8_t)PMIC_PWR_RSRC_VCCA_VMON,
};

static const uint8_t PwrResourceImonCapable[] = {
    (uint8_t)PMIC_PWR_RSRC_BUCK1,
    (uint8_t)PMIC_PWR_RSRC_BUCK2,
    (uint8_t)PMIC_PWR_RSRC_BUCK3,
    (uint8_t)PMIC_PWR_RSRC_LDO_LS1_VMON1,
    (uint8_t)PMIC_PWR_RSRC_LS2_VMON2,
};

typedef enum {
    PWR_RSRC_BUCKS,
    PWR_RSRC_VMON_CAPABLE,
    PWR_RSRC_IMON_CAPABLE,
} tPowerResourceTypes;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static inline void PWR_copyPowerSequenceCfg(const Pmic_PowerSequenceCfg_t *src, Pmic_PowerSequenceCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_PowerSequenceCfg_t));
}

static inline void PWR_copyPowerResourceCfg(const Pmic_PowerResourceCfg_t *src, Pmic_PowerResourceCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_PowerResourceCfg_t));
}

static bool PWR_isResourceValid(const uint8_t validResources[], uint8_t numValid, uint8_t resource)
{
    bool isValid = false;

    for (uint8_t i = 0U; (i < numValid) && !isValid; i++) {
        if (resource == validResources[i]) {
            isValid = true;
        }
    }

    return isValid;
}

static inline bool PWR_isInRangeU16(uint16_t min, uint16_t max, uint16_t param)
{
    return (param >= min) && (param <= max);
}

static inline bool PWR_isInRangeU8(uint8_t min, uint8_t max, uint8_t param)
{
    return (param >= min) && (param <= max);
}

static int32_t PWR_validateParams(uint8_t resource, uint8_t value, tPowerResourceTypes type, uint8_t min, uint8_t max)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t *validResources;
    uint8_t numResources = 0U;

    switch (type) {
        case PWR_RSRC_BUCKS:
            validResources = (const uint8_t *)&PwrResourceBucks;
            numResources = COUNT(PwrResourceBucks);
            break;
        case PWR_RSRC_VMON_CAPABLE:
            validResources = (const uint8_t *)&PwrResourceVmonCapable;
            numResources = COUNT(PwrResourceVmonCapable);
            break;
        case PWR_RSRC_IMON_CAPABLE:
            validResources = (const uint8_t *)&PwrResourceImonCapable;
            numResources = COUNT(PwrResourceImonCapable);
            break;
        default: /* LCOV_EXCL_START */
            status = PMIC_ST_ERR_INV_PARAM;
            break; /* LCOV_EXCL_STOP */
    }

    // Validate resource requested supports configuration of this parameter
    if ((status == PMIC_ST_SUCCESS) && !PWR_isResourceValid(validResources, numResources, resource)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Validate parameter within allowed range
    if ((status == PMIC_ST_SUCCESS) && !PWR_isInRangeU8(min, max, value)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

static int32_t PWR_readModifyWrite(const Pmic_Handle_t *handle, uint8_t regAddr, uint8_t shift, uint8_t mask, uint8_t value)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, regAddr, &regData);

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField(&regData, shift, mask, value);
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_pwrSetResourceEnable(const Pmic_Handle_t *handle, uint8_t resource, bool enable)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Validate that the requested resource is within range and supports
    // enablement. All power resource support enable/disable, this is mostly
    // just parameter validation.
    if ((status == PMIC_ST_SUCCESS) && (resource > PMIC_PWR_RSRC_GPO)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, BLOCK_EN_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        // PWR_RSRC types are defined the same as the shift values for each bit in
        // this register, therefore the resource can be used directly as the shift
        // value
        Pmic_setBitField_b(&regData, resource, enable);
        status = Pmic_ioTxByte(handle, BLOCK_EN_CTRL_REG, regData);
    }

    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrGetResourceEnable(const Pmic_Handle_t *handle, uint8_t resource, bool *isEnabled)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (isEnabled == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Validate that the requested resource is within range and supports
    // enablement. All power resource support enable/disable, this is mostly
    // just parameter validation.
    if ((status == PMIC_ST_SUCCESS) && (resource > PMIC_PWR_RSRC_GPO)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, BLOCK_EN_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        // PWR_RSRC types are defined the same as the shift values for each bit in
        // this register, therefore the resource can be used directly as the shift
        // value
        *isEnabled = Pmic_getBitField_b(regData, resource);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t PWR_setModeCfgLdoLs1Vmon1(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    struct LdoLs1ModeConfig_s {
        uint8_t bypConfig;
        uint8_t vmon1Sel;
        uint8_t lswConfig;
    };

    // For LDO_LS1_VMON1
    //
    // LDO_LS1_VMON1_PG_LEVEL : LDO_LS1_BYP_CONFIG -> Select LDO or Bypass mode (LSW)
    // FUNC_CONF : LDO_LS1_VMON1_SEL -> Select LDO or VMON1 mode
    // FUNC_CONF : LDO_LS1_LSW_CONFIG -> Select LDO/BYP mode or LSW mode
    // "REG"  -> LDO_LS1_BYP_CONFIG=0, LDO_LS1_VMON1_SEL=0, LDO_LS1_LSW_CONFIG=0
    // "BYP"  -> LDO_LS1_BYP_CONFIG=1, LDO_LS1_VMON1_SEL=0, LDO_LS1_LSW_CONFIG=0
    // "LSW"  -> LDO_LS1_BYP_CONFIG=1, LDO_LS1_VMON1_SEL=0, LDO_LS1_LSW_CONFIG=1
    // "VMON" -> LDO_LS1_BYP_CONFIG=1, LDO_LS1_VMON1_SEL=1, LDO_LS1_LSW_CONFIG=1 ??? (unsure about this one)
    const struct LdoLs1ModeConfig_s configurations[] = {
        // PMIC_PWR_RSRC_MODE_REG
        { .bypConfig = 0U, .vmon1Sel = 0U, .lswConfig = 0U },
        // PMIC_PWR_RSRC_MODE_BYP
        { .bypConfig = 1U, .vmon1Sel = 0U, .lswConfig = 0U },
        // PMIC_PWR_RSRC_MODE_LSW
        { .bypConfig = 1U, .vmon1Sel = 0U, .lswConfig = 1U },
        // PMIC_PWR_RSRC_MODE_VMON
        { .bypConfig = 1U, .vmon1Sel = 1U, .lswConfig = 1U },
    };

    struct LdoLs1ModeConfig_s modeConfig = configurations[config->mode];

    // Handle LDO_LS1_VMON1_PG_LEVEL fields first, this is only LDO_LS1_BYP_CONFIG
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, LDO_LS1_VMON1_PG_LEVEL_REG, &regData);

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField(&regData, LDO_LS1_BYP_CONFIG_SHIFT, LDO_LS1_BYP_CONFIG_MASK, modeConfig.bypConfig);
        status = Pmic_ioTxByte(handle, LDO_LS1_VMON1_PG_LEVEL_REG, regData);
    }

    // Handle FUNC_CONF fields, this covers the remaining fields
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, FUNC_CONF_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField(&regData, LDO_LS1_VMON1_SEL_SHIFT, LDO_LS1_VMON1_SEL_MASK, modeConfig.vmon1Sel);
        Pmic_setBitField(&regData, LDO_LS1_LSW_CONFIG_SHIFT, LDO_LS1_LSW_CONFIG_MASK, modeConfig.lswConfig);
        status = Pmic_ioTxByte(handle, FUNC_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

static int32_t PWR_setModeCfgLs2Vmon2(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // For LS2_VMON2
    // FUNC_CONF : LS2_VMON2_GPO_SEL -> Select LSW or VMON mode (also has option for SYNCCLKIN)
    // 0 = LOAD_SWITCH mode
    // 1 = VMON2 mode
    const uint8_t mode = (config->mode == PMIC_PWR_RSRC_MODE_LSW) ? 0U : 1U;

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, FUNC_CONF_REG, &regData);

    if (status == PMIC_ST_SUCCESS) {
        Pmic_setBitField(&regData, LS2_VMON2_GPO_SEL_SHIFT, LS2_VMON2_GPO_SEL_MASK, mode);
        status = Pmic_ioTxByte(handle, FUNC_CONF_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setModeCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t mode = config->mode;

    // Validate parameter and set up any resource specific variables
    switch (config->resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            // BUCKs do not have mode configurability, assert that the user
            // requested "REG" mode, as long as that is true proceed without
            // error
            if (mode != PMIC_PWR_RSRC_MODE_REG) {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }
            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            // LDO_LS1_VMON1 supports all valid configurations, assert that the
            // user provided parameter is in range
            if (PWR_isInRangeU8((uint8_t)PMIC_PWR_RSRC_MODE_MIN, (uint8_t)PMIC_PWR_RSRC_MODE_MAX, (uint8_t)mode) == false) {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            if (status == PMIC_ST_SUCCESS) {
                status = PWR_setModeCfgLdoLs1Vmon1(handle, config);
            }

            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            // LS2_VMON2 supports selection between LSW or VMON, assert that the
            // user didn't select "REG" or "BYP" mode
            if (PWR_isInRangeU8((uint8_t)PMIC_PWR_RSRC_MODE_REG, (uint8_t)PMIC_PWR_RSRC_MODE_BYP, (uint8_t)mode) == true) {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }

            if (status == PMIC_ST_SUCCESS) {
                status = PWR_setModeCfgLs2Vmon2(handle, config);
            }

            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            // VCCA_VMON does not have mode configurability, assert that the
            // user requested "VMON" mode, as long as this is true, proceed
            // without error.
            if (mode != PMIC_PWR_RSRC_MODE_VMON) {
                status = PMIC_ST_ERR_NOT_SUPPORTED;
            }
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

static int32_t PWR_getModeCfgLdoLs1Vmon1(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t pgLevelReg = 0U;
    uint8_t funcConfReg = 0U;
    uint8_t resolvedMode = 0U;
    bool bypConfig;
    bool vmon1Sel;
    bool lswConfig;

    // Perform necessary register reads to decode current state
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, LDO_LS1_VMON1_PG_LEVEL_REG, &pgLevelReg);

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, FUNC_CONF_REG, &funcConfReg);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    // For LDO_LS1_VMON1
    //
    // LDO_LS1_VMON1_PG_LEVEL : LDO_LS1_BYP_CONFIG -> Select LDO or Bypass mode (LSW)
    // FUNC_CONF : LDO_LS1_VMON1_SEL -> Select LDO or VMON1 mode
    // FUNC_CONF : LDO_LS1_LSW_CONFIG -> Select LDO/BYP mode or LSW mode
    // "REG"  -> LDO_LS1_BYP_CONFIG=0, LDO_LS1_VMON1_SEL=0, LDO_LS1_LSW_CONFIG=0
    // "BYP"  -> LDO_LS1_BYP_CONFIG=1, LDO_LS1_VMON1_SEL=0, LDO_LS1_LSW_CONFIG=0
    // "LSW"  -> LDO_LS1_BYP_CONFIG=1, LDO_LS1_VMON1_SEL=0, LDO_LS1_LSW_CONFIG=1
    // "VMON" -> LDO_LS1_BYP_CONFIG=1, LDO_LS1_VMON1_SEL=1, LDO_LS1_LSW_CONFIG=1 ??? (unsure about this one)
    bypConfig = Pmic_getBitField_b(pgLevelReg, LDO_LS1_BYP_CONFIG_SHIFT);
    vmon1Sel = Pmic_getBitField_b(funcConfReg, LDO_LS1_VMON1_SEL_SHIFT);
    lswConfig = Pmic_getBitField_b(funcConfReg, LDO_LS1_LSW_CONFIG_SHIFT);
    if ((!bypConfig) && (!vmon1Sel) && (!lswConfig)) {
        resolvedMode = PMIC_PWR_RSRC_MODE_REG;
    } else if ((bypConfig) && (!vmon1Sel) && (!lswConfig)) {
        resolvedMode = PMIC_PWR_RSRC_MODE_BYP;
    } else if ((bypConfig) && (!vmon1Sel) && (lswConfig)) {
        resolvedMode = PMIC_PWR_RSRC_MODE_LSW;
    } else if ((bypConfig) && (vmon1Sel) && (lswConfig)) {
        resolvedMode = PMIC_PWR_RSRC_MODE_VMON;
    } else {
        // This is an invalid state, return failure
        status = PMIC_ST_ERR_FAIL;
    }

    if (status == PMIC_ST_SUCCESS) {
        config->mode = resolvedMode;
    }

    return status;
}

static int32_t PWR_getModeCfgLs2Vmon2(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t hwModeVal = 0U;

    status = Pmic_ioRxByte_CS(handle, FUNC_CONF_REG, &regData);

    if (status == PMIC_ST_SUCCESS) {
        hwModeVal = Pmic_getBitField(regData, LS2_VMON2_GPO_SEL_SHIFT, LS2_VMON2_GPO_SEL_MASK);
    }

    if (status == PMIC_ST_SUCCESS) {
        if (hwModeVal == 0U) {
            config->mode = PMIC_PWR_RSRC_MODE_LSW;
        } else {
            config->mode = PMIC_PWR_RSRC_MODE_VMON;
        }
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getModeCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    switch (config->resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            // BUCKs do not have mode configurability, report they are in REG
            // mode
            config->mode = PMIC_PWR_RSRC_MODE_REG;
            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            status = PWR_getModeCfgLdoLs1Vmon1(handle, config);
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            status = PWR_getModeCfgLs2Vmon2(handle, config);
            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            // VCCA_CMON does not have mode configurabilite, report that it is
            // in VMON mode
            config->mode = PMIC_PWR_RSRC_MODE_VMON;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setIlimCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    const uint8_t value = config->ilim;
    uint8_t regAddr = 0U;

    if (config->resource > PMIC_PWR_RSRC_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        regAddr = BUCK1_MON_CONF_REG + config->resource;
    }

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_BUCKS, (uint8_t)PMIC_PWR_ILIM_MIN, (uint8_t)PMIC_PWR_ILIM_MAX);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, (uint8_t)BUCKx_ILIM_SHIFT, (uint8_t)BUCKx_ILIM_MASK, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getIlimCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;

    if (config->resource > PMIC_PWR_RSRC_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        regAddr = BUCK1_MON_CONF_REG + config->resource;
    }

    // Validate resource requested supports this parameter
    if ((status == PMIC_ST_SUCCESS) &&
        (PWR_isResourceValid(PwrResourceBucks, (uint8_t)COUNT(PwrResourceBucks), (uint8_t)config->resource) == false)) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->ilim = Pmic_getBitField(regData, BUCKx_ILIM_SHIFT, BUCKx_ILIM_MASK);
    }

    return status;
}

static int32_t PWR_getDeglitchParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask) {
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register and bitshifts need to be used:
    // - BUCKs, and LDO1_LS1_VMON1 are in BUCK_LDO_LS1_VMON1_DEGLIT_REG
    // - LS2_VMON2 and VCCA are in VCCA_MON_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = BUCK_LDO_LS1_VMON1_DEGLIT_REG;
            *shift = (uint8_t)(resource * PWR_RSRC_FIELD_WIDTH_BITS);
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = VCCA_MON_CONF_REG;
            *shift = LS2_VMON2_DEGLITCH_SEL_SHIFT;
            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            *regAddr = VCCA_MON_CONF_REG;
            *shift = VMONIN_DEGLITCH_SEL_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 2-bit control, set the mask accordingly
    *mask = (uint8_t)(0x3U << *shift);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setDeglitchCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->deglitch;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getDeglitchParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_VMON_CAPABLE, (uint8_t)PMIC_PWR_DEGLITCH_MIN, (uint8_t)PMIC_PWR_DEGLITCH_MAX);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getDeglitchCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceVmonCapable, (uint8_t)COUNT(PwrResourceVmonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getDeglitchParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->deglitch = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

static int32_t PWR_getUvThreshParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register needs to be used:
    // - BUCKs are in BUCKx_MON_CONF_REG
    // - LDO_LS1_VMON1 is in LDO_LS1_VMON1_MON_CONF_REG
    // - LS2_VMON2 is in LS2_VMON2_MON_CONF_REG
    // - VCCA_VMON is in VCCA_MON_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            // The BUCK registers are all defined in the same order as the
            // PMIC_PWR_RSRC definitions, so the base address of BUCK1_MON_CONF
            // can be used and the resource value can be used as an offset
            *regAddr = BUCK1_MON_CONF_REG + resource;
            *shift = BUCKx_UV_THR_SHIFT;
            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = LDO_LS1_VMON1_MON_CONF_REG;
            *shift = LDO_LS1_VMON1_UV_THR_SHIFT;
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = LS2_VMON2_MON_CONF_REG;
            *shift = LS2_VMON2_UV_THR_SHIFT;
            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            *regAddr = VCCA_MON_CONF_REG;
            *shift = VCCA_UV_THR_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 2-bit control, set the mask accordingly
    *mask = (uint8_t)(0x3U << *shift);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setUvThreshCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->uvThresh;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getUvThreshParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_VMON_CAPABLE, (uint8_t)UV_OV_THR_MIN_VALUE, (uint8_t)UV_OV_THR_MAX_VALUE);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getUvThreshCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceVmonCapable, (uint8_t)COUNT(PwrResourceVmonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getUvThreshParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->uvThresh = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

static int32_t PWR_getUvReactionParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register and bitshifts need to be used:
    // - BUCKs, and LDO1_LS1_VMON1 are in REG_UV_CONF_REG
    // - VCCA, and LS2_VMON2 are in VCCA_LS2_VMON2_UV_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = REG_UV_CONF_REG;
            *shift = (uint8_t)(resource * PWR_RSRC_FIELD_WIDTH_BITS);
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = VCCA_LS2_VMON2_UV_CONF_REG;
            *shift = LS2_VMON2_UV_SEL_SHIFT;
            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            *regAddr = VCCA_LS2_VMON2_UV_CONF_REG;
            *shift = VCCA_UV_SEL_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 2-bit control, set the mask accordingly
    *mask = (uint8_t)(0x3U << *shift);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setUvReactionCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->uvReaction;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getUvReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_VMON_CAPABLE, (uint8_t)PMIC_PWR_FAULT_REACT_MIN, (uint8_t)PMIC_PWR_FAULT_REACT_MAX);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getUvReactionCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceVmonCapable, (uint8_t)COUNT(PwrResourceVmonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getUvReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->uvReaction = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

static int32_t PWR_getOvThreshParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register needs to be used:
    // - BUCKs are in BUCKx_MON_CONF_REG
    // - LDO_LS1_VMON1 is in LDO_LS1_VMON1_MON_CONF_REG
    // - LS2_VMON2 is in LS2_VMON2_MON_CONF_REG
    // - VCCA_VMON is in VCCA_MON_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            // The BUCK registers are all defined in the same order as the
            // PMIC_PWR_RSRC definitions, so the base address of BUCK1_MON_CONF
            // can be used and the resource value can be used as an offset
            *regAddr = BUCK1_MON_CONF_REG + resource;
            *shift = BUCKx_OV_THR_SHIFT;
            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = LDO_LS1_VMON1_MON_CONF_REG;
            *shift = LDO_LS1_VMON1_OV_THR_SHIFT;
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = LS2_VMON2_MON_CONF_REG;
            *shift = LS2_VMON2_OV_THR_SHIFT;
            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            *regAddr = VCCA_MON_CONF_REG;
            *shift = VCCA_OV_THR_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 2-bit control, set the mask accordingly
    *mask = (uint8_t)(0x3U << *shift);
    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setOvThreshCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->ovThresh;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getOvThreshParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_VMON_CAPABLE, (uint8_t)UV_OV_THR_MIN_VALUE, (uint8_t)UV_OV_THR_MAX_VALUE);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

static int32_t PWR_getOvThreshCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceVmonCapable, (uint8_t)COUNT(PwrResourceVmonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getOvThreshParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->ovThresh = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

static int32_t PWR_getOvReactionParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register and bitshifts need to be used:
    // - BUCKs, and LDO1_LS1_VMON1 are in REG_OV_CONF_REG
    // - VCCA, and LS2_VMON2 are in VCCA_LS2_VMON2_OV_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = REG_OV_CONF_REG;
            *shift = (uint8_t)(resource * PWR_RSRC_FIELD_WIDTH_BITS);
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = VCCA_LS2_VMON2_OV_CONF_REG;
            *shift = LS2_VMON2_OV_SEL_SHIFT;
            break;
        case PMIC_PWR_RSRC_VCCA_VMON:
            *regAddr = VCCA_LS2_VMON2_OV_CONF_REG;
            *shift = VCCA_OV_SEL_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 2-bit control, set the mask accordingly
    *mask = (uint8_t)(0x3U << *shift);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setOvReactionCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->ovReaction;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getOvReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_VMON_CAPABLE, (uint8_t)PMIC_PWR_FAULT_REACT_MIN, (uint8_t)PMIC_PWR_FAULT_REACT_MAX);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getOvReactionCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceVmonCapable, (uint8_t)COUNT(PwrResourceVmonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getOvReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->ovReaction = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}


static int32_t PWR_getRvReactionParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register and bitshifts need to be used:
    // - BUCKs are in sequential BUCKx_MON_CONF_REG
    // - LDO1_LS1_VMON1 are in LDO_LS1_VMON1_MON_CONF_REG
    // - LS2_VMON2 are in LS2_VMON2_MON_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            *regAddr = BUCK1_MON_CONF_REG + resource;
            *shift = BUCKx_RV_CONF_SHIFT;
            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = LDO_LS1_VMON1_MON_CONF_REG;
            *shift = LDO_LS12_VMON_RV_CONF_SHIFT;
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = LS2_VMON2_MON_CONF_REG;
            *shift = LDO_LS12_VMON_RV_CONF_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 1-bit control, set the mask accordingly
    *mask = (uint8_t)(0x1UL << *shift);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setRvReactionCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->rvReaction;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getRvReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_IMON_CAPABLE, (uint8_t)PMIC_PWR_RV_CONF_INT_ONLY, (uint8_t)PMIC_PWR_RV_CONF_SHUT_DOWN);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getRvReactionCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceImonCapable, (uint8_t)COUNT(PwrResourceImonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getRvReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->rvReaction = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

static int32_t PWR_getScReactionParamLoc(uint8_t resource, uint8_t *regAddr, uint8_t *shift, uint8_t *mask)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register and bitshifts need to be used:
    // - BUCKs and LDO_LS1_VMON1 are in REG_SC_CONF_REG
    // - LS2_VMON2 are in VCCA_LS2_VMON2_UV_CONF_REG
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            *regAddr = REG_SC_CONF_REG;
            *shift = (uint8_t)(resource * PWR_RSRC_FIELD_WIDTH_BITS);
            break;
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = VCCA_LS2_VMON2_UV_CONF_REG;
            *shift = LS2_VMON2_SC_SEL_SHIFT;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // For all resources, this is a 2-bit control, set the mask accordingly
    *mask = (uint8_t)(0x3U << *shift);

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setScReactionCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    const uint8_t value = config->scReaction;

    // Get register address and bit shifts/masks to use for this resource
    status = PWR_getScReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);

    // Validate resource requested supports configuration and that parameter is
    // within allowed range
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_validateParams((uint8_t)config->resource, value,
                                    PWR_RSRC_IMON_CAPABLE, (uint8_t)PMIC_PWR_FAULT_REACT_MIN, (uint8_t)PMIC_PWR_FAULT_REACT_MAX);
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, shift, mask, value);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getScReactionCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t shift = 0U;
    uint8_t mask = 0U;
    uint8_t regData = 0U;

    // Validate resource requested supports this parameter
    if (PWR_isResourceValid(PwrResourceImonCapable, (uint8_t)COUNT(PwrResourceImonCapable), (uint8_t)config->resource) == false) {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Get register address and bit shifts/masks to use for this resource
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_getScReactionParamLoc((uint8_t)config->resource, &regAddr, &shift, &mask);
    }

    // Now read configuration register and update the config struct
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->scReaction = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

static inline int32_t PWR_convertVoltageMvToCodeBuck(uint16_t voltageMv, uint8_t *voltageCode)
{
    // Formula for code to Volt is 0.9+VSET*0.020
    //
    // NOTE: The device does not limit the BUCK_VSET value to 0x32 (which is the
    // highest specified operating range), if a higher value is written then the
    // device will be out of the specified operating range. This driver limits
    // the code value by limiting the voltage range allowed by the user.
    int32_t status = PMIC_ST_SUCCESS;

    // Assert that the voltage is within supported ranges
    if (PWR_isInRangeU16(BUCK_VSET_MIN_MV, BUCK_VSET_MAX_MV, voltageMv) == false) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Assert that the voltage is in 20mV units
    if ((voltageMv % BUCK_VSET_STEP_MV) != 0U) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // If all assertions passed, we can safely convert the voltage to VSET code
    if (status == PMIC_ST_SUCCESS) {
        *voltageCode = (uint8_t)((voltageMv - BUCK_VSET_MIN_MV) / BUCK_VSET_STEP_MV);
    }

    return status;
}

static inline int32_t PWR_convertVoltageMvToCodeVmon(uint16_t voltageMv, uint8_t *voltageCode)
{
    // Formula for code to Volt is 0.5+PG_SET*0.025
    int32_t status = PMIC_ST_SUCCESS;

    // Assert that the voltage is within supported ranges
    if (PWR_isInRangeU16(VMON_PG_LEVEL_MIN_MV, VMON_PG_LEVEL_MAX_MV, voltageMv) == false) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Assert that the voltage is in 25mV units
    if ((voltageMv % VMON_PG_LEVEL_STEP_MV) != 0U) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // If all assertions passed, we can safely convert the voltage to VSET code
    if (status == PMIC_ST_SUCCESS) {
        *voltageCode = (uint8_t)((voltageMv - VMON_PG_LEVEL_MIN_MV) / VMON_PG_LEVEL_STEP_MV);
    }

    return status;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_setVoltageCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t voltageCode = 0U;
    uint8_t mask = 0U;

    if (config->resource > PMIC_PWR_RSRC_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        regAddr = BUCK1_VOUT_REG + config->resource;
    }

    // Attempt to convert the user provided mV into the desired code, and set up
    // bit-mask to use depending on requested resource.
    switch (config->resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            mask = BUCK_VSET_MASK;

            if (status == PMIC_ST_SUCCESS) {
                status = PWR_convertVoltageMvToCodeBuck((uint16_t)config->voltage_mV, &voltageCode);
            }

            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
        case PMIC_PWR_RSRC_LS2_VMON2:
        case PMIC_PWR_RSRC_VCCA_VMON:
            mask = VMON_PGSET_MASK;

            if (status == PMIC_ST_SUCCESS) {
                status = PWR_convertVoltageMvToCodeVmon((uint16_t)config->voltage_mV, &voltageCode);
            }

            break;
        default:
            if (status == PMIC_ST_SUCCESS) {
                status = PMIC_ST_ERR_INV_PARAM;
            }

            break;
    }

    // Now read configuration register, modify control, and write back with
    // critical section
    if (status == PMIC_ST_SUCCESS) {
        status = PWR_readModifyWrite(handle, regAddr, VSET_PGSET_SHIFT, mask, voltageCode);
    }

    return status;
}

static inline int32_t PWR_convertCodeToVoltageMvBuck(uint16_t *voltageMv, uint8_t code)
{
    // Formula for code to Volt is 0.9+VSET*0.020
    //
    // NOTE: The device does not limit the BUCK_VSET value to 0x32 (which is the
    // highest specified operating range). While the "set" API of this driver
    // does limit the input values, it is possible that the register would have
    // been configured via other means. For this reason, the value in the
    // register will be converted to a voltage faithfully and without
    // restriction.
    *voltageMv = BUCK_VSET_MIN_MV + (code * BUCK_VSET_STEP_MV);
    return PMIC_ST_SUCCESS;
}

static inline int32_t PWR_convertCodeToVoltageMvVmon(uint16_t *voltageMv, uint8_t code)
{
    // Formula for code to Volt is 0.5+PG_SET*0.025
    *voltageMv = VMON_PG_LEVEL_MIN_MV + (code * VMON_PG_LEVEL_STEP_MV);
    return PMIC_ST_SUCCESS;
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested a change.
static int32_t PWR_getVoltageCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t mask = 0U;
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint16_t voltageMv = 0U;

    if (config->resource > PMIC_PWR_RSRC_MAX) {
        status = PMIC_ST_ERR_INV_PARAM;
    } else {
        regAddr = BUCK1_VOUT_REG + config->resource;
    }

    // Select the correct mask value
    switch (config->resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
            mask = BUCK_VSET_MASK;
            break;
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
        case PMIC_PWR_RSRC_LS2_VMON2:
        case PMIC_PWR_RSRC_VCCA_VMON:
            mask = VMON_PGSET_MASK;
            break;
        default:
            // Only possible values of status at this point are SUCCESS or
            // ERR_INV_PARAM, so this can be unconditional.
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    // Read from the target register
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        uint8_t voltageCode = Pmic_getBitField(regData, VSET_PGSET_SHIFT, mask);

        // Convert voltage code to mV based on resource type
        switch (config->resource) {
            case PMIC_PWR_RSRC_BUCK1:
            case PMIC_PWR_RSRC_BUCK2:
            case PMIC_PWR_RSRC_BUCK3:
                status = PWR_convertCodeToVoltageMvBuck(&voltageMv, voltageCode);
                break;
            case PMIC_PWR_RSRC_LDO_LS1_VMON1:
            case PMIC_PWR_RSRC_LS2_VMON2:
            case PMIC_PWR_RSRC_VCCA_VMON:
                status = PWR_convertCodeToVoltageMvVmon(&voltageMv, voltageCode);
                break;
            default: /* LCOV_EXCL_START */
                status = PMIC_ST_ERR_INV_PARAM;
                break; /* LCOV_EXCL_STOP */
        }
    }

    // If the conversion was successful, update the config struct
    if (status == PMIC_ST_SUCCESS) {
        config->voltage_mV = voltageMv;
    }

    return status;
}

static int32_t PWR_setSingleResourceCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config) {
    int32_t status = PMIC_ST_SUCCESS;

    // Define linkage between user provided valid params and the functions which
    // handle setting those configurations
    const SetResourceProcessor_t resourceProcessors[] = {
        { (uint16_t)PMIC_PWR_CFG_MODE_VALID, PWR_setModeCfg },
        { (uint16_t)PMIC_PWR_CFG_ILIM_VALID, PWR_setIlimCfg },
        { (uint16_t)PMIC_PWR_CFG_VOLTAGE_VALID, PWR_setVoltageCfg },
        { (uint16_t)PMIC_PWR_CFG_DEGLITCH_VALID, PWR_setDeglitchCfg },
        { (uint16_t)PMIC_PWR_CFG_UV_THRESH_VALID, PWR_setUvThreshCfg },
        { (uint16_t)PMIC_PWR_CFG_UV_REACT_VALID, PWR_setUvReactionCfg },
        { (uint16_t)PMIC_PWR_CFG_OV_THRESH_VALID, PWR_setOvThreshCfg },
        { (uint16_t)PMIC_PWR_CFG_OV_REACT_VALID, PWR_setOvReactionCfg },
        { (uint16_t)PMIC_PWR_CFG_RV_REACT_VALID, PWR_setRvReactionCfg },
        { (uint16_t)PMIC_PWR_CFG_SC_REACT_VALID, PWR_setScReactionCfg },
    };

    // First check for the enable bit, if this is valid and the user is
    // disabling the resource, set this first. If the user is enabling the power
    // resource, do that last.
    if ((Pmic_validParamStatusCheck(config->validParams, PMIC_PWR_CFG_ENABLE_VALID, status) == true) &&
        (config->enable == PMIC_DISABLE)) {
        status = Pmic_pwrSetResourceEnable(handle, (uint8_t)config->resource, (bool)config->enable);
    }

    // Handle all other potential field modifications
    for (uint8_t i = 0U; i < COUNT(resourceProcessors); i++) {
        const uint16_t validParam = resourceProcessors[i].validParam;

        if (status != PMIC_ST_SUCCESS) {
            break;
        }

        if (Pmic_validParamCheck(config->validParams, validParam)) {
            status = resourceProcessors[i].fptr(handle, config);
        }
    }

    // Final check for the enable bit, if this is valid and the user is enabling
    // the resource, we skipped configuration at the start of this function and
    // need to enable it now.
    if ((Pmic_validParamStatusCheck(config->validParams, PMIC_PWR_CFG_ENABLE_VALID, status) == true) &&
        (config->enable == PMIC_ENABLE)) {
        status = Pmic_pwrSetResourceEnable(handle, (uint8_t)config->resource, (bool)config->enable);
    }

    return status;
}

int32_t Pmic_pwrSetResourceCfg(const Pmic_Handle_t *handle, const Pmic_PowerResourceCfg_t *config)
{
    return Pmic_pwrSetResourceCfgs(handle, 1U, config);
}

int32_t Pmic_pwrSetResourceCfgs(const Pmic_Handle_t *handle, uint8_t numConfigs, const Pmic_PowerResourceCfg_t config[])
{
    Pmic_PowerResourceCfg_t localConfigs[PMIC_PWR_RSRC_MAX];
    int32_t status = Pmic_checkHandle(handle);

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && ((numConfigs == 0U) || (numConfigs > PMIC_PWR_RSRC_MAX))) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numConfigs; i++) {
            PWR_copyPowerResourceCfg(&config[i], &localConfigs[i]);
        }
    }

    for (uint8_t i = 0U; i < numConfigs; i++) {
        if (status != PMIC_ST_SUCCESS) {
            break;
        }
        status = PWR_setSingleResourceCfg(handle, &localConfigs[i]);
    }

    return Pmic_logStatus(handle, status);
}

// NOTE: This function expects that the validParam value has already been
// checked and the user requested this param be updated.
static int32_t PWR_getEnableCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    bool isEnabled = false;
    int32_t status = Pmic_pwrGetResourceEnable(handle, config->resource, &isEnabled);

    if (status == PMIC_ST_SUCCESS) {
        config->enable = isEnabled;
    }

    return status;
}

static int32_t PWR_getSingleResourceCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Define linkage between user provided valid params and the functions which
    // handle getting those configurations
    const GetResourceProcessor_t resourceProcessors[] = {
        { (uint16_t)PMIC_PWR_CFG_ENABLE_VALID, PWR_getEnableCfg },
        { (uint16_t)PMIC_PWR_CFG_MODE_VALID, PWR_getModeCfg },
        { (uint16_t)PMIC_PWR_CFG_ILIM_VALID, PWR_getIlimCfg },
        { (uint16_t)PMIC_PWR_CFG_VOLTAGE_VALID, PWR_getVoltageCfg },
        { (uint16_t)PMIC_PWR_CFG_DEGLITCH_VALID, PWR_getDeglitchCfg },
        { (uint16_t)PMIC_PWR_CFG_UV_THRESH_VALID, PWR_getUvThreshCfg },
        { (uint16_t)PMIC_PWR_CFG_UV_REACT_VALID, PWR_getUvReactionCfg },
        { (uint16_t)PMIC_PWR_CFG_OV_THRESH_VALID, PWR_getOvThreshCfg },
        { (uint16_t)PMIC_PWR_CFG_OV_REACT_VALID, PWR_getOvReactionCfg },
        { (uint16_t)PMIC_PWR_CFG_RV_REACT_VALID, PWR_getRvReactionCfg },
        { (uint16_t)PMIC_PWR_CFG_SC_REACT_VALID, PWR_getScReactionCfg },
    };

    // Read all fields requested by the user
    for (uint8_t i = 0U; i < COUNT(resourceProcessors); i++) {
        const uint16_t validParam = resourceProcessors[i].validParam;

        if (status != PMIC_ST_SUCCESS) {
            break;
        }

        if (Pmic_validParamCheck(config->validParams, validParam)) {
            status = resourceProcessors[i].fptr(handle, config);
        }
    }

    return status;
}

int32_t Pmic_pwrGetResourceCfg(const Pmic_Handle_t *handle, Pmic_PowerResourceCfg_t *config)
{
    return Pmic_pwrGetResourceCfgs(handle, 1U, config);
}

int32_t Pmic_pwrGetResourceCfgs(const Pmic_Handle_t *handle, uint8_t numConfigs, Pmic_PowerResourceCfg_t config[])
{
    Pmic_PowerResourceCfg_t localConfigs[PMIC_PWR_RSRC_MAX];
    int32_t status = Pmic_checkHandle(handle);

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && ((numConfigs == 0U) || (numConfigs > PMIC_PWR_RSRC_MAX))) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numConfigs; i++) {
            PWR_copyPowerResourceCfg(&config[i], &localConfigs[i]);
        }
    }

    for (uint8_t i = 0U; i < numConfigs; i++) {
        if (status != PMIC_ST_SUCCESS) {
            break;
        }
        status = PWR_getSingleResourceCfg(handle, &localConfigs[i]);
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numConfigs; i++) {
            PWR_copyPowerResourceCfg(&localConfigs[i], &config[i]);
        }
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrSetSequenceCfg(const Pmic_Handle_t *handle, const Pmic_PowerSequenceCfg_t *config)
{
    return Pmic_pwrSetSequenceCfgs(handle, 1U, config);
}

int32_t Pmic_pwrGetSequenceCfg(const Pmic_Handle_t *handle, Pmic_PowerSequenceCfg_t *config)
{
    return Pmic_pwrGetSequenceCfgs(handle, 1U, config);
}

static int32_t PWR_getSeqRegister(uint8_t resource, uint8_t *regAddr)
{
    int32_t status = PMIC_ST_SUCCESS;

    // Determine which register to use for this resource
    switch (resource) {
        case PMIC_PWR_RSRC_BUCK1:
        case PMIC_PWR_RSRC_BUCK2:
        case PMIC_PWR_RSRC_BUCK3:
        case PMIC_PWR_RSRC_LDO_LS1_VMON1:
        case PMIC_PWR_RSRC_LS2_VMON2:
            *regAddr = BUCK1_SEQUENCE_REG + resource;
            break;
        case PMIC_PWR_RSRC_GPO:
            *regAddr = LDO_LS1_VMON1_SEQUENCE_REG;
            break;
        case PMIC_PWR_RSRC_NRSTOUT:
            *regAddr = NRSTOUT_SEQUENCE_REG;
            break;
        default:
            status = PMIC_ST_ERR_INV_PARAM;
            break;
    }

    return status;
}

static int32_t PWR_getSingleSequence(const Pmic_Handle_t *handle, Pmic_PowerSequenceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;

    status = PWR_getSeqRegister((uint8_t)config->resource, &regAddr);

    // Read the relevant sequencing register
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    // Extract the requested fields
    if (status == PMIC_ST_SUCCESS) {
        if (Pmic_validParamCheck(config->validParams, PMIC_PWR_SEQ_STARTUP_VALID)) {
            config->startupDelay = Pmic_getBitField(regData, SEQ_STARTUP_DELAY_SHIFT, SEQ_STARTUP_DELAY_MASK);
        }

        if (Pmic_validParamCheck(config->validParams, PMIC_PWR_SEQ_SHUTDOWN_VALID)) {
            config->shutdownDelay = Pmic_getBitField(regData, SEQ_SHUTDOWN_DELAY_SHIFT, SEQ_SHUTDOWN_DELAY_MASK);
        }
    }

    return status;
}

static int32_t PWR_setSingleSequence(const Pmic_Handle_t *handle, const Pmic_PowerSequenceCfg_t *config)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regAddr = 0U;
    uint8_t regData = 0U;

    status = PWR_getSeqRegister((uint8_t)config->resource, &regAddr);

    // Read the relevant sequencing register
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    // Modify requested fields
    if (Pmic_validParamStatusCheck(config->validParams, PMIC_PWR_SEQ_STARTUP_VALID, status)) {
        if (config->startupDelay > PMIC_PWR_SEQ_DLY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, SEQ_STARTUP_DELAY_SHIFT, SEQ_STARTUP_DELAY_MASK, config->startupDelay);
        }
    }

    if (Pmic_validParamStatusCheck(config->validParams, PMIC_PWR_SEQ_SHUTDOWN_VALID, status)) {
        if (config->shutdownDelay > PMIC_PWR_SEQ_DLY_MAX) {
            status = PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, SEQ_SHUTDOWN_DELAY_SHIFT, SEQ_SHUTDOWN_DELAY_MASK, config->shutdownDelay);
        }
    }

    // Write the modifed register contents back
    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return status;
}

int32_t Pmic_pwrSetSequenceCfgs(const Pmic_Handle_t *handle, uint8_t numConfigs, const Pmic_PowerSequenceCfg_t config[])
{
    Pmic_PowerSequenceCfg_t localConfigs[8];
    int32_t status = Pmic_checkHandle(handle);

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && ((numConfigs == 0U) || (numConfigs > 8U))) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numConfigs; i++) {
            PWR_copyPowerSequenceCfg(&config[i], &localConfigs[i]);
        }
    }

    for (uint8_t i = 0U; i < numConfigs; i++) {
        if (status != PMIC_ST_SUCCESS) {
            break;
        }
        status = PWR_setSingleSequence(handle, &localConfigs[i]);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrGetSequenceCfgs(const Pmic_Handle_t *handle, uint8_t numConfigs, Pmic_PowerSequenceCfg_t config[])
{
    Pmic_PowerSequenceCfg_t localConfigs[8];
    int32_t status = Pmic_checkHandle(handle);

    // Validate parameters
    if ((status == PMIC_ST_SUCCESS) && ((numConfigs == 0U) || (numConfigs > 8U))) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numConfigs; i++) {
            PWR_copyPowerSequenceCfg(&config[i], &localConfigs[i]);
        }
    }

    for (uint8_t i = 0U; i < numConfigs; i++) {
        if (status != PMIC_ST_SUCCESS) {
            break;
        }
        status = PWR_getSingleSequence(handle, &localConfigs[i]);
    }

    if (status == PMIC_ST_SUCCESS) {
        for (uint8_t i = 0U; i < numConfigs; i++) {
            PWR_copyPowerSequenceCfg(&localConfigs[i], &config[i]);
        }
    }

    return Pmic_logStatus(handle, status);
}
