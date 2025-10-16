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
/**
 *   @file pmic_power.c
 *
 *   @brief This file contains the PMIC power module configuration API's and
 *   structures.
 */

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdint.h>

#include "pmic_power.h"
#include "regmap/power.h"

#include "pmic.h"
#include "pmic_io.h"
#include "pmic_common.h"

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

// Number of clearable status registers for the power module
#define NUM_CLEARABLE_POWER_STAT_REGS (8U)

// Used with Pmic_PwrRsrcType and Pmic_PwrRsrcId to extract values
#define PWR_RSRC_ID_MASK    ((uint16_t)0x00FFU)
#define PWR_RSRC_TYPE_MASK  ((uint16_t)0xFF00U)

#define CLEAR_ALL_STAT_BITS (0xFFU)

/* ========================================================================== */
/*                           Variables and Data                               */
/* ========================================================================== */
static const uint8_t ClearableStatRegs[NUM_CLEARABLE_POWER_STAT_REGS] = {
    DCDC_STAT_REG,
    VMON_DEV_STAT_REG,
    VMON_LDO_STAT_REG,
    VMON_PLDO_STAT_REG,
    EXT_VMON_STAT_REG,
    ILIM_STAT_REG,
    THERMAL_STAT1_REG,
    THERMAL_STAT2_REG
};

/* ========================================================================== */
/*                            Function Definitions                            */
/* ========================================================================== */

// Check if power resource is buck boost
static inline bool PWR_isBuckBoost(uint16_t pwrRsrc)
{
    return (bool)(pwrRsrc == PMIC_PWR_BUCK_BOOST);
}

// Check if power resource is LDO
static inline bool PWR_isLdo(uint16_t pwrRsrc)
{
    return (bool)((pwrRsrc >= PMIC_PWR_LDO_MIN) && (pwrRsrc <= PMIC_PWR_LDO_MAX));
}

// Check if power resource is PLDO
static inline bool PWR_isPldo(uint16_t pwrRsrc)
{
    return (bool)((pwrRsrc >= PMIC_PWR_PLDO_MIN) && (pwrRsrc <= PMIC_PWR_PLDO_MAX));
}

// Check if power resource is external VMON
static inline bool PWR_isExtVmon(uint16_t pwrRsrc)
{
    return (bool)((pwrRsrc >= PMIC_PWR_EXT_VMON_MIN) && (pwrRsrc <= PMIC_PWR_EXT_VMON_MAX));
}

// Get the resource type
static inline uint8_t PWR_getRsrcType(uint16_t pwrRsrc)
{
    return (uint8_t)((pwrRsrc & PWR_RSRC_TYPE_MASK) >> PMIC_PWR_RSRC_TYPE_SHIFT);
}

// Get the resource identifier
static inline uint8_t PWR_getRsrcId(uint16_t pwrRsrc)
{
    return (uint8_t)(pwrRsrc & PWR_RSRC_ID_MASK);
}

// Set Buck Boost spread spectrum enable, voltage level, standby voltage level, and PGOOD CFG
static int32_t PWR_setBbSsEnLvlPGoodCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read BUCK_BST_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, BUCK_BST_CFG_REG, &regData);

    // Modify BB_PGOOD_CFG
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID, status))
    {
        Pmic_setBitField_b(&regData, BB_PGOOD_CFG_SHIFT, buckBoostCfg->includeOvUvStatInPGood);
    }

    // Modify BB_SS_EN
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_SS_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, BB_SS_EN_SHIFT, buckBoostCfg->ssEn);
    }

    // Modify BB_LVL_CFG
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_LVL_VALID, status))
    {
        if (buckBoostCfg->lvl > PMIC_PWR_BB_LVL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, BB_LVL_CFG_SHIFT, BB_LVL_CFG_MASK, buckBoostCfg->lvl);
        }
    }

    // Modify BB_STBY_LVL_CFG
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_STBY_LVL_VALID, status))
    {
        if (buckBoostCfg->stbyLvl > PMIC_PWR_BB_STBY_LVL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, BB_STBY_LVL_CFG_SHIFT, BB_STBY_LVL_CFG_MASK, buckBoostCfg->stbyLvl);
        }
    }

    // Write BUCK_BST_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, BUCK_BST_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set Buck Boost VMON threshold and boost timeout
static int32_t PWR_setBbVmonThrBstTmo(Pmic_CoreHandle_t *handle, const Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read VMON_TH_CFG3
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, VMON_TH_CFG3_REG, &regData);

    // Modify BB_VMON_TH
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_VMON_THR_VALID, status))
    {
        if (buckBoostCfg->vmonThr > PMIC_PWR_BB_VMON_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, BB_VMON_TH_SHIFT, BB_VMON_TH_MASK, buckBoostCfg->vmonThr);
        }
    }

    // Modify BB_BST_TMO_CFG
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_BOOST_TMO_VALID, status))
    {
        if (buckBoostCfg->boostTmo > PMIC_PWR_BOOST_TMO_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, BB_BST_TMO_CFG_SHIFT, BB_BST_TMO_CFG_MASK, buckBoostCfg->boostTmo);
        }
    }

    // Write VMON_TH_CFG3
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, VMON_TH_CFG3_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set Buck Boost VMON deglitch
static int32_t PWR_setBbVmonDgl(Pmic_CoreHandle_t *handle, const Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (buckBoostCfg->vmonDgl > PMIC_PWR_RSRC_VMON_DGL_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        // Read VMON_DGL_CFG1
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, VMON_DGL_CFG1_REG, &regData);

        // Modify BB_VMON_DGL and write VMON_DGL_CFG1
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, BB_VMON_DGL_SHIFT, BB_VMON_DGL_MASK, buckBoostCfg->vmonDgl);
            status = Pmic_ioTxByte(handle, VMON_DGL_CFG1_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_pwrSetBuckBoostCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (buckBoostCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (buckBoostCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set spread spectrum enable, voltage level, standby voltage level, and PGOOD CFG
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_SS_EN_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_LVL_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_STBY_LVL_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)))
    {
        status = PWR_setBbSsEnLvlPGoodCfg(handle, buckBoostCfg);
    }

    // Set VMON threshold and boost timeout
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_VMON_THR_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_BOOST_TMO_VALID)))
    {
        status = PWR_setBbVmonThrBstTmo(handle, buckBoostCfg);
    }

    // Set VMON deglitch
    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_VMON_DGL_VALID, status))
    {
        status = PWR_setBbVmonDgl(handle, buckBoostCfg);
    }

    return status;
}

// Get Buck Boost spread spectrum enable, voltage level, standby voltage level, and PGOOD CFG
static int32_t PWR_getBbSsEnLvlPGoodCfg(Pmic_CoreHandle_t *handle, Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read BUCK_BST_CFG
    status = Pmic_ioRxByte_CS(handle, BUCK_BST_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BB_PGOOD_CFG
        if (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID))
        {
            buckBoostCfg->includeOvUvStatInPGood = Pmic_getBitField_b(regData, BB_PGOOD_CFG_SHIFT);
        }

        // Extract BB_SS_EN
        if (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_SS_EN_VALID))
        {
            buckBoostCfg->ssEn = Pmic_getBitField_b(regData, BB_SS_EN_SHIFT);
        }

        // Extract BB_LVL_CFG
        if (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_LVL_VALID))
        {
            buckBoostCfg->lvl = Pmic_getBitField(regData, BB_LVL_CFG_SHIFT, BB_LVL_CFG_MASK);
        }

        // Extract BB_STBY_LVL_CFG
        if (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_STBY_LVL_VALID))
        {
            buckBoostCfg->stbyLvl = Pmic_getBitField(regData, BB_STBY_LVL_CFG_SHIFT, BB_STBY_LVL_CFG_MASK);
        }
    }

    return status;
}

// Get Buck Boost VMON threshold and boost timeout
static int32_t PWR_getBbVmonThrBstTmo(Pmic_CoreHandle_t *handle, Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read VMON_TH_CFG3
    status = Pmic_ioRxByte_CS(handle, VMON_TH_CFG3_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BB_VMON_TH
        if (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_VMON_THR_VALID))
        {
            buckBoostCfg->vmonThr = Pmic_getBitField(regData, BB_VMON_TH_SHIFT, BB_VMON_TH_MASK);
        }

        // Extract BB_BST_TMO_CFG
        if (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_BOOST_TMO_VALID))
        {
            buckBoostCfg->boostTmo = Pmic_getBitField(regData, BB_BST_TMO_CFG_SHIFT, BB_BST_TMO_CFG_MASK);
        }
    }

    return status;
}

// Get Buck Boost VMON deglitch
static int32_t PWR_getBbVmonDgl(Pmic_CoreHandle_t *handle, Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read VMON_DGL_CFG1
    status = Pmic_ioRxByte_CS(handle, VMON_DGL_CFG1_REG, &regData);

    // Extract BB_VMON_DGL
    if (status == PMIC_ST_SUCCESS)
    {
        buckBoostCfg->vmonDgl = Pmic_getBitField(regData, BB_VMON_DGL_SHIFT, BB_VMON_DGL_MASK);
    }

    return status;
}

int32_t Pmic_pwrGetBuckBoostCfg(Pmic_CoreHandle_t *handle, Pmic_PwrBuckBoostCfg_t *buckBoostCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (buckBoostCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (buckBoostCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get spread spectrum enable, level, standby level, and PGOOD CFG
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_SS_EN_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_LVL_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_STBY_LVL_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)))
    {
        status = PWR_getBbSsEnLvlPGoodCfg(handle, buckBoostCfg);
    }

    // Get VMON threshold, VMON deglitch, and boost timeout
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_VMON_THR_VALID) ||
         Pmic_validParamCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_BOOST_TMO_VALID)))
    {
        status = PWR_getBbVmonThrBstTmo(handle, buckBoostCfg);
    }

    if (Pmic_validParamStatusCheck(buckBoostCfg->validParams, PMIC_PWR_CFG_BB_VMON_DGL_VALID, status))
    {
        status = PWR_getBbVmonDgl(handle, buckBoostCfg);
    }

    return status;
}

// Set LDO ramp time, voltage level, and current limit level
static int32_t PWR_setLdoRtLvlIlimLvl(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // LDOx_CFG registers are contiguous in the PMIC register map
    const uint8_t ldoCfgReg = LDO1_CFG_REG + PWR_getRsrcId(ldoCfg->ldo);

    // Read LDOx_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ldoCfgReg, &regData);

    // Modify LDOx_RT_CFG
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_RAMP_TIME_VALID, status))
    {
        if (ldoCfg->rampTime > PMIC_PWR_RT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK, ldoCfg->rampTime);
        }
    }

    // Modify LDOx_ILIM_LVL_CFG
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_ILIM_LVL_VALID, status))
    {
        if (ldoCfg->ilimLvl > PMIC_PWR_LDO_ILIM_LVL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK, ldoCfg->ilimLvl);
        }
    }

    // Modify LDOx_LVL_CFG
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_LVL_VALID, status))
    {
        if (ldoCfg->lvl > PMIC_PWR_LDO_LVL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK, ldoCfg->lvl);
        }
    }

    // Write LDOx_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, ldoCfgReg, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set LDO mode
static int32_t PWR_setLdoMode(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    if (ldoCfg->mode > PMIC_PWR_LDO_MODE_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        // Read LDO_CTRL
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, LDO_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Calculate LDOx_CTRL shift and mask
            shift = LDO1_CTRL_SHIFT + (uint8_t)(PWR_getRsrcId(ldoCfg->ldo) << 1U);
            mask = (uint8_t)(LDO1_CTRL_MASK << shift);

            // Modify LDOx_CTRL and write LDO_CTRL
            Pmic_setBitField(&regData, shift, mask, ldoCfg->mode);
            status = Pmic_ioTxByte(handle, LDO_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

// Set LDO PGOOD CFG
static int32_t PWR_setLdoPGoodCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_PGOOD_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, LDO_PGOOD_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_PGOOD_CFG shift
        shift = LDO1_PGOOD_CFG_SHIFT + PWR_getRsrcId(ldoCfg->ldo);

        // Modify LDOx_PGOOD_CFG and write LDO_PGOOD_CFG
        Pmic_setBitField_b(&regData, shift, ldoCfg->includeOvUvStatInPGood);
        status = Pmic_ioTxByte(handle, LDO_PGOOD_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set LDO VMON threshold
static int32_t PWR_setLdoVmonThr(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    if (ldoCfg->vmonThr > PMIC_PWR_LDO_VMON_THR_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        // Read VMON_TH_CFG1
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, VMON_TH_CFG1_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Calculate LDOx_VMON_TH shift and mask
            shift = LDO1_VMON_TH_SHIFT + (uint8_t)(PWR_getRsrcId(ldoCfg->ldo) << 1U);
            mask = (uint8_t)(LDO1_VMON_TH_MASK << shift);

            // Modify LDOx_VMON_TH and write VMON_TH_CFG1
            Pmic_setBitField(&regData, shift, mask, ldoCfg->vmonThr);
            status = Pmic_ioTxByte(handle, VMON_TH_CFG1_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

// Set LDO VMON deglitch
static int32_t PWR_setLdoVmonDgl(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    if (ldoCfg->vmonDgl > PMIC_PWR_RSRC_VMON_DGL_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        // Read VMON_DGL_CFG2
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, VMON_DGL_CFG2_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Calculate LDOx_VMON_DGL shift and mask
            shift = LDO1_VMON_DGL_SHIFT + (uint8_t)(PWR_getRsrcId(ldoCfg->ldo) << 1U);
            mask = (uint8_t)(LDO1_VMON_DGL_MASK << shift);

            // Modify LDOx_VMON_DGL and write VMON_DGL_CFG2
            Pmic_setBitField(&regData, shift, mask, ldoCfg->vmonDgl);
            status = Pmic_ioTxByte(handle, VMON_DGL_CFG2_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

// Set LDO discharge
static int32_t PWR_setLdoDischargeDisable(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_DSCG_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, LDO_DSCG_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_DSCG_DIS shift
        shift = LDO1_DSCG_DIS_SHIFT + PWR_getRsrcId(ldoCfg->ldo);

        // Modify LDOx_DSCG_DIS and write LDO_DSCG_CFG
        Pmic_setBitField_b(&regData, shift, ldoCfg->disableDischarge);
        status = Pmic_ioTxByte(handle, LDO_DSCG_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set LDO ILIM deglitch
static int32_t PWR_setLdoIlimDgl(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    if (ldoCfg->ilimDgl > PMIC_PWR_LDO_ILIM_DEGLITCH_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }
    else
    {
        // Read ILIM_DGL_CFG
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, ILIM_DGL_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Calculate LDOx_ILIM_DGL_CFG shift and mask
            shift = LDO1_ILIM_DGL_CFG_SHIFT + PWR_getRsrcId(ldoCfg->ldo);
            mask = (uint8_t)(LDO1_ILIM_DGL_CFG_MASK << shift);

            // Modify LDOx_ILIM_DGL_CFG and write ILIM_DGL_CFG
            Pmic_setBitField(&regData, shift, mask, ldoCfg->ilimDgl);
            status = Pmic_ioTxByte(handle, ILIM_DGL_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_pwrSetLdoCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (ldoCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && !PWR_isLdo(ldoCfg->ldo))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set LDO ramp time, voltage level, and current limit level
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_RAMP_TIME_VALID) ||
         Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_LVL_VALID) ||
         Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_ILIM_LVL_VALID)))
    {
        status = PWR_setLdoRtLvlIlimLvl(handle, ldoCfg);
    }

    // Set LDO mode
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_MODE_VALID, status))
    {
        status = PWR_setLdoMode(handle, ldoCfg);
    }

    // Set LDO PGOOD CFG
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID, status))
    {
        status = PWR_setLdoPGoodCfg(handle, ldoCfg);
    }

    // Set LDO VMON threshold
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_VMON_THR_VALID, status))
    {
        status = PWR_setLdoVmonThr(handle, ldoCfg);
    }

    // Set LDO VMON deglitch
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_VMON_DGL_VALID, status))
    {
        status = PWR_setLdoVmonDgl(handle, ldoCfg);
    }

    // Set LDO discharge
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID, status))
    {
        status = PWR_setLdoDischargeDisable(handle, ldoCfg);
    }

    // Set LDO current limit deglitch
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_ILIM_DGL_VALID, status))
    {
        status = PWR_setLdoIlimDgl(handle, ldoCfg);
    }

    return status;
}

// Get LDO ramp time, voltage level, and current limit level
static int32_t PWR_getLdoRtLvlIlimLvl(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // LDOx_CFG registers are contiguous in the PMIC register map
    const uint8_t reg = LDO1_CFG_REG + PWR_getRsrcId(ldoCfg->ldo);

    // Read LDOx_CFG
    status = Pmic_ioRxByte_CS(handle, reg, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract LDOx_RT_CFG
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_RAMP_TIME_VALID))
        {
            ldoCfg->rampTime = Pmic_getBitField(regData, LDO_RT_CFG_SHIFT, LDO_RT_CFG_MASK);
        }

        // Extract LDOx_ILIM_LVL_CFG
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_ILIM_LVL_VALID))
        {
            ldoCfg->ilimLvl = Pmic_getBitField(regData, LDO_ILIM_LVL_CFG_SHIFT, LDO_ILIM_LVL_CFG_MASK);
        }

        // Extract LDOx_LVL_CFG
        if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_LVL_VALID))
        {
            ldoCfg->lvl = Pmic_getBitField(regData, LDO_LVL_CFG_SHIFT, LDO_LVL_CFG_MASK);
        }
    }

    return status;
}

// Get LDO mode
static int32_t PWR_getLdoMode(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read LDO_CTRL
    status = Pmic_ioRxByte_CS(handle, LDO_CTRL_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_CTRL shift and mask
        shift = LDO1_CTRL_SHIFT + (uint8_t)(PWR_getRsrcId(ldoCfg->ldo) << 1U);
        mask = (uint8_t)(LDO1_CTRL_MASK << shift);

        // Extract LDOx_CTRL
        ldoCfg->mode = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

// Get LDO PGOOD CFG
static int32_t PWR_getLdoPGoodCfg(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_PGOOD_CFG
    status = Pmic_ioRxByte_CS(handle, LDO_PGOOD_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_PGOOD_CFG shift
        shift = LDO1_PGOOD_CFG_SHIFT + PWR_getRsrcId(ldoCfg->ldo);

        // Extract LDOx_PGOOD_CFG
        ldoCfg->includeOvUvStatInPGood = Pmic_getBitField_b(regData, shift);
    }

    return status;
}

// Get LDO VMON threshold
static int32_t PWR_getLdoVmonThr(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read VMON_TH_CFG1
    status = Pmic_ioRxByte_CS(handle, VMON_TH_CFG1_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_VMON_TH shift and mask
        shift = LDO1_VMON_TH_SHIFT + (uint8_t)(PWR_getRsrcId(ldoCfg->ldo) << 1U);
        mask = (uint8_t)(LDO1_VMON_TH_MASK << shift);

        // Extract LDOx_VMON_TH
        ldoCfg->vmonThr = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

// Get LDO VMON deglitch
static int32_t PWR_getLdoVmonDgl(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read VMON_DGL_CFG2
    status = Pmic_ioRxByte_CS(handle, VMON_DGL_CFG2_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_VMON_DGL shift and mask
        shift = LDO1_VMON_DGL_SHIFT + (uint8_t)(PWR_getRsrcId(ldoCfg->ldo) << 1U);
        mask = (uint8_t)(LDO1_VMON_DGL_MASK << shift);

        // Extract LDOx_VMON_DGL
        ldoCfg->vmonDgl = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

// Get LDO discharge
static int32_t PWR_getLdoDischargeDisable(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_DSCG_CFG
    status = Pmic_ioRxByte_CS(handle, LDO_DSCG_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_DSCG_DIS shift and mask
        shift = LDO1_DSCG_DIS_SHIFT + PWR_getRsrcId(ldoCfg->ldo);

        // Extract LDOx_DSCG_DIS
        ldoCfg->disableDischarge = Pmic_getBitField_b(regData, shift);
    }

    return status;
}

// Get LDO ILIM deglitch
static int32_t PWR_getLdoIlimDgl(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read ILIM_DGL_CFG
    status = Pmic_ioRxByte_CS(handle, ILIM_DGL_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Calculate LDOx_ILIM_DGL_CFG shift and mask
        shift = LDO1_ILIM_DGL_CFG_SHIFT + PWR_getRsrcId(ldoCfg->ldo);
        mask = (uint8_t)(LDO1_ILIM_DGL_CFG_MASK << shift);

        // Extract LDOx_ILIM_DGL_CFG
        ldoCfg->ilimDgl = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

int32_t Pmic_pwrGetLdoCfg(Pmic_CoreHandle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (ldoCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && !PWR_isLdo(ldoCfg->ldo))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get LDO ramp time, voltage level, and current limit level
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_RAMP_TIME_VALID) ||
         Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_LVL_VALID) ||
         Pmic_validParamCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_ILIM_LVL_VALID)))
    {
        status = PWR_getLdoRtLvlIlimLvl(handle, ldoCfg);
    }

    // Get LDO mode
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_MODE_VALID, status))
    {
        status = PWR_getLdoMode(handle, ldoCfg);
    }

    // Get LDO PGOOD CFG
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID, status))
    {
        status = PWR_getLdoPGoodCfg(handle, ldoCfg);
    }

    // Get LDO VMON threshold
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_VMON_THR_VALID, status))
    {
        status = PWR_getLdoVmonThr(handle, ldoCfg);
    }

    // Get VMON deglitch
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_VMON_DGL_VALID, status))
    {
        status = PWR_getLdoVmonDgl(handle, ldoCfg);
    }

    // Get LDO discharge
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_DISABLE_DISCHARGE_VALID, status))
    {
        status = PWR_getLdoDischargeDisable(handle, ldoCfg);
    }

    // Get current limit deglitch
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_PWR_CFG_LDO_ILIM_DGL_VALID, status))
    {
        status = PWR_getLdoIlimDgl(handle, ldoCfg);
    }

    return status;
}

// Set PLDO tracking mode configuration, voltage level, and ILIM level
static int32_t PWR_setPldoTrackingModeLvlIlimLvl(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t pldoCfgReg = (pldoCfg->pldo == PMIC_PWR_PLDO1) ? PLDO1_CFG_REG : PLDO2_CFG_REG;

    // Read PLDOx_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, pldoCfgReg, &regData);

    // Modify PLDOx_MODE
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID, status))
    {
        Pmic_setBitField_b(&regData, PLDO_MODE_SHIFT, pldoCfg->trackingMode);
    }

    // Modify PLDOx_ILIM_LVL_CFG
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID, status))
    {
        if (pldoCfg->ilimLvl > PMIC_PWR_PLDO_ILIM_LVL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PLDO_ILIM_LVL_CFG_SHIFT, PLDO_ILIM_LVL_CFG_MASK, pldoCfg->ilimLvl);
        }
    }

    // Modify PLDOx_LVL_CFG
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_LVL_VALID, status))
    {
        if (pldoCfg->lvl > PMIC_PWR_PLDO_LVL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, PLDO_LVL_CFG_SHIFT, PLDO_LVL_CFG_MASK, pldoCfg->lvl);
        }
    }

    // Write PLDOx_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, pldoCfgReg, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set PLDO ramp time and VTRACK range
static int32_t PWR_setPldoRtVTrackRange(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read PLDO_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, PLDO_CFG_REG, &regData);

    // Modify VTRACK_RNG
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID, status))
    {
        if (pldoCfg->vtrackRange > PMIC_PWR_VTRACK_RANGE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, VTRACK_RNG_SHIFT, VTRACK_RNG_MASK, pldoCfg->vtrackRange);
        }
    }

    // Modify PLDOx_RT_CFG
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_RT_VALID, status))
    {
        if (pldoCfg->rampTime > PMIC_PWR_RT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_RT_CFG_SHIFT;
            mask = PLDO1_RT_CFG_MASK;
        }
        else
        {
            shift = PLDO2_RT_CFG_SHIFT;
            mask = PLDO2_RT_CFG_MASK;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, pldoCfg->rampTime);
        }
    }

    // Write PLDO_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PLDO_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set PLDO PGOOD CFG
static int32_t PWR_setPldoPGoodCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_PGOOD_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, LDO_PGOOD_CFG_REG, &regData);

    // Modify PLDOx_PGOOD_CFG and write LDO_PGOOD_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        shift = (pldoCfg->pldo == PMIC_PWR_PLDO1) ? PLDO1_PGOOD_CFG_SHIFT : PLDO2_PGOOD_CFG_SHIFT;
        Pmic_setBitField_b(&regData, shift, pldoCfg->includeOvUvStatInPGood);

        status = Pmic_ioTxByte(handle, LDO_PGOOD_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set PLDO mode
static int32_t PWR_setPldoMode(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;
    const bool invalidPldo1Mode = (bool)(
        (pldoCfg->pldo == PMIC_PWR_PLDO1) &&
        (pldoCfg->mode > PMIC_PWR_PLDO1_MODE_MAX));

    const bool invalidPldo2Mode = (bool)(
        (pldoCfg->pldo == PMIC_PWR_PLDO2) &&
        (pldoCfg->mode > PMIC_PWR_PLDO2_MODE_MAX));

    // Read PLDO_EN_OUT_CTRL
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, PLDO_EN_OUT_CTRL_REG, &regData);

    // Modify PLDOx_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        if (invalidPldo1Mode || invalidPldo2Mode)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_CTRL_SHIFT;
            mask = PLDO1_CTRL_MASK;
        }
        else
        {
            shift = PLDO2_CTRL_SHIFT;
            mask = PLDO2_CTRL_MASK;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, pldoCfg->mode);
        }
    }

    // Write PLDO_EN_OUT_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, PLDO_EN_OUT_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// set PLDO discharge
static int32_t PWR_setPldoDischarge(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_DSCG_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, LDO_DSCG_CFG_REG, &regData);

    // Modify PLDOx_DSCG_DIS and write LDO_DSCG_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        shift = (pldoCfg->pldo == PMIC_PWR_PLDO1) ? PLDO1_DSCG_DIS_SHIFT : PLDO2_DSCG_DIS_SHIFT;
        Pmic_setBitField_b(&regData, shift, pldoCfg->disableDischarge);

        status = Pmic_ioTxByte(handle, LDO_DSCG_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set PLDO VMON threshold
static int32_t PWR_setPldoVmonThr(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read VMON_TH_CFG2
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, VMON_TH_CFG2_REG, &regData);

    // Modify PLDOx_VMON_TH
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->vmonThr > PMIC_PWR_PLDO_VMON_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_VMON_TH_SHIFT;
            mask = PLDO1_VMON_TH_MASK;
        }
        else
        {
            shift = PLDO2_VMON_TH_SHIFT;
            mask = PLDO2_VMON_TH_MASK;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, pldoCfg->vmonThr);
        }
    }

    // Write VMON_TH_CFG2
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, VMON_TH_CFG2_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set PLDO VMON deglitch
static int32_t PWR_setPldoVmonDgl(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read VMON_DGL_CFG3
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, VMON_DGL_CFG3_REG, &regData);

    // Modify PLDOx_VMON_DGL
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->vmonDgl > PMIC_PWR_RSRC_VMON_DGL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_VMON_DGL_SHIFT;
            mask = PLDO1_VMON_DGL_MASK;
        }
        else
        {
            shift = PLDO2_VMON_DGL_SHIFT;
            mask = PLDO2_VMON_DGL_MASK;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, pldoCfg->vmonDgl);
        }
    }

    // Write VMON_DGL_CFG3
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, VMON_DGL_CFG3_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set PLDO ILIM deglitch
static int32_t PWR_setPldoIlimDgl(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read ILIM_DGL_CFG
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ILIM_DGL_CFG_REG, &regData);

    // Modify PLDOx_ILIM_DGL_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->ilimDgl > PMIC_PWR_LDO_ILIM_DEGLITCH_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
           shift = PLDO1_ILIM_DGL_CFG_SHIFT;
           mask = PLDO1_ILIM_DGL_CFG_MASK;
        }
        else
        {
            shift = PLDO2_ILIM_DGL_CFG_SHIFT;
            mask = PLDO2_ILIM_DGL_CFG_MASK;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, pldoCfg->ilimDgl);
        }
    }

    // Write ILIM_DGL_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, ILIM_DGL_CFG_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_pwrSetPldoCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (pldoCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && !PWR_isPldo(pldoCfg->pldo))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set PLDO tracking mode configuration, voltage level, and ILIM level
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID) ||
         Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_LVL_VALID) ||
         Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID)))
    {
        status = PWR_setPldoTrackingModeLvlIlimLvl(handle, pldoCfg);
    }

    // Set PLDO ramp time and VTRACK range
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_RT_VALID) ||
         Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID)))
    {
        status = PWR_setPldoRtVTrackRange(handle, pldoCfg);
    }

    // Set PLDO PGOOD_CFG
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID, status))
    {
        status = PWR_setPldoPGoodCfg(handle, pldoCfg);
    }

    // Set PLDO mode
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_MODE_VALID, status))
    {
        status = PWR_setPldoMode(handle, pldoCfg);
    }

    // Set PLDO discharge
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID, status))
    {
        status = PWR_setPldoDischarge(handle, pldoCfg);
    }

    // Set PLDO VMON threshold
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VMON_THR_VALID, status))
    {
        status = PWR_setPldoVmonThr(handle, pldoCfg);
    }

    // Set PLDO VMON deglitch
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VMON_DGL_VALID, status))
    {
        status = PWR_setPldoVmonDgl(handle, pldoCfg);
    }

    // Set PLDO ILIM deglitch
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID, status))
    {
        status = PWR_setPldoIlimDgl(handle, pldoCfg);
    }

    return status;
}

// Get PLDO tracking mode configuration, voltage level, and ILIM level
static int32_t PWR_getPldoTrackingModeLvlIlimLvl(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const uint8_t pldoCfgReg = (pldoCfg->pldo == PMIC_PWR_PLDO1) ? PLDO1_CFG_REG : PLDO2_CFG_REG;

    // Read PLDOx_CFG
    status = Pmic_ioRxByte_CS(handle, pldoCfgReg, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract PLDOx_MODE
        if (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID))
        {
            pldoCfg->trackingMode = Pmic_getBitField_b(regData, PLDO_MODE_SHIFT);
        }

        // Extract PLDOx_ILIM_LVL_CFG
        if (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID))
        {
            pldoCfg->ilimLvl = Pmic_getBitField(regData, PLDO_ILIM_LVL_CFG_SHIFT, PLDO_ILIM_LVL_CFG_MASK);
        }

        // Extract PLDOx_LVL_CFG
        if (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_LVL_VALID))
        {
            pldoCfg->lvl = Pmic_getBitField(regData, PLDO_LVL_CFG_SHIFT, PLDO_LVL_CFG_MASK);
        }
    }

    return status;
}

// Get PLDO ramp time and VTRACK range
static int32_t PWR_getPldoRtVTrackRange(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read PLDO_CFG
    status = Pmic_ioRxByte_CS(handle, PLDO_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract VTRACK_RNG
        if (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID))
        {
            pldoCfg->vtrackRange = Pmic_getBitField(regData, VTRACK_RNG_SHIFT, VTRACK_RNG_MASK);
        }

        // Extract PLDOx_RT_CFG
        if (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_RT_VALID))
        {
            if (pldoCfg->pldo == PMIC_PWR_PLDO1)
            {
                shift = PLDO1_RT_CFG_SHIFT;
                mask = PLDO1_RT_CFG_MASK;
            }
            else
            {
                shift = PLDO2_RT_CFG_SHIFT;
                mask = PLDO2_RT_CFG_MASK;
            }
            pldoCfg->rampTime = Pmic_getBitField(regData, shift, mask);
        }
    }

    return status;
}

// Get PLDO PGOOD CFG
static int32_t PWR_getPldoPGoodCfg(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_PGOOD_CFG
    status = Pmic_ioRxByte_CS(handle, LDO_PGOOD_CFG_REG, &regData);

    // Extract PLDOx_PGOOD_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        shift = (pldoCfg->pldo == PMIC_PWR_PLDO1) ? PLDO1_PGOOD_CFG_SHIFT : PLDO2_PGOOD_CFG_SHIFT;
        pldoCfg->includeOvUvStatInPGood = Pmic_getBitField_b(regData, shift);
    }

    return status;
}

// Get PLDO mode
static int32_t PWR_getPldoMode(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read PLDO_EN_OUT_CTRL
    status = Pmic_ioRxByte_CS(handle, PLDO_EN_OUT_CTRL_REG, &regData);

    // Extract PLDOx_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_CTRL_SHIFT;
            mask = PLDO1_CTRL_MASK;
        }
        else
        {
            shift = PLDO2_CTRL_SHIFT;
            mask = PLDO2_CTRL_MASK;
        }
        pldoCfg->mode = Pmic_getBitField(regData, shift, mask);

        // Convert redundant value into value seen by end-users
        if (pldoCfg->mode > PMIC_PWR_PLDO2_MODE_MAX)
        {
            pldoCfg->mode = PMIC_PWR_PLDO_DISABLED;
        }
    }

    return status;
}

// Get PLDO discharge
static int32_t PWR_getPldoDischarge(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;

    // Read LDO_DSCG_CFG
    status = Pmic_ioRxByte_CS(handle, LDO_DSCG_CFG_REG, &regData);

    // Extract PLDOx_DSCG_DIS
    if (status == PMIC_ST_SUCCESS)
    {
        shift = (pldoCfg->pldo == PMIC_PWR_PLDO1) ? PLDO1_DSCG_DIS_SHIFT : PLDO2_DSCG_DIS_SHIFT;
        pldoCfg->disableDischarge = Pmic_getBitField_b(regData, shift);
    }

    return status;
}

// Get PLDO VMON threshold
static int32_t PWR_getPldoVmonThr(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read VMON_TH_CFG2
    status = Pmic_ioRxByte_CS(handle, VMON_TH_CFG2_REG, &regData);

    // Extract PLDOx_VMON_TH
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_VMON_TH_SHIFT;
            mask = PLDO1_VMON_TH_MASK;
        }
        else
        {
            shift = PLDO2_VMON_TH_SHIFT;
            mask = PLDO2_VMON_TH_MASK;
        }
        pldoCfg->vmonThr = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

// Get PLDO VMON deglitch
static int32_t PWR_getPldoVmonDgl(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read VMON_DGL_CFG3
    status = Pmic_ioRxByte_CS(handle, VMON_DGL_CFG3_REG, &regData);

    // Extract PLDOx_VMON_DGL
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_VMON_DGL_SHIFT;
            mask = PLDO1_VMON_DGL_MASK;
        }
        else
        {
            shift = PLDO2_VMON_DGL_SHIFT;
            mask = PLDO2_VMON_DGL_MASK;
        }
        pldoCfg->vmonDgl = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

// Get PLDO ILIM deglitch
static int32_t PWR_getPldoIlimDgl(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read ILIM_DGL_CFG
    status = Pmic_ioRxByte_CS(handle, ILIM_DGL_CFG_REG, &regData);

    // Extract PLDOx_ILIM_DGL_CFG
    if (status == PMIC_ST_SUCCESS)
    {
        if (pldoCfg->pldo == PMIC_PWR_PLDO1)
        {
            shift = PLDO1_ILIM_DGL_CFG_SHIFT;
            mask = PLDO1_ILIM_DGL_CFG_MASK;
        }
        else
        {
            shift = PLDO2_ILIM_DGL_CFG_SHIFT;
            mask = PLDO2_ILIM_DGL_CFG_MASK;
        }
        pldoCfg->ilimDgl = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

int32_t Pmic_pwrGetPldoCfg(Pmic_CoreHandle_t *handle, Pmic_PwrPldoCfg_t *pldoCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (pldoCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && !PWR_isPldo(pldoCfg->pldo))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get PLDO tracking mode configuration, voltage level, and ILIM level
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_TRACKING_MODE_VALID) ||
         Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_LVL_VALID) ||
         Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_ILIM_LVL_VALID)))
    {
        status = PWR_getPldoTrackingModeLvlIlimLvl(handle, pldoCfg);
    }

    // Get PLDO ramp time and VTRACK range
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_RT_VALID) ||
         Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VTRACK_RANGE_VALID)))
    {
        status = PWR_getPldoRtVTrackRange(handle, pldoCfg);
    }

    // Get PLDO PGOOD_CFG
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID, status))
    {
        status = PWR_getPldoPGoodCfg(handle, pldoCfg);
    }

    // Get PLDO mode
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_MODE_VALID, status))
    {
        status = PWR_getPldoMode(handle, pldoCfg);
    }

    // Get PLDO discharge
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_DISABLE_DISCHARGE_VALID, status))
    {
        status = PWR_getPldoDischarge(handle, pldoCfg);
    }

    // Get PLDO VMON threshold
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VMON_THR_VALID, status))
    {
        status = PWR_getPldoVmonThr(handle, pldoCfg);
    }

    // Get PLDO VMON deglitch
    if (Pmic_validParamStatusCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_VMON_DGL_VALID, status))
    {
        status = PWR_getPldoVmonDgl(handle, pldoCfg);
    }

    // Get PLDO ILIM deglitch
    if ((status == PMIC_ST_SUCCESS) && Pmic_validParamCheck(pldoCfg->validParams, PMIC_PWR_CFG_PLDO_ILIM_DGL_VALID))
    {
        status = PWR_getPldoIlimDgl(handle, pldoCfg);
    }

    return status;
}

// Set external VMON mode and PGOOD CFG
static int32_t PWR_setExtVmonModePGoodCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read EXT_VMON_CFG_CTRL
    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, EXT_VMON_CFG_CTRL_REG, &regData);

    // Modify EXT_VMONx_CTRL
    if (Pmic_validParamStatusCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_MODE_VALID, status))
    {
        if (extVmonCfg->mode > PMIC_PWR_EXT_VMON_MODE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }

        if (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1)
        {
            shift = EXT_VMON1_CTRL_SHIFT;
            mask = EXT_VMON1_CTRL_MASK;
        }
        else
        {
            shift = EXT_VMON2_CTRL_SHIFT;
            mask = EXT_VMON2_CTRL_MASK;
        }

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, extVmonCfg->mode);
        }
    }

    // Modify EXT_VMONx_PGOOD_CFG
    if (Pmic_validParamStatusCheck(extVmonCfg->validParams,
                                   PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID,
                                   status))
    {
        shift = (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1) ? EXT_VMON1_PGOOD_CFG_SHIFT : EXT_VMON2_PGOOD_CFG_SHIFT;
        Pmic_setBitField_b(&regData, shift, extVmonCfg->includeOvUvStatInPGood);
    }

    // Write EXT_VMON_CFG_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, EXT_VMON_CFG_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

// Set external VMON threshold
static int32_t PWR_setExtVmonThr(Pmic_CoreHandle_t *handle, const Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, vmonThCfgReg = 0U, shift = 0U, mask = 0U;

    if (extVmonCfg->vmonThr > PMIC_PWR_EXT_VMON_THR_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get register address, shift, and mask
    if (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1)
    {
        vmonThCfgReg = VMON_TH_CFG2_REG;
        shift = EXT_VMON1_TH_SHIFT;
        mask = EXT_VMON1_TH_MASK;
    }
    else
    {
        vmonThCfgReg = VMON_TH_CFG3_REG;
        shift = EXT_VMON2_TH_SHIFT;
        mask = EXT_VMON2_TH_MASK;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read VMON_TH_CFGx register
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, vmonThCfgReg, &regData);

        // Modify EXT_VMONx_TH and write VMON_TH_CFGx register
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, extVmonCfg->vmonThr);
            status = Pmic_ioTxByte(handle, vmonThCfgReg, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

// Set external VMON deglitch
static int32_t PWR_setExtVmonDgl(Pmic_CoreHandle_t *handle, const Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    if (extVmonCfg->vmonDgl > PMIC_PWR_RSRC_VMON_DGL_MAX)
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get shift and mask
    if (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1)
    {
        shift = EXT_VMON1_DGL_SHIFT;
        mask = EXT_VMON1_DGL_MASK;
    }
    else
    {
        shift = EXT_VMON2_DGL_SHIFT;
        mask = EXT_VMON2_DGL_MASK;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read VMON_DGL_CFG3
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioRxByte(handle, VMON_DGL_CFG3_REG, &regData);

        // Modify EXT_VMONx_DGL and write VMON_DGL_CFG3
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, shift, mask, extVmonCfg->vmonDgl);
            status = Pmic_ioTxByte(handle, VMON_DGL_CFG3_REG, regData);
        }
        Pmic_criticalSectionStop(handle);
    }

    return status;
}

int32_t Pmic_pwrSetExtVmonCfg(Pmic_CoreHandle_t *handle, const Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (extVmonCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && !PWR_isExtVmon(extVmonCfg->extVmon))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Set external VMON mode and PGOOD CFG
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_MODE_VALID) ||
         Pmic_validParamCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)))
    {
        status = PWR_setExtVmonModePGoodCfg(handle, extVmonCfg);
    }

    // Set external VMON threshold
    if (Pmic_validParamStatusCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_THR_VALID, status))
    {
        status = PWR_setExtVmonThr(handle, extVmonCfg);
    }

    // Set external VMON deglitch
    if (Pmic_validParamStatusCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_DGL_VALID, status))
    {
        status = PWR_setExtVmonDgl(handle, extVmonCfg);
    }

    return status;
}

// Get external VMON mode and PGOOD CFG
static int32_t PWR_getExtVmonModePGoodCfg(Pmic_CoreHandle_t *handle, Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Read EXT_VMON_CFG_CTRL
    status = Pmic_ioRxByte_CS(handle, EXT_VMON_CFG_CTRL_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract EXT_VMONx_CTRL
        if (Pmic_validParamCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_MODE_VALID))
        {
            if (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1)
            {
                shift = EXT_VMON1_CTRL_SHIFT;
                mask = EXT_VMON1_CTRL_MASK;
            }
            else
            {
                shift = EXT_VMON2_CTRL_SHIFT;
                mask = EXT_VMON2_CTRL_MASK;
            }
            extVmonCfg->mode = Pmic_getBitField(regData, shift, mask);
        }

        // Extract EXT_VMONx_PGOOD_CFG
        if (Pmic_validParamCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID))
        {
            shift = (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1) ?
                EXT_VMON1_PGOOD_CFG_SHIFT:
                EXT_VMON2_PGOOD_CFG_SHIFT;
            extVmonCfg->includeOvUvStatInPGood = Pmic_getBitField_b(regData, shift);
        }
    }

    return status;
}

// Get external VMON threshold
static int32_t PWR_getExtVmonThr(Pmic_CoreHandle_t *handle, Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, vmonThrCfgReg = 0U, shift = 0U, mask = 0U;

    // Get register address, shift, and mask
    if (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1)
    {
        vmonThrCfgReg = VMON_TH_CFG2_REG;
        shift = EXT_VMON1_TH_SHIFT;
        mask = EXT_VMON1_TH_MASK;
    }
    else
    {
        vmonThrCfgReg = VMON_TH_CFG3_REG;
        shift = EXT_VMON2_TH_SHIFT;
        mask = EXT_VMON2_TH_MASK;
    }

    // Read VMON_TH_CFGx register
    status = Pmic_ioRxByte_CS(handle, vmonThrCfgReg, &regData);

    // Extract EXT_VMONx_TH
    if (status == PMIC_ST_SUCCESS)
    {
        extVmonCfg->vmonThr = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

// Get external VMON deglitch
static int32_t PWR_getExtVmonDgl(Pmic_CoreHandle_t *handle, Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U, mask = 0U;

    // Get shift and mask
    if (extVmonCfg->extVmon == PMIC_PWR_EXT_VMON1)
    {
        shift = EXT_VMON1_DGL_SHIFT;
        mask = EXT_VMON1_DGL_MASK;
    }
    else
    {
        shift = EXT_VMON2_DGL_SHIFT;
        mask = EXT_VMON2_DGL_MASK;
    }

    // Read VMON_DGL_CFG3
    status = Pmic_ioRxByte_CS(handle, VMON_DGL_CFG3_REG, &regData);

    // Extract EXT_VMONx_DGL
    if (status == PMIC_ST_SUCCESS)
    {
        extVmonCfg->vmonDgl = Pmic_getBitField(regData, shift, mask);
    }

    return status;
}

int32_t Pmic_pwrGetExtVmonCfg(Pmic_CoreHandle_t *handle, Pmic_PwrExtVmonCfg_t *extVmonCfg)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (extVmonCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && !PWR_isExtVmon(extVmonCfg->extVmon))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Get external VMON control and PGOOD CFG
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_MODE_VALID) ||
         Pmic_validParamCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_INCLUDE_OV_UV_STAT_IN_PGOOD_VALID)))
    {
        status = PWR_getExtVmonModePGoodCfg(handle, extVmonCfg);
    }

    // Get external VMON threshold and deglitch
    if (Pmic_validParamStatusCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_THR_VALID, status))
    {
        status = PWR_getExtVmonThr(handle, extVmonCfg);
    }

    if (Pmic_validParamStatusCheck(extVmonCfg->validParams, PMIC_PWR_CFG_EXT_VMON_DGL_VALID, status))
    {
        status = PWR_getExtVmonDgl(handle, extVmonCfg);
    }

    return status;
}

static int32_t PWR_getBbStat(Pmic_CoreHandle_t *handle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (!PWR_isBuckBoost(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read DCDC_STAT
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, DCDC_STAT_REG, &regData);
    }

    // Get BB lite, BB ILIM level, BB mode, and OV/UV error statuses
    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BB_LITE
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID))
        {
            pwrRsrcStat->bbLite = Pmic_getBitField_b(regData, BB_LITE_SHIFT);
        }

        // Extract BB_ILIM_LVL
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID))
        {
            pwrRsrcStat->bbIlimLvl = Pmic_getBitField_b(regData, BB_ILIM_LVL_SHIFT);
        }

        // Extract BB_BST
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID))
        {
            pwrRsrcStat->bbMode = Pmic_getBitField_b(regData, BB_BST_SHIFT);
        }

        // Extract BB_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            pwrRsrcStat->ovErr = Pmic_getBitField_b(regData, BB_OV_ERR_SHIFT);
        }

        // Extract BB_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            pwrRsrcStat->uvErr = Pmic_getBitField_b(regData, BB_UV_ERR_SHIFT);
        }
    }

    if (Pmic_validParamStatusCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID, status))
    {
        // read ILIM_STAT
        status = Pmic_ioRxByte_CS(handle, ILIM_STAT_REG, &regData);

        // Extract BB_AVG_ILIM_ERR
        if (status == PMIC_ST_SUCCESS)
        {
            pwrRsrcStat->ilimErr = Pmic_getBitField_b(regData, BB_AVG_ILIM_ERR_SHIFT);
        }
    }

    // Read THERMAL_STAT2
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, THERMAL_STAT2_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract BB_TSD_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID))
        {
            pwrRsrcStat->tsdErr = Pmic_getBitField_b(regData, BB_TSD_ERR_SHIFT);
        }

        // Extract BB_T_PRE_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID))
        {
            pwrRsrcStat->tsdWarn = Pmic_getBitField_b(regData, BB_T_PRE_ERR_SHIFT);
        }
    }

    return status;
}

static int32_t PWR_getLdoStat(Pmic_CoreHandle_t *handle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;
    const bool unsupportedParams = (bool)(
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID));

    if (!PWR_isLdo(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && unsupportedParams)
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read VMON_LDO_STAT
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, VMON_LDO_STAT_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract LDOx_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            shift = LDO1_UV_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            pwrRsrcStat->uvErr = Pmic_getBitField_b(regData, shift);
        }

        // Extract LDOx_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            shift = LDO1_OV_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            pwrRsrcStat->ovErr = Pmic_getBitField_b(regData, shift);
        }
    }

    if (Pmic_validParamStatusCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID, status))
    {
        // Read ILIM_STAT
        status = Pmic_ioRxByte_CS(handle, ILIM_STAT_REG, &regData);

        // Extract LDOx_ILIM_ERR
        if (status == PMIC_ST_SUCCESS)
        {
            shift = LDO1_ILIM_ERR_SHIFT + PWR_getRsrcId(pwrRsrcStat->pwrRsrc);
            pwrRsrcStat->ilimErr = Pmic_getBitField_b(regData, shift);
        }
    }

    // Read THERMAL_STAT1
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, THERMAL_STAT1_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract LDOx_TSD_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID))
        {
            shift = LDO1_TSD_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            pwrRsrcStat->tsdErr = Pmic_getBitField_b(regData, shift);
        }

        // Extract LDOx_T_PRE_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID))
        {
            shift = LDO1_T_PRE_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            pwrRsrcStat->tsdWarn = Pmic_getBitField_b(regData, shift);
        }
    }

    return status;
}

static int32_t PWR_getPldoStat(Pmic_CoreHandle_t *handle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;
    const bool unsupportedParams = (bool)(
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID));

    if (!PWR_isPldo(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && unsupportedParams)
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read VMON_PLDO_STAT
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, VMON_PLDO_STAT_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract PLDOx_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_OV_ERR_SHIFT : PLDO2_OV_ERR_SHIFT;
            pwrRsrcStat->ovErr = Pmic_getBitField_b(regData, shift);
        }

        // Extract PLDOx_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_UV_ERR_SHIFT : PLDO2_UV_ERR_SHIFT;
            pwrRsrcStat->uvErr = Pmic_getBitField_b(regData, shift);
        }
    }

    if (Pmic_validParamStatusCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID, status))
    {
        // Read ILIM_STAT
        status = Pmic_ioRxByte_CS(handle, ILIM_STAT_REG, &regData);

        // Extract PLDOx_ILIM_ERR
        if (status == PMIC_ST_SUCCESS)
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_ILIM_ERR_SHIFT : PLDO2_ILIM_ERR_SHIFT;
            pwrRsrcStat->ilimErr = Pmic_getBitField_b(regData, shift);
        }
    }

    // Read THERMAL_STAT2
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, THERMAL_STAT2_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract PLDOx_TSD_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_TSD_ERR_SHIFT : PLDO2_TSD_ERR_SHIFT;
            pwrRsrcStat->tsdErr = Pmic_getBitField_b(regData, shift);
        }

        // Extract PLDOx_T_PRE_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_T_PRE_ERR_SHIFT : PLDO2_T_PRE_ERR_SHIFT;
            pwrRsrcStat->tsdWarn = Pmic_getBitField_b(regData, shift);
        }
    }

    return status;
}

static int32_t PWR_getExtVmonStat(Pmic_CoreHandle_t *handle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;
    const bool unsupportedParams = (bool)(
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID_SHIFT) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID_SHIFT));

    if (!PWR_isExtVmon(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && unsupportedParams)
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    // Read EXT_VMON_STAT
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID) ||
         Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, EXT_VMON_STAT_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract EXT_VMON1_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_EXT_VMON1) ? EXT_VMON1_UV_ERR_SHIFT : EXT_VMON2_UV_ERR_SHIFT;
            pwrRsrcStat->uvErr = Pmic_getBitField_b(regData, shift);
        }

        // Extract EXT_VMON1_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_EXT_VMON1) ? EXT_VMON1_OV_ERR_SHIFT : EXT_VMON2_OV_ERR_SHIFT;
            pwrRsrcStat->ovErr = Pmic_getBitField_b(regData, shift);
        }
    }

    return status;
}

int32_t Pmic_pwrGetRsrcStat(Pmic_CoreHandle_t *handle, Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (pwrRsrcStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        switch (PWR_getRsrcType(pwrRsrcStat->pwrRsrc))
        {
            case PMIC_PWR_TYPE_BUCK_BOOST:
            {
                status = PWR_getBbStat(handle, pwrRsrcStat);
                break;
            }
            case PMIC_PWR_TYPE_LDO:
            {
                status = PWR_getLdoStat(handle, pwrRsrcStat);
                break;
            }
            case PMIC_PWR_TYPE_PLDO:
            {
                status = PWR_getPldoStat(handle, pwrRsrcStat);
                break;
            }
            case PMIC_PWR_TYPE_EXT_VMON:
            {
                status = PWR_getExtVmonStat(handle, pwrRsrcStat);
                break;
            }
            default:
            {
                status = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    return status;
}

static int32_t PWR_clrBbStat(Pmic_CoreHandle_t *handle, const Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (!PWR_isBuckBoost(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear BB_BST
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID))
        {
            Pmic_setBitField_b(&regData, BB_BST_SHIFT, PMIC_CLEAR_STAT);
        }

        // Clear BB_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, BB_OV_ERR_SHIFT, PMIC_CLEAR_STAT);
        }

        // Clear BB_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, BB_UV_ERR_SHIFT, PMIC_CLEAR_STAT);
        }

        // Write DCDC_STAT
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, DCDC_STAT_REG, regData);
            regData = 0U;
        }
    }

    if (Pmic_validParamStatusCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID, status))
    {
        // Clear BB_AVG_ILIM_ERR
        Pmic_setBitField_b(&regData, BB_AVG_ILIM_ERR_SHIFT, PMIC_CLEAR_STAT);

        // Write ILIM_STAT
        status = Pmic_ioTxByte_CS(handle, ILIM_STAT_REG, regData);
        regData = 0U;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear BB_TSD_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID))
        {
            Pmic_setBitField_b(&regData, BB_TSD_ERR_SHIFT, PMIC_CLEAR_STAT);
        }

        // Clear BB_T_PRE_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID))
        {
            Pmic_setBitField_b(&regData, BB_T_PRE_ERR_SHIFT, PMIC_CLEAR_STAT);
        }

        // Write THERMAL_STAT2
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, THERMAL_STAT2_REG, regData);
        }
    }

    return status;
}

static int32_t PWR_clrLdoStat(Pmic_CoreHandle_t *handle, const Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;
    const bool unsupportedParams = (bool)(
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID));

    if (!PWR_isLdo(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && unsupportedParams)
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear LDOx_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            shift = LDO1_UV_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Clear LDOx_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            shift = LDO1_OV_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Write VMON_LDO_STAT
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, VMON_LDO_STAT_REG, regData);
            regData = 0U;
        }
    }

    if (Pmic_validParamStatusCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID, status))
    {
        // Clear LDOx_ILIM_ERR
        shift = LDO1_ILIM_ERR_SHIFT + PWR_getRsrcId(pwrRsrcStat->pwrRsrc);
        Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);

        // Write ILIM_STAT
        status = Pmic_ioTxByte_CS(handle, ILIM_STAT_REG, regData);
        regData = 0U;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear LDOx_TSD_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID))
        {
            shift = LDO1_TSD_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Clear LDOx_T_PRE_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID))
        {
            shift = LDO1_T_PRE_ERR_SHIFT + (uint8_t)(PWR_getRsrcId(pwrRsrcStat->pwrRsrc) << 1U);
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Write THERMAL_STAT1
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, THERMAL_STAT1_REG, regData);
        }
    }

    return status;
}

static int32_t PWR_clrPldoStat(Pmic_CoreHandle_t *handle, const Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;
    const bool unsupportedParams = (bool)(
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID));

    if (!PWR_isPldo(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && unsupportedParams)
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear PLDOx_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_OV_ERR_SHIFT : PLDO2_OV_ERR_SHIFT;
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Clear PLDOx_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_UV_ERR_SHIFT : PLDO2_UV_ERR_SHIFT;
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Write VMON_PLDO_STAT
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, VMON_PLDO_STAT_REG, regData);
            regData = 0U;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear PLDOx_TSD_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_TSD_ERR_SHIFT : PLDO2_TSD_ERR_SHIFT;
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Clear PLDOx_T_PRE_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_PLDO1) ? PLDO1_T_PRE_ERR_SHIFT : PLDO2_T_PRE_ERR_SHIFT;
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Write THERMAL_STAT2
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, THERMAL_STAT2_REG, regData);
        }
    }

    return status;
}

static int32_t PWR_clrExtVmonStat(Pmic_CoreHandle_t *handle, const Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U, shift = 0U;
    const bool unsupportedParams = (bool)(
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_ILIM_ERR_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_LITE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_ILIM_LVL_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_BB_MODE_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_ERR_VALID) ||
        Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_TSD_WARN_VALID));

    if (!PWR_isExtVmon(pwrRsrcStat->pwrRsrc))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && unsupportedParams)
    {
        status = PMIC_ST_ERR_NOT_SUPPORTED;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Clear EXT_VMON1_UV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_UV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_EXT_VMON1) ? EXT_VMON1_UV_ERR_SHIFT : EXT_VMON2_UV_ERR_SHIFT;
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Clear EXT_VMON1_OV_ERR
        if (Pmic_validParamCheck(pwrRsrcStat->validParams, PMIC_PWR_RSRC_STAT_OV_ERR_VALID))
        {
            shift = (pwrRsrcStat->pwrRsrc == PMIC_PWR_EXT_VMON1) ? EXT_VMON1_OV_ERR_SHIFT : EXT_VMON2_OV_ERR_SHIFT;
            Pmic_setBitField_b(&regData, shift, PMIC_CLEAR_STAT);
        }

        // Write EXT_VMON_STAT
        if (regData != 0U)
        {
            status = Pmic_ioTxByte_CS(handle, EXT_VMON_STAT_REG, regData);
        }
    }

    return status;
}

int32_t Pmic_pwrClrRsrcStat(Pmic_CoreHandle_t *handle, const Pmic_PwrRsrcStat_t *pwrRsrcStat)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    if ((status == PMIC_ST_SUCCESS) && (pwrRsrcStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        switch (PWR_getRsrcType(pwrRsrcStat->pwrRsrc))
        {
            case PMIC_PWR_TYPE_BUCK_BOOST:
            {
                status = PWR_clrBbStat(handle, pwrRsrcStat);
                break;
            }
            case PMIC_PWR_TYPE_LDO:
            {
                status = PWR_clrLdoStat(handle, pwrRsrcStat);
                break;
            }
            case PMIC_PWR_TYPE_PLDO:
            {
                status = PWR_clrPldoStat(handle, pwrRsrcStat);
                break;
            }
            case PMIC_PWR_TYPE_EXT_VMON:
            {
                status = PWR_clrExtVmonStat(handle, pwrRsrcStat);
                break;
            }
            default:
            {
                status = PMIC_ST_ERR_INV_PARAM;
                break;
            }
        }
    }

    return status;
}

int32_t Pmic_pwrClrRsrcStatAll(Pmic_CoreHandle_t *handle)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    for (uint8_t i = 0U; i < NUM_CLEARABLE_POWER_STAT_REGS; i++)
    {
        if (status != PMIC_ST_SUCCESS)
        {
            break;
        }

        // Statuses are write 1 to clear (W1C)
        status = Pmic_ioTxByte_CS(handle, ClearableStatRegs[i], CLEAR_ALL_STAT_BITS);
    }

    return status;
}

int32_t Pmic_pwrSetPGoodInStby(Pmic_CoreHandle_t *handle, bool enable)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read PLDO_EN_OUT_CTRL
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PLDO_EN_OUT_CTRL_REG, &regData);
    }

    // Modify PGOOD_CTRL and write PLDO_EN_OUT_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField_b(&regData, PGOOD_CTRL_SHIFT, enable);
        status = Pmic_ioTxByte(handle, PLDO_EN_OUT_CTRL_REG, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_pwrGetPGoodInStby(Pmic_CoreHandle_t *handle, bool *isEnabled)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);
    uint8_t regData = 0U;

    // Read PLDO_EN_OUT_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PLDO_EN_OUT_CTRL_REG, &regData);
    }

    // Extract PGOOD_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        *isEnabled = Pmic_getBitField_b(regData, PGOOD_CTRL_SHIFT);
    }

    return status;
}
