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
#include "regmap/core.h"
#include "regmap/irq.h"

#include "pmic.h"
#include "pmic_io.h"
#include "pmic_common.h"

#include <string.h>

/* ========================================================================== */
/*                             Macros & Typedefs                              */
/* ========================================================================== */

// Used to extract resource ID from resource value
#define PWR_RSRC_ID_MASK    ((uint16_t)0x00FFU)

/* ========================================================================== */
/*                            Function Definitions                            */
/* ========================================================================== */

/**
 * @brief Copy Pmic_PwrBuckCfg_t structure member-wise
 */
static inline void PWR_copyPwrBuckCfg(const Pmic_PwrBuckCfg_t *src, Pmic_PwrBuckCfg_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_PwrBuckCfg_t));
}

/**
 * @brief Copy Pmic_PwrLdoCfg_t structure member-wise
 */
static inline void PWR_copyPwrLdoCfg(const Pmic_PwrLdoCfg_t *src, Pmic_PwrLdoCfg_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_PwrLdoCfg_t));
}

/**
 * @brief Copy Pmic_PwrVccaVmonCfg_t structure member-wise
 */
static inline void PWR_copyPwrVccaVmonCfg(const Pmic_PwrVccaVmonCfg_t *src, Pmic_PwrVccaVmonCfg_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_PwrVccaVmonCfg_t));
}

/**
 * @brief Copy Pmic_PwrThermalCfg_t structure member-wise
 */
static inline void PWR_copyPwrThermalCfg(const Pmic_PwrThermalCfg_t *src, Pmic_PwrThermalCfg_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_PwrThermalCfg_t));
}

/**
 * @brief Copy Pmic_PwrSpreadSpectrumCfg_t structure member-wise
 */
static inline void PWR_copyPwrSpreadSpectrumCfg(const Pmic_PwrSpreadSpectrumCfg_t *src, Pmic_PwrSpreadSpectrumCfg_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_PwrSpreadSpectrumCfg_t));
}

/**
 * @brief Copy Pmic_PwrRsrcStatus_t structure member-wise
 */
static inline void PWR_copyPwrRsrcStatus(const Pmic_PwrRsrcStatus_t *src, Pmic_PwrRsrcStatus_t *dst)
{
    memmove((void *)dst, (const void *)src, sizeof(Pmic_PwrRsrcStatus_t));
}

// Get the resource ID from resource value
static inline void PWR_getRsrcId(uint16_t pwrRsrc, uint8_t *rsrcId)
{
    *rsrcId = (uint8_t)(pwrRsrc & PWR_RSRC_ID_MASK);
}

// Check if power resource is a buck
static inline bool PWR_isBuck(uint16_t pwrRsrc)
{
    return (bool)((pwrRsrc >= PMIC_POWER_RESOURCE_BUCK_MIN) &&
                  (pwrRsrc <= PMIC_POWER_RESOURCE_BUCK_MAX));
}

// Check if power resource is an LDO
static inline bool PWR_isLdo(uint16_t pwrRsrc)
{
    return (bool)((pwrRsrc >= PMIC_POWER_RESOURCE_LDO_MIN) &&
                  (pwrRsrc <= PMIC_POWER_RESOURCE_LDO_MAX));
}

// Check if power resource is a VMON
static inline bool PWR_isVmon(uint16_t pwrRsrc)
{
    return (bool)((pwrRsrc >= PMIC_POWER_RESOURCE_VMON_MIN) &&
                  (pwrRsrc <= PMIC_POWER_RESOURCE_VMON_MAX));
}

// Get buck control register address
static inline void PWR_getBuckCtrlRegAddr(uint8_t buckId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(BUCK1_CTRL_REG + (2U * (buckId - 1U)));
}

// Get buck config register address
static inline void PWR_getBuckConfRegAddr(uint8_t buckId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(BUCK1_CONF_REG + (2U * (buckId - 1U)));
}

// Get buck VOUT register address
static inline void PWR_getBuckVoutRegAddr(uint8_t buckId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(BUCK1_VOUT_REG + (2U * (buckId - 1U)));
}

// Get buck PG window register address
static inline void PWR_getBuckPgWindowRegAddr(uint8_t buckId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(BUCK1_PG_WINDOW_REG + (buckId - 1U));
}

// Get LDO control register address
static inline void PWR_getLdoCtrlRegAddr(uint8_t ldoId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(LDO1_CTRL_REG + (ldoId - 1U));
}

// Get LDO VOUT register address
static inline void PWR_getLdoVoutRegAddr(uint8_t ldoId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(LDO1_VOUT_REG + (ldoId - 1U));
}

// Get LDO PG window register address
static inline void PWR_getLdoPgWindowRegAddr(uint8_t ldoId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(LDO1_PG_WINDOW_REG + (ldoId - 1U));
}

// Get VMON PG level register address (for VMON1 and VMON2)
static inline void PWR_getVmonPgLevelRegAddr(uint8_t vmonId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(VMON1_PG_LEVEL_REG + (2U * (vmonId - 1U)));
}

// Get VMON PG window register address
static inline void PWR_getVmonPgWindowRegAddr(uint8_t vmonId, uint16_t *regAddr)
{
    *regAddr = (uint16_t)(VMON1_PG_WINDOW_REG + (2U * (vmonId - 1U)));
}

// Get buck group select register and shift based on buck ID
static inline void PWR_getBuckGrpSelInfo(uint8_t buckId, uint16_t *regAddr, uint8_t *shift)
{
    *regAddr = RAIL_SEL_1_REG;
    *shift = (uint8_t)((buckId - 1U) * 2U);
}

// Get LDO group select register and shift based on LDO ID
static inline void PWR_getLdoGrpSelInfo(uint8_t ldoId, uint16_t *regAddr, uint8_t *shift)
{
    *regAddr = RAIL_SEL_2_REG;
    *shift = (uint8_t)((ldoId - 1U) * 2U);
}

// Get VMON group select register and shift based on VMON ID
static inline void PWR_getVmonGrpSelInfo(uint8_t vmonId, uint16_t *regAddr, uint8_t *shift)
{
    *regAddr = RAIL_SEL_3_REG;
    *shift = (uint8_t)((vmonId - 1U) * 2U);
}

// Set buck configuration
static int32_t PWR_setBuckCfg(const Pmic_Handle_t *handle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t buckId = 0U;
    PWR_getRsrcId((uint16_t)((uint16_t)buckCfg->resource), &buckId);

    // Set BUCK control register fields (EN, PLDN, VMON_EN, FPWM)
    if ((Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_EN_VALID)) ||
        (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_PLDN_EN_VALID)) ||
        (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_VMON_EN_VALID)) ||
        (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_FPWM_EN_VALID)))
    {
        uint16_t ctrlRegAddr = 0U;
        PWR_getBuckCtrlRegAddr(buckId, &ctrlRegAddr);
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ctrlRegAddr, &regData);

        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, BUCK_EN_SHIFT, BUCK_EN_MASK, buckCfg->buckEn);
        }

        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_PLDN_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, BUCK_PLDN_SHIFT, BUCK_PLDN_MASK, buckCfg->pldnEn);
        }

        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_VMON_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, BUCK_VMON_EN_SHIFT, BUCK_VMON_EN_MASK, buckCfg->vmonEn);
        }

        if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_FPWM_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, BUCK_FPWM_SHIFT, BUCK_FPWM_MASK, buckCfg->fpwmEn);
        }

        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, ctrlRegAddr, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Set buck slew rate
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_SLEW_RATE_VALID, status))
    {
        if (buckCfg->slewRate > PMIC_POWER_BUCK_SLEW_RATE_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            uint16_t confRegAddr = 0U;
            PWR_getBuckConfRegAddr(buckId, &confRegAddr);
            Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
            status = Pmic_ioRxByte(handle, confRegAddr, &regData);

            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField(&regData, BUCK_SLEW_RATE_SHIFT, BUCK_SLEW_RATE_MASK, buckCfg->slewRate);
                status = Pmic_ioTxByte(handle, confRegAddr, regData);
            }
            Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
        }
    }

    // Set buck voltage
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_VSET_VALID, status))
    {
        uint8_t vsetMin, vsetMax;
        if (buckId == PMIC_POWER_RESOURCE_ID_BUCK1)
        {
            vsetMin = PMIC_POWER_BUCK1_VSET_MIN;
            vsetMax = PMIC_POWER_BUCK1_VSET_MAX;
        }
        else
        {
            vsetMin = PMIC_POWER_BUCK2_3_4_VSET_MIN;
            vsetMax = PMIC_POWER_BUCK2_3_4_VSET_MAX;
        }

        if ((buckCfg->vset < vsetMin) || (buckCfg->vset > vsetMax))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            uint16_t voutRegAddr = 0U;
            PWR_getBuckVoutRegAddr(buckId, &voutRegAddr);
            status = Pmic_ioTxByte_CS(handle, voutRegAddr, buckCfg->vset);
        }
    }

    // Set buck VMON threshold
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_VMON_THR_VALID, status))
    {
        if (buckCfg->vmonThr > PMIC_POWER_VMON_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            uint16_t pgWindowRegAddr = 0U;
            PWR_getBuckPgWindowRegAddr(buckId, &pgWindowRegAddr);
            Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
            status = Pmic_ioRxByte(handle, pgWindowRegAddr, &regData);

            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField(&regData, BUCK_VMON_THR_SHIFT, BUCK_VMON_THR_MASK, buckCfg->vmonThr);
                status = Pmic_ioTxByte(handle, pgWindowRegAddr, regData);
            }
            Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
        }
    }

    // Set buck group select
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_GRP_SEL_VALID, status))
    {
        uint16_t grpSelRegAddr;
        uint8_t grpSelShift;

        if (buckCfg->grpSel > PMIC_POWER_GRP_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            PWR_getBuckGrpSelInfo(buckId, &grpSelRegAddr, &grpSelShift);

            Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
            status = Pmic_ioRxByte(handle, grpSelRegAddr, &regData);

            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField(&regData, grpSelShift, (uint8_t)(0x03U << grpSelShift), buckCfg->grpSel);
                status = Pmic_ioTxByte(handle, grpSelRegAddr, regData);
            }
            Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
        }
    }

    return status;
}

// Get buck configuration
static int32_t PWR_getBuckCfg(const Pmic_Handle_t *handle, Pmic_PwrBuckCfg_t *buckCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t buckId = 0U;
    PWR_getRsrcId((uint16_t)((uint16_t)buckCfg->resource), &buckId);

    // Get BUCK control register fields
    if ((Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_EN_VALID)) ||
        (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_PLDN_EN_VALID)) ||
        (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_VMON_EN_VALID)) ||
        (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_FPWM_EN_VALID)))
    {
        uint16_t ctrlRegAddr = 0U;
        PWR_getBuckCtrlRegAddr(buckId, &ctrlRegAddr);
        status = Pmic_ioRxByte_CS(handle, ctrlRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_EN_VALID))
            {
                buckCfg->buckEn = Pmic_getBitField_b(regData, BUCK_EN_SHIFT);
            }

            if (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_PLDN_EN_VALID))
            {
                buckCfg->pldnEn = Pmic_getBitField_b(regData, BUCK_PLDN_SHIFT);
            }

            if (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_VMON_EN_VALID))
            {
                buckCfg->vmonEn = Pmic_getBitField_b(regData, BUCK_VMON_EN_SHIFT);
            }

            if (Pmic_validParamCheck(buckCfg->validParams, PMIC_POWER_BUCK_FPWM_EN_VALID))
            {
                buckCfg->fpwmEn = Pmic_getBitField_b(regData, BUCK_FPWM_SHIFT);
            }
        }
    }

    // Get buck slew rate
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_SLEW_RATE_VALID, status))
    {
        uint16_t confRegAddr = 0U;
        PWR_getBuckConfRegAddr(buckId, &confRegAddr);
        status = Pmic_ioRxByte_CS(handle, confRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->slewRate = Pmic_getBitField(regData, BUCK_SLEW_RATE_SHIFT, BUCK_SLEW_RATE_MASK);
        }
    }

    // Get buck voltage
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_VSET_VALID, status))
    {
        uint16_t voutRegAddr = 0U;
        PWR_getBuckVoutRegAddr(buckId, &voutRegAddr);
        status = Pmic_ioRxByte_CS(handle, voutRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->vset = regData;
        }
    }

    // Get buck VMON threshold
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_VMON_THR_VALID, status))
    {
        uint16_t pgWindowRegAddr = 0U;
        PWR_getBuckPgWindowRegAddr(buckId, &pgWindowRegAddr);
        status = Pmic_ioRxByte_CS(handle, pgWindowRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->vmonThr = Pmic_getBitField(regData, BUCK_VMON_THR_SHIFT, BUCK_VMON_THR_MASK);
        }
    }

    // Get buck group select
    if (Pmic_validParamStatusCheck(buckCfg->validParams, PMIC_POWER_BUCK_GRP_SEL_VALID, status))
    {
        uint16_t grpSelRegAddr;
        uint8_t grpSelShift;

        PWR_getBuckGrpSelInfo(buckId, &grpSelRegAddr, &grpSelShift);

        status = Pmic_ioRxByte_CS(handle, grpSelRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            buckCfg->grpSel = Pmic_getBitField(regData, grpSelShift, (uint8_t)(0x03U << grpSelShift));
        }
    }

    return status;
}

// Set LDO configuration
static int32_t PWR_setLdoCfg(const Pmic_Handle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t ldoId = 0U;
    PWR_getRsrcId((uint16_t)((uint16_t)ldoCfg->resource), &ldoId);

    // Set LDO control register fields (EN, VMON_EN, DISCHARGE_EN)
    if ((Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_EN_VALID)) ||
        (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_VMON_EN_VALID)) ||
        (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_DISCHARGE_EN_VALID)))
    {
        uint16_t ctrlRegAddr = 0U;
        PWR_getLdoCtrlRegAddr(ldoId, &ctrlRegAddr);
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, ctrlRegAddr, &regData);

        if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, LDO_EN_SHIFT, LDO_EN_MASK, ldoCfg->ldoEn);
        }

        if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_VMON_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, LDO_VMON_EN_SHIFT, LDO_VMON_EN_MASK, ldoCfg->vmonEn);
        }

        if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_DISCHARGE_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, LDO_DISCHARGE_EN_SHIFT, LDO_DISCHARGE_EN_MASK, ldoCfg->dischargeEn);
        }

        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, ctrlRegAddr, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Set LDO VSET and bypass enable
    if ((Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_VSET_VALID, status)) ||
        (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_BYP_EN_VALID, status)))
    {
        uint16_t voutRegAddr = 0U;
        PWR_getLdoVoutRegAddr(ldoId, &voutRegAddr);
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, voutRegAddr, &regData);

        if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_BYP_EN_VALID, status))
        {
            Pmic_setBitField_b(&regData, LDO_BYP_CONFIG_SHIFT, LDO_BYP_CONFIG_MASK, ldoCfg->bypEn);
        }

        if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_VSET_VALID, status))
        {
            uint8_t vsetMin, vsetMax;
            if (ldoId == PMIC_POWER_RESOURCE_ID_LDO1)
            {
                vsetMin = PMIC_POWER_LDO1_VSET_MIN;
                vsetMax = PMIC_POWER_LDO1_VSET_MAX;
            }
            else
            {
                vsetMin = PMIC_POWER_LDO2_3_VSET_MIN;
                vsetMax = PMIC_POWER_LDO2_3_VSET_MAX;
            }

            if ((ldoCfg->vset < vsetMin) || (ldoCfg->vset > vsetMax))
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                Pmic_setBitField(&regData, LDO_VSET_SHIFT, LDO_VSET_MASK, ldoCfg->vset);
            }
        }

        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, voutRegAddr, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Set LDO VMON threshold
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_VMON_THR_VALID, status))
    {
        if (ldoCfg->vmonThr > PMIC_POWER_VMON_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            uint16_t pgWindowRegAddr = 0U;
            PWR_getLdoPgWindowRegAddr(ldoId, &pgWindowRegAddr);
            Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
            status = Pmic_ioRxByte(handle, pgWindowRegAddr, &regData);

            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField(&regData, LDO_VMON_THR_SHIFT, LDO_VMON_THR_MASK, ldoCfg->vmonThr);
                status = Pmic_ioTxByte(handle, pgWindowRegAddr, regData);
            }
            Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
        }
    }

    // Set LDO group select
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_GRP_SEL_VALID, status))
    {
        uint16_t grpSelRegAddr;
        uint8_t grpSelShift;

        if (ldoCfg->grpSel > PMIC_POWER_GRP_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            PWR_getLdoGrpSelInfo(ldoId, &grpSelRegAddr, &grpSelShift);

            Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
            status = Pmic_ioRxByte(handle, grpSelRegAddr, &regData);

            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField(&regData, grpSelShift, (uint8_t)(0x03U << grpSelShift), ldoCfg->grpSel);
                status = Pmic_ioTxByte(handle, grpSelRegAddr, regData);
            }
            Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
        }
    }

    return status;
}

// Get LDO configuration
static int32_t PWR_getLdoCfg(const Pmic_Handle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    uint8_t ldoId = 0U;
    PWR_getRsrcId((uint16_t)((uint16_t)ldoCfg->resource), &ldoId);

    // Get LDO control register fields
    if ((Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_EN_VALID)) ||
        (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_VMON_EN_VALID)) ||
        (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_DISCHARGE_EN_VALID)))
    {
        uint16_t ctrlRegAddr = 0U;
        PWR_getLdoCtrlRegAddr(ldoId, &ctrlRegAddr);
        status = Pmic_ioRxByte_CS(handle, ctrlRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_EN_VALID))
            {
                ldoCfg->ldoEn = Pmic_getBitField_b(regData, LDO_EN_SHIFT);
            }

            if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_VMON_EN_VALID))
            {
                ldoCfg->vmonEn = Pmic_getBitField_b(regData, LDO_VMON_EN_SHIFT);
            }

            if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_DISCHARGE_EN_VALID))
            {
                ldoCfg->dischargeEn = Pmic_getBitField_b(regData, LDO_DISCHARGE_EN_SHIFT);
            }
        }
    }

    // Get LDO VSET and bypass enable
    if ((Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_VSET_VALID, status)) ||
        (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_BYP_EN_VALID, status)))
    {
        uint16_t voutRegAddr = 0U;
        PWR_getLdoVoutRegAddr(ldoId, &voutRegAddr);
        status = Pmic_ioRxByte_CS(handle, voutRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_BYP_EN_VALID))
            {
                ldoCfg->bypEn = Pmic_getBitField_b(regData, LDO_BYP_CONFIG_SHIFT);
            }

            if (Pmic_validParamCheck(ldoCfg->validParams, PMIC_POWER_LDO_VSET_VALID))
            {
                ldoCfg->vset = Pmic_getBitField(regData, LDO_VSET_SHIFT, LDO_VSET_MASK);
            }
        }
    }

    // Get LDO VMON threshold
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_VMON_THR_VALID, status))
    {
        uint16_t pgWindowRegAddr = 0U;
        PWR_getLdoPgWindowRegAddr(ldoId, &pgWindowRegAddr);
        status = Pmic_ioRxByte_CS(handle, pgWindowRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            ldoCfg->vmonThr = Pmic_getBitField(regData, LDO_VMON_THR_SHIFT, LDO_VMON_THR_MASK);
        }
    }

    // Get LDO group select
    if (Pmic_validParamStatusCheck(ldoCfg->validParams, PMIC_POWER_LDO_GRP_SEL_VALID, status))
    {
        uint16_t grpSelRegAddr;
        uint8_t grpSelShift;

        PWR_getLdoGrpSelInfo(ldoId, &grpSelRegAddr, &grpSelShift);

        status = Pmic_ioRxByte_CS(handle, grpSelRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            ldoCfg->grpSel = Pmic_getBitField(regData, grpSelShift, (uint8_t)(0x03U << grpSelShift));
        }
    }

    return status;
}

// Set VCCA_VMON/VMONx configuration
static int32_t PWR_setVccaVmonCfg(const Pmic_Handle_t *handle, const Pmic_PwrVccaVmonCfg_t *vccaVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const bool isVcca = (vccaVmonCfg->resource == PMIC_POWER_RESOURCE_VCCA_VMON);

    // Set VMON enable
    if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_EN_VALID, status))
    {
        uint8_t vmonId = 0U;
        uint8_t vmonEnShift;
        PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);

        if (isVcca)
        {
            vmonEnShift = VCCA_VMON_EN_SHIFT;
        }
        else if (vmonId == PMIC_POWER_RESOURCE_ID_VMON1)
        {
            vmonEnShift = VMON1_EN_SHIFT;
        }
        else
        {
            vmonEnShift = VMON2_EN_SHIFT;
        }

        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, VCCA_VMON_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField_b(&regData, vmonEnShift, (uint8_t)(1U << vmonEnShift), vccaVmonCfg->vmonEn);
            status = Pmic_ioTxByte(handle, VCCA_VMON_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Set PG set and threshold
    if ((Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_PG_SET_VALID, status)) ||
        (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_THR_VALID, status)))
    {
        uint16_t pgWindowRegAddr = 0U;
        if (isVcca)
        {
            pgWindowRegAddr = VCCA_PG_WINDOW_REG;
        }
        else
        {
            uint8_t vmonId = 0U;
            PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);
            PWR_getVmonPgWindowRegAddr(vmonId, &pgWindowRegAddr);
        }

        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, pgWindowRegAddr, &regData);

        if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_THR_VALID, status))
        {
            uint8_t thrMax = isVcca ? PMIC_POWER_VCCA_VMON_THR_MAX : PMIC_POWER_VMON_THR_MAX;

            if (vccaVmonCfg->vmonThr > thrMax)
            {
                status = PMIC_ST_ERR_INV_PARAM;
            }
            else
            {
                uint8_t thrShift = isVcca ? VCCA_VMON_THR_SHIFT : VMON_THR_SHIFT;
                uint8_t thrMask = isVcca ? VCCA_VMON_THR_MASK : VMON_THR_MASK;
                Pmic_setBitField(&regData, thrShift, thrMask, vccaVmonCfg->vmonThr);
            }
        }

        if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_PG_SET_VALID, status))
        {
            if (isVcca)
            {
                if (vccaVmonCfg->pgSet > PMIC_POWER_VCCA_VMON_PG_SET_MAX)
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
                else
                {
                    Pmic_setBitField(&regData, VCCA_PG_SET_SHIFT, VCCA_PG_SET_MASK, vccaVmonCfg->pgSet);
                }
            }
            else
            {
                // For VMON1/VMON2, write to separate PG_LEVEL register
                uint8_t vmonId = 0U;
                uint16_t pgLevelRegAddr = 0U;
                uint8_t pgSetMin, pgSetMax;
                PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);
                PWR_getVmonPgLevelRegAddr(vmonId, &pgLevelRegAddr);

                if (vmonId == PMIC_POWER_RESOURCE_ID_VMON1)
                {
                    pgSetMin = PMIC_POWER_VMON1_PG_SET_MIN;
                    pgSetMax = PMIC_POWER_VMON1_PG_SET_MAX;
                }
                else
                {
                    pgSetMin = PMIC_POWER_VMON2_PG_SET_MIN;
                    pgSetMax = PMIC_POWER_VMON2_PG_SET_MAX;
                }

                if ((vccaVmonCfg->pgSet < pgSetMin) || (vccaVmonCfg->pgSet > pgSetMax))
                {
                    status = PMIC_ST_ERR_INV_PARAM;
                }
                else
                {
                    status = Pmic_ioTxByte(handle, pgLevelRegAddr, vccaVmonCfg->pgSet);
                }
            }
        }

        if ((status == PMIC_ST_SUCCESS) && (Pmic_validParamCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_THR_VALID)))
        {
            status = Pmic_ioTxByte(handle, pgWindowRegAddr, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Set group select
    if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_GRP_SEL_VALID, status))
    {
        uint16_t grpSelRegAddr;
        uint8_t grpSelShift;

        if (vccaVmonCfg->grpSel > PMIC_POWER_GRP_SEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            uint8_t vmonId = 0U;
            PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);
            PWR_getVmonGrpSelInfo(vmonId, &grpSelRegAddr, &grpSelShift);

            Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
            status = Pmic_ioRxByte(handle, grpSelRegAddr, &regData);

            if (status == PMIC_ST_SUCCESS)
            {
                Pmic_setBitField(&regData, grpSelShift, (uint8_t)(0x03U << grpSelShift), vccaVmonCfg->grpSel);
                status = Pmic_ioTxByte(handle, grpSelRegAddr, regData);
            }
            Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
        }
    }

    return status;
}

// Get VCCA_VMON/VMONx configuration
static int32_t PWR_getVccaVmonCfg(const Pmic_Handle_t *handle, Pmic_PwrVccaVmonCfg_t *vccaVmonCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;
    const bool isVcca = (vccaVmonCfg->resource == PMIC_POWER_RESOURCE_VCCA_VMON);

    // Get VMON enable
    if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_EN_VALID, status))
    {
        uint8_t vmonId = 0U;
        uint8_t vmonEnShift;
        PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);

        if (isVcca)
        {
            vmonEnShift = VCCA_VMON_EN_SHIFT;
        }
        else if (vmonId == PMIC_POWER_RESOURCE_ID_VMON1)
        {
            vmonEnShift = VMON1_EN_SHIFT;
        }
        else
        {
            vmonEnShift = VMON2_EN_SHIFT;
        }

        status = Pmic_ioRxByte_CS(handle, VCCA_VMON_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            vccaVmonCfg->vmonEn = Pmic_getBitField_b(regData, vmonEnShift);
        }
    }

    // Get PG window (threshold)
    if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_THR_VALID, status) ||
        (isVcca && Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_PG_SET_VALID, status)))
    {
        uint8_t vmonId = 0U;
        uint16_t pgWindowRegAddr = 0U;
        PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);
        if (isVcca)
        {
            pgWindowRegAddr = VCCA_PG_WINDOW_REG;
        }
        else
        {
            PWR_getVmonPgWindowRegAddr(vmonId, &pgWindowRegAddr);
        }

        status = Pmic_ioRxByte_CS(handle, pgWindowRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_THR_VALID))
            {
                uint8_t thrShift = isVcca ? VCCA_VMON_THR_SHIFT : VMON_THR_SHIFT;
                uint8_t thrMask = isVcca ? VCCA_VMON_THR_MASK : VMON_THR_MASK;
                vccaVmonCfg->vmonThr = Pmic_getBitField(regData, thrShift, thrMask);
            }

            if (isVcca && Pmic_validParamCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_PG_SET_VALID))
            {
                vccaVmonCfg->pgSet = Pmic_getBitField(regData, VCCA_PG_SET_SHIFT, VCCA_PG_SET_MASK);
            }
        }
    }

    // Get PG level (for VMON1/VMON2)
    if (!isVcca && Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_PG_SET_VALID, status))
    {
        uint8_t vmonId = 0U;
        uint16_t pgLevelRegAddr = 0U;
        PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);
        PWR_getVmonPgLevelRegAddr(vmonId, &pgLevelRegAddr);

        status = Pmic_ioRxByte_CS(handle, pgLevelRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            vccaVmonCfg->pgSet = regData;
        }
    }

    // Get group select
    if (Pmic_validParamStatusCheck(vccaVmonCfg->validParams, PMIC_POWER_VCCA_VMON_GRP_SEL_VALID, status))
    {
        uint8_t vmonId = 0U;
        uint16_t grpSelRegAddr;
        uint8_t grpSelShift;
        PWR_getRsrcId((uint16_t)((uint16_t)vccaVmonCfg->resource), &vmonId);

        PWR_getVmonGrpSelInfo(vmonId, &grpSelRegAddr, &grpSelShift);

        status = Pmic_ioRxByte_CS(handle, grpSelRegAddr, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            vccaVmonCfg->grpSel = Pmic_getBitField(regData, grpSelShift, (uint8_t)(0x03U << grpSelShift));
        }
    }

    return status;
}

/* ========================================================================== */
/*                          Public API Implementations                        */
/* ========================================================================== */

int32_t Pmic_pwrSetBuckCfg(const Pmic_Handle_t *handle, const Pmic_PwrBuckCfg_t *buckCfg)
{
    Pmic_PwrBuckCfg_t buckCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (buckCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (buckCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    if (!PWR_isBuck((uint16_t)((uint16_t)buckCfg->resource)))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrBuckCfg(buckCfg, &buckCfgLocal);
    return Pmic_logStatus(handle, PWR_setBuckCfg(handle, &buckCfgLocal));
}

int32_t Pmic_pwrGetBuckCfg(const Pmic_Handle_t *handle, Pmic_PwrBuckCfg_t *buckCfg)
{
    Pmic_PwrBuckCfg_t buckCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (buckCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (buckCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    if (!PWR_isBuck((uint16_t)((uint16_t)buckCfg->resource)))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrBuckCfg(buckCfg, &buckCfgLocal);
    status = PWR_getBuckCfg(handle, &buckCfgLocal);

    if (status == PMIC_ST_SUCCESS)
    {
        PWR_copyPwrBuckCfg(&buckCfgLocal, buckCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrSetLdoCfg(const Pmic_Handle_t *handle, const Pmic_PwrLdoCfg_t *ldoCfg)
{
    Pmic_PwrLdoCfg_t ldoCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (ldoCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (ldoCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    if (!PWR_isLdo((uint16_t)((uint16_t)ldoCfg->resource)))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrLdoCfg(ldoCfg, &ldoCfgLocal);
    return Pmic_logStatus(handle, PWR_setLdoCfg(handle, &ldoCfgLocal));
}

int32_t Pmic_pwrGetLdoCfg(const Pmic_Handle_t *handle, Pmic_PwrLdoCfg_t *ldoCfg)
{
    Pmic_PwrLdoCfg_t ldoCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (ldoCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (ldoCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    if (!PWR_isLdo((uint16_t)((uint16_t)ldoCfg->resource)))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrLdoCfg(ldoCfg, &ldoCfgLocal);
    status = PWR_getLdoCfg(handle, &ldoCfgLocal);

    if (status == PMIC_ST_SUCCESS)
    {
        PWR_copyPwrLdoCfg(&ldoCfgLocal, ldoCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrSetVccaVmonCfg(const Pmic_Handle_t *handle, const Pmic_PwrVccaVmonCfg_t *vccaVmonCfg)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (vccaVmonCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (vccaVmonCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    if (!PWR_isVmon((uint16_t)((uint16_t)vccaVmonCfg->resource)))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrVccaVmonCfg(vccaVmonCfg, &vccaVmonCfgLocal);
    return Pmic_logStatus(handle, PWR_setVccaVmonCfg(handle, &vccaVmonCfgLocal));
}

int32_t Pmic_pwrGetVccaVmonCfg(const Pmic_Handle_t *handle, Pmic_PwrVccaVmonCfg_t *vccaVmonCfg)
{
    Pmic_PwrVccaVmonCfg_t vccaVmonCfgLocal;
    int32_t status = Pmic_checkHandle(handle);

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (vccaVmonCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (vccaVmonCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    if (!PWR_isVmon((uint16_t)((uint16_t)vccaVmonCfg->resource)))
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrVccaVmonCfg(vccaVmonCfg, &vccaVmonCfgLocal);
    status = PWR_getVccaVmonCfg(handle, &vccaVmonCfgLocal);

    if (status == PMIC_ST_SUCCESS)
    {
        PWR_copyPwrVccaVmonCfg(&vccaVmonCfgLocal, vccaVmonCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrSetGlobalVmonDegl(const Pmic_Handle_t *handle, uint8_t vmonDegl)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (vmonDegl > PMIC_POWER_VMON_DEGL_SEL_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, VCCA_VMON_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, VMON_DEGLITCH_SEL_SHIFT, VMON_DEGLITCH_SEL_MASK, vmonDegl);
            status = Pmic_ioTxByte(handle, VCCA_VMON_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrSetThermalCfg(const Pmic_Handle_t *handle, const Pmic_PwrThermalCfg_t *thermalCfg)
{
    Pmic_PwrThermalCfg_t thermalCfgLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (thermalCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (thermalCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrThermalCfg(thermalCfg, &thermalCfgLocal);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, CONFIG_1_REG, &regData);

    if (Pmic_validParamStatusCheck(thermalCfgLocal.validParams, PMIC_POWER_TWARN_LEVEL_VALID, status))
    {
        if (thermalCfgLocal.twarnLvl > PMIC_POWER_TWARN_LEVEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, TWARN_LEVEL_SHIFT, TWARN_LEVEL_MASK, thermalCfgLocal.twarnLvl);
        }
    }

    if (Pmic_validParamStatusCheck(thermalCfgLocal.validParams, PMIC_POWER_TSD_ORD_LEVEL_VALID, status))
    {
        if (thermalCfgLocal.tsdOrdLvl > PMIC_POWER_TSD_ORD_LEVEL_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        else
        {
            Pmic_setBitField(&regData, TSD_ORD_LEVEL_SHIFT, TSD_ORD_LEVEL_MASK, thermalCfgLocal.tsdOrdLvl);
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, CONFIG_1_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrGetThermalCfg(const Pmic_Handle_t *handle, Pmic_PwrThermalCfg_t *thermalCfg)
{
    Pmic_PwrThermalCfg_t thermalCfgLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (thermalCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (thermalCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrThermalCfg(thermalCfg, &thermalCfgLocal);
    status = Pmic_ioRxByte_CS(handle, CONFIG_1_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(thermalCfgLocal.validParams, PMIC_POWER_TWARN_LEVEL_VALID))
        {
            thermalCfgLocal.twarnLvl = Pmic_getBitField(regData, TWARN_LEVEL_SHIFT, TWARN_LEVEL_MASK);
        }

        if (Pmic_validParamCheck(thermalCfgLocal.validParams, PMIC_POWER_TSD_ORD_LEVEL_VALID))
        {
            thermalCfgLocal.tsdOrdLvl = Pmic_getBitField(regData, TSD_ORD_LEVEL_SHIFT, TSD_ORD_LEVEL_MASK);
        }

        PWR_copyPwrThermalCfg(&thermalCfgLocal, thermalCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrSetSpreadSpectrumCfg(const Pmic_Handle_t *handle, const Pmic_PwrSpreadSpectrumCfg_t *spreadSpectrumCfg)
{
    Pmic_PwrSpreadSpectrumCfg_t spreadSpectrumCfgLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (spreadSpectrumCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (spreadSpectrumCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrSpreadSpectrumCfg(spreadSpectrumCfg, &spreadSpectrumCfgLocal);

    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, SPREAD_SPECTRUM_1_REG, &regData);

    if (Pmic_validParamStatusCheck(spreadSpectrumCfgLocal.validParams, PMIC_POWER_SS_EN_VALID, status))
    {
        Pmic_setBitField_b(&regData, SS_EN_SHIFT, SS_EN_MASK, spreadSpectrumCfgLocal.ssEn);
    }

    if (Pmic_validParamStatusCheck(spreadSpectrumCfgLocal.validParams, PMIC_POWER_SS_DEPTH_VALID, status))
    {
        Pmic_setBitField(&regData, SS_DEPTH_SHIFT, SS_DEPTH_MASK, spreadSpectrumCfgLocal.ssDepth);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte(handle, SPREAD_SPECTRUM_1_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrGetSpreadSpectrumCfg(const Pmic_Handle_t *handle, Pmic_PwrSpreadSpectrumCfg_t *spreadSpectrumCfg)
{
    Pmic_PwrSpreadSpectrumCfg_t spreadSpectrumCfgLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (spreadSpectrumCfg == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (spreadSpectrumCfg->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrSpreadSpectrumCfg(spreadSpectrumCfg, &spreadSpectrumCfgLocal);
    status = Pmic_ioRxByte_CS(handle, SPREAD_SPECTRUM_1_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(spreadSpectrumCfgLocal.validParams, PMIC_POWER_SS_EN_VALID))
        {
            spreadSpectrumCfgLocal.ssEn = Pmic_getBitField_b(regData, SS_EN_SHIFT);
        }

        if (Pmic_validParamCheck(spreadSpectrumCfgLocal.validParams, PMIC_POWER_SS_DEPTH_VALID))
        {
            spreadSpectrumCfgLocal.ssDepth = Pmic_getBitField_b(regData, SS_DEPTH_SHIFT);
        }

        PWR_copyPwrSpreadSpectrumCfg(&spreadSpectrumCfgLocal, spreadSpectrumCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_pwrGetRsrcStatus(const Pmic_Handle_t *handle, Pmic_PwrRsrcStatus_t *rsrcStatus)
{
    Pmic_PwrRsrcStatus_t rsrcStatusLocal;
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if (status != PMIC_ST_SUCCESS)
    {
        return Pmic_logStatus(handle, status);
    }

    if (rsrcStatus == NULL)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_NULL_PARAM);
    }

    if (rsrcStatus->validParams == 0U)
    {
        return Pmic_logStatus(handle, PMIC_ST_ERR_INV_PARAM);
    }

    PWR_copyPwrRsrcStatus(rsrcStatus, &rsrcStatusLocal);

    // Get buck UVOV status
    if (
        (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK1_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK2_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK3_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK4_UVOV_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, STAT_BUCK_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK1_UVOV_VALID))
            {
                rsrcStatusLocal.buck1UVOV = Pmic_getBitField_b(regData, BUCK1_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK2_UVOV_VALID))
            {
                rsrcStatusLocal.buck2UVOV = Pmic_getBitField_b(regData, BUCK2_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK3_UVOV_VALID))
            {
                rsrcStatusLocal.buck3UVOV = Pmic_getBitField_b(regData, BUCK3_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_BUCK4_UVOV_VALID))
            {
                rsrcStatusLocal.buck4UVOV = Pmic_getBitField_b(regData, BUCK4_UVOV_STAT_SHIFT);
            }
        }
    }

    // Get LDO and VMON UVOV status
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_LDO1_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_LDO2_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_LDO3_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_VMON1_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_VMON2_UVOV_VALID) ||
         Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_VCCA_VMON_UVOV_VALID)))
    {
        status = Pmic_ioRxByte_CS(handle, STAT_LDO_VMON_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_LDO1_UVOV_VALID))
            {
                rsrcStatusLocal.ldo1UVOV = Pmic_getBitField_b(regData, LDO1_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_LDO2_UVOV_VALID))
            {
                rsrcStatusLocal.ldo2UVOV = Pmic_getBitField_b(regData, LDO2_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_LDO3_UVOV_VALID))
            {
                rsrcStatusLocal.ldo3UVOV = Pmic_getBitField_b(regData, LDO3_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_VMON1_UVOV_VALID))
            {
                rsrcStatusLocal.vmon1UVOV = Pmic_getBitField_b(regData, VMON1_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_VMON2_UVOV_VALID))
            {
                rsrcStatusLocal.vmon2UVOV = Pmic_getBitField_b(regData, VMON2_UVOV_STAT_SHIFT);
            }

            if (Pmic_validParamCheck(rsrcStatusLocal.validParams, PMIC_POWER_VCCA_VMON_UVOV_VALID))
            {
                rsrcStatusLocal.vccaVmonUVOV = Pmic_getBitField_b(regData, VCCA_UVOV_STAT_SHIFT);
            }
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        PWR_copyPwrRsrcStatus(&rsrcStatusLocal, rsrcStatus);
    }

    return Pmic_logStatus(handle, status);
}
