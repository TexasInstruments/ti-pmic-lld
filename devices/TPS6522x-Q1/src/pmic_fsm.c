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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_fsm.h"
#include "pmic_io.h"
#include "regmap/fsm.h"

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_fsmGetRecovCnt(const Pmic_Handle_t *handle, uint8_t *recovCnt)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_1 and extract RECOV_CNT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, RECOV_CNT_REG_1_REGADDR, &regData);
        *recovCnt = Pmic_getBitField(regData, RECOV_CNT_SHIFT, RECOV_CNT_MASK);
    }

    return status;
}

int32_t Pmic_fsmClrRecovCnt(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Set RECOV_CNT_CLR bit field to 1 and write to RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, RECOV_CNT_CLR_SHIFT, RECOV_CNT_CLR_MASK, 1U);
        status = Pmic_ioTxByte_CS(handle, RECOV_CNT_REG_2_REGADDR, regData);
    }

    return status;
}

int32_t Pmic_fsmSetRecovCntThr(const Pmic_Handle_t *handle, uint8_t recovCntThr)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCntThr > PMIC_FSM_RECOV_CNT_THR_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read RECOV_CNT_REG_2
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, RECOV_CNT_REG_2_REGADDR, &regData);
    }

    // Modify RECOV_CNT_THR and Write RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, RECOV_CNT_THR_SHIFT, RECOV_CNT_THR_MASK, recovCntThr);
        status = Pmic_ioTxByte(handle, RECOV_CNT_REG_2_REGADDR, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_fsmGetRecovCntThr(const Pmic_Handle_t *handle, uint8_t *recovCntThr)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCntThr == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_2 and extract RECOV_CNT_THR
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, RECOV_CNT_REG_2_REGADDR, &regData);
        *recovCntThr = Pmic_getBitField(regData, RECOV_CNT_THR_SHIFT, RECOV_CNT_THR_MASK);
    }

    return status;
}

int32_t Pmic_fsmSendSoftRebootReq(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Set SOFT_REBOOT bit field to 1 and write to SOFT_REBOOT_REG
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, SOFT_REBOOT_SHIFT, SOFT_REBOOT_MASK, 1U);
        status = Pmic_ioTxByte_CS(handle, SOFT_REBOOT_REG_REGADDR, regData);
    }

    return status;
}

int32_t Pmic_fsmSetStartupDest(const Pmic_Handle_t *handle, uint8_t destination)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (destination > PMIOC_FSM_START_UP_DEST_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read STARTUP_CTRL
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, STARTUP_CTRL_REGADDR, &regData);
    }

    // Modify STARTUP_DEST and write STARTUP_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, STARTUP_DEST_SHIFT, STARTUP_DEST_MASK, destination);
        status = Pmic_ioTxByte(handle, STARTUP_CTRL_REGADDR, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_fsmGetStartupDest(const Pmic_Handle_t *handle, uint8_t *destination)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (destination == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read STARTUP_CTRL and extract STARTUP_DEST
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, STARTUP_CTRL_REGADDR, &regData);
        *destination = Pmic_getBitField(regData, STARTUP_DEST_SHIFT, STARTUP_DEST_MASK);
    }

    return status;
}

int32_t Pmic_fsmSetGpioTriggerCfg(const Pmic_Handle_t *handle, const Pmic_FsmGpioTriggerCfg_t *gpioTriggerCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t maskShift = 0U;
    uint8_t maskPolShift = 0U;

    if ((status == PMIC_ST_SUCCESS) && (gpioTriggerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioTriggerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate GPIO pin number
    if ((status == PMIC_ST_SUCCESS) &&
        ((gpioTriggerCfg->pinNum < PMIC_FSM_GPIO_PIN_MIN) ||
         (gpioTriggerCfg->pinNum > PMIC_FSM_GPIO_PIN_MAX)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate mask polarity
    if ((status == PMIC_ST_SUCCESS) &&
        Pmic_validParamCheck(gpioTriggerCfg->validParams, PMIC_FSM_MASK_POL_VALID) &&
        (gpioTriggerCfg->maskPol > PMIC_FSM_GPIO_MASK_POL_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Determine register address and bit field shifts based on GPIO pin number
    if (status == PMIC_ST_SUCCESS)
    {
        switch (gpioTriggerCfg->pinNum)
        {
            case PMIC_FSM_GPIO_PIN1:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO1_FSM_MASK_SHIFT;
                maskPolShift = GPIO1_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN2:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO2_FSM_MASK_SHIFT;
                maskPolShift = GPIO2_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN3:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO3_FSM_MASK_SHIFT;
                maskPolShift = GPIO3_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN4:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO4_FSM_MASK_SHIFT;
                maskPolShift = GPIO4_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN5:
                regAddr = FSM_TRIG_MASK_2_REGADDR;
                maskShift = GPIO5_FSM_MASK_SHIFT;
                maskPolShift = GPIO5_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN6:
                regAddr = FSM_TRIG_MASK_2_REGADDR;
                maskShift = GPIO6_FSM_MASK_SHIFT;
                maskPolShift = GPIO6_FSM_MASK_POL_SHIFT;
                break;
            default:
                status = PMIC_ST_ERR_INV_PARAM;
                break;
        }
    }

    // Read the appropriate FSM_TRIG_MASK register
    Pmic_criticalSectionStart(handle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Modify GPIO FSM mask
        if (Pmic_validParamCheck(gpioTriggerCfg->validParams, PMIC_FSM_MASK_VALID))
        {
            Pmic_setBitField_b(&regData, maskShift, (uint8_t)(1U << maskShift), gpioTriggerCfg->mask);
        }

        // Modify GPIO FSM mask polarity
        if (Pmic_validParamCheck(gpioTriggerCfg->validParams, PMIC_FSM_MASK_POL_VALID))
        {
            Pmic_setBitField(&regData, maskPolShift, (uint8_t)(1U << maskPolShift), gpioTriggerCfg->maskPol);
        }

        // Write the modified register
        status = Pmic_ioTxByte(handle, regAddr, regData);
    }
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_fsmGetGpioTriggerCfg(const Pmic_Handle_t *handle, Pmic_FsmGpioTriggerCfg_t *gpioTriggerCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t regAddr = 0U;
    uint8_t maskShift = 0U;
    uint8_t maskPolShift = 0U;

    if ((status == PMIC_ST_SUCCESS) && (gpioTriggerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (gpioTriggerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate GPIO pin number
    if ((status == PMIC_ST_SUCCESS) &&
        ((gpioTriggerCfg->pinNum < PMIC_FSM_GPIO_PIN_MIN) ||
         (gpioTriggerCfg->pinNum > PMIC_FSM_GPIO_PIN_MAX)))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Determine register address and bit field shifts based on GPIO pin number
    if (status == PMIC_ST_SUCCESS)
    {
        switch (gpioTriggerCfg->pinNum)
        {
            case PMIC_FSM_GPIO_PIN1:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO1_FSM_MASK_SHIFT;
                maskPolShift = GPIO1_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN2:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO2_FSM_MASK_SHIFT;
                maskPolShift = GPIO2_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN3:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO3_FSM_MASK_SHIFT;
                maskPolShift = GPIO3_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN4:
                regAddr = FSM_TRIG_MASK_1_REGADDR;
                maskShift = GPIO4_FSM_MASK_SHIFT;
                maskPolShift = GPIO4_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN5:
                regAddr = FSM_TRIG_MASK_2_REGADDR;
                maskShift = GPIO5_FSM_MASK_SHIFT;
                maskPolShift = GPIO5_FSM_MASK_POL_SHIFT;
                break;
            case PMIC_FSM_GPIO_PIN6:
                regAddr = FSM_TRIG_MASK_2_REGADDR;
                maskShift = GPIO6_FSM_MASK_SHIFT;
                maskPolShift = GPIO6_FSM_MASK_POL_SHIFT;
                break;
            default:
                status = PMIC_ST_ERR_INV_PARAM;
                break;
        }
    }

    // Read the appropriate FSM_TRIG_MASK register
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, regAddr, &regData);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract GPIO FSM mask
        if (Pmic_validParamCheck(gpioTriggerCfg->validParams, PMIC_FSM_MASK_VALID))
        {
            gpioTriggerCfg->mask = Pmic_getBitField_b(regData, maskShift);
        }

        // Extract GPIO FSM mask polarity
        if (Pmic_validParamCheck(gpioTriggerCfg->validParams, PMIC_FSM_MASK_POL_VALID))
        {
            gpioTriggerCfg->maskPol = Pmic_getBitField(regData, maskPolShift, (uint8_t)(1U << maskPolShift));
        }
    }

    return status;
}

int32_t Pmic_fsmSetTriggerCfg(const Pmic_Handle_t *handle, const Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData1 = 0U;
    uint8_t regData2 = 0U;
    bool updateReg1 = false;
    bool updateReg2 = false;

    if ((status == PMIC_ST_SUCCESS) && (triggerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (triggerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Validate trigger values
    if (status == PMIC_ST_SUCCESS)
    {
        if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SEVERE_ERR_TRIG_VALID) &&
            (triggerCfg->severeErrTrig > PMIC_FSM_TRIGGER_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_OTHER_RAIL_TRIG_VALID) &&
            (triggerCfg->otherRailTrig > PMIC_FSM_TRIGGER_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SOC_RAIL_TRIG_VALID) &&
            (triggerCfg->socRailTrig > PMIC_FSM_TRIGGER_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MCU_RAIL_TRIG_VALID) &&
            (triggerCfg->mcuRailTrig > PMIC_FSM_TRIGGER_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
        if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MODERATE_ERR_TRIG_VALID) &&
            (triggerCfg->moderateErrTrig > PMIC_FSM_TRIGGER_MAX))
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    Pmic_criticalSectionStart(handle);

    // Determine which registers need to be updated
    if (status == PMIC_ST_SUCCESS)
    {
        updateReg1 = (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SEVERE_ERR_TRIG_VALID) ||
                      Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_OTHER_RAIL_TRIG_VALID) ||
                      Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SOC_RAIL_TRIG_VALID) ||
                      Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MCU_RAIL_TRIG_VALID));

        updateReg2 = Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MODERATE_ERR_TRIG_VALID);
    }

    // Update FSM_TRIG_SEL_1 register
    if ((status == PMIC_ST_SUCCESS) && updateReg1)
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_1_REGADDR, &regData1);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SEVERE_ERR_TRIG_VALID))
            {
                Pmic_setBitField(&regData1, SEVERE_ERR_TRIG_SHIFT, SEVERE_ERR_TRIG_MASK, triggerCfg->severeErrTrig);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_OTHER_RAIL_TRIG_VALID))
            {
                Pmic_setBitField(&regData1, OTHER_RAIL_TRIG_SHIFT, OTHER_RAIL_TRIG_MASK, triggerCfg->otherRailTrig);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SOC_RAIL_TRIG_VALID))
            {
                Pmic_setBitField(&regData1, SOC_RAIL_TRIG_SHIFT, SOC_RAIL_TRIG_MASK, triggerCfg->socRailTrig);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MCU_RAIL_TRIG_VALID))
            {
                Pmic_setBitField(&regData1, MCU_RAIL_TRIG_SHIFT, MCU_RAIL_TRIG_MASK, triggerCfg->mcuRailTrig);
            }

            status = Pmic_ioTxByte(handle, FSM_TRIG_SEL_1_REGADDR, regData1);
        }
    }

    // Update FSM_TRIG_SEL_2 register
    if ((status == PMIC_ST_SUCCESS) && updateReg2)
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_2_REGADDR, &regData2);

        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData2, MODERATE_ERR_TRIG_SHIFT, MODERATE_ERR_TRIG_MASK, triggerCfg->moderateErrTrig);
            status = Pmic_ioTxByte(handle, FSM_TRIG_SEL_2_REGADDR, regData2);
        }
    }

    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_fsmGetTriggerCfg(const Pmic_Handle_t *handle, Pmic_FsmTriggerCfg_t *triggerCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData1 = 0U;
    uint8_t regData2 = 0U;
    bool readReg1 = false;
    bool readReg2 = false;

    if ((status == PMIC_ST_SUCCESS) && (triggerCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (triggerCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Determine which registers need to be read
    if (status == PMIC_ST_SUCCESS)
    {
        readReg1 = (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SEVERE_ERR_TRIG_VALID) ||
                    Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_OTHER_RAIL_TRIG_VALID) ||
                    Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SOC_RAIL_TRIG_VALID) ||
                    Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MCU_RAIL_TRIG_VALID));

        readReg2 = Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MODERATE_ERR_TRIG_VALID);
    }

    Pmic_criticalSectionStart(handle);

    // Read FSM_TRIG_SEL_1 register
    if ((status == PMIC_ST_SUCCESS) && readReg1)
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_1_REGADDR, &regData1);

        if (status == PMIC_ST_SUCCESS)
        {
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SEVERE_ERR_TRIG_VALID))
            {
                triggerCfg->severeErrTrig = Pmic_getBitField(regData1, SEVERE_ERR_TRIG_SHIFT, SEVERE_ERR_TRIG_MASK);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_OTHER_RAIL_TRIG_VALID))
            {
                triggerCfg->otherRailTrig = Pmic_getBitField(regData1, OTHER_RAIL_TRIG_SHIFT, OTHER_RAIL_TRIG_MASK);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_SOC_RAIL_TRIG_VALID))
            {
                triggerCfg->socRailTrig = Pmic_getBitField(regData1, SOC_RAIL_TRIG_SHIFT, SOC_RAIL_TRIG_MASK);
            }
            if (Pmic_validParamCheck(triggerCfg->validParams, PMIC_FSM_MCU_RAIL_TRIG_VALID))
            {
                triggerCfg->mcuRailTrig = Pmic_getBitField(regData1, MCU_RAIL_TRIG_SHIFT, MCU_RAIL_TRIG_MASK);
            }
        }
    }

    // Read FSM_TRIG_SEL_2 register
    if ((status == PMIC_ST_SUCCESS) && readReg2)
    {
        status = Pmic_ioRxByte(handle, FSM_TRIG_SEL_2_REGADDR, &regData2);

        if (status == PMIC_ST_SUCCESS)
        {
            triggerCfg->moderateErrTrig = Pmic_getBitField(regData2, MODERATE_ERR_TRIG_SHIFT, MODERATE_ERR_TRIG_MASK);
        }
    }

    Pmic_criticalSectionStop(handle);

    return status;
}
