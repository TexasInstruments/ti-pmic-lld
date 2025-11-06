#include "pmic.h"
#include "pmic_io.h"

#include "pmic_fsm.h"

#include "regmap/core.h"

int32_t Pmic_fsmSetDevState(const Pmic_CoreHandle_t *pmicHandle, uint8_t fsmCmd)
{
    int32_t status = Pmic_checkHandle(pmicHandle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = CORE_checkFsmCmd(fsmCmd);
    }

    // Write FSM command to FSM_COMMAND_REG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(pmicHandle, PMIC_FSM_COMMAND_REG_REGADDR, fsmCmd);
    }

    return status;
}

int32_t Pmic_fsmSetRecovCntThr(const Pmic_CoreHandle_t *pmicHandle, uint8_t threshold)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold > PMIC_RESET_RECOV_CNT_THR_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read RECOV_CNT_REG_2
    Pmic_criticalSectionStart(pmicHandle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_RECOV_CNT_REG_2_REGADDR, &regData);
    }

    // Modify RECOV_CNT_THR and Write RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RECOV_CNT_THR_SHIFT, PMIC_RECOV_CNT_THR_MASK, threshold);
        status = Pmic_ioTxByte(pmicHandle, PMIC_RECOV_CNT_REG_2_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_fsmGetRecovCntThr(const Pmic_CoreHandle_t *pmicHandle, uint8_t *threshold)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_2 and extract RECOV_CNT_THR
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_RECOV_CNT_REG_2_REGADDR, &regData);
        *threshold = Pmic_getBitField(regData, PMIC_RECOV_CNT_THR_SHIFT, PMIC_RECOV_CNT_THR_MASK);
    }

    return status;
}

int32_t Pmic_fsmGetRecovCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *recovCnt)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (recovCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_1 and extract RECOV_CNT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_RECOV_CNT_REG_1_REGADDR, &regData);
        *recovCnt = Pmic_getBitField(regData, PMIC_RECOV_CNT_SHIFT, PMIC_RECOV_CNT_MASK);
    }

    return status;
}

int32_t Pmic_fsmClrRecovCnt(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    // Set RECOV_CNT_CLR bit field to 1 and write to RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RECOV_CNT_CLR_SHIFT, PMIC_RECOV_CNT_CLR_MASK, 1U);
        status = Pmic_ioTxByte_CS(pmicHandle, PMIC_RECOV_CNT_CONTROL_REGADDR, regData);
    }

    return status;
}

int32_t Pmic_fsmSetResetCntThr(const Pmic_CoreHandle_t *pmicHandle, uint8_t threshold)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold > PMIC_RESET_RECOV_CNT_THR_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read RECOV_CNT_REG_2
    Pmic_criticalSectionStart(pmicHandle);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(pmicHandle, PMIC_RECOV_CNT_REG_2_REGADDR, &regData);
    }

    // Modify RESET_CNT_THR and write RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RESET_CNT_THR_SHIFT, PMIC_RESET_CNT_THR_MASK, threshold);
        status = Pmic_ioTxByte(pmicHandle, PMIC_RECOV_CNT_REG_2_REGADDR, regData);
    }
    Pmic_criticalSectionStop(pmicHandle);

    return status;
}

int32_t Pmic_fsmGetResetCntThr(const Pmic_CoreHandle_t *pmicHandle, uint8_t *threshold)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_2 and extract RESET_CNT_THR
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_RECOV_CNT_REG_2_REGADDR, &regData);
        *threshold = Pmic_getBitField(regData, PMIC_RESET_CNT_THR_SHIFT, PMIC_RESET_CNT_THR_MASK);
    }

    return status;
}

int32_t Pmic_fsmGetResetCnt(const Pmic_CoreHandle_t *pmicHandle, uint8_t *resetCnt)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (resetCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_1 and extract RESET_CNT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(pmicHandle, PMIC_RECOV_CNT_REG_1_REGADDR, &regData);
        *resetCnt = Pmic_getBitField(regData, PMIC_RESET_CNT_SHIFT, PMIC_RESET_CNT_MASK);
    }

    return status;
}

int32_t Pmic_fsmClrResetCnt(const Pmic_CoreHandle_t *pmicHandle)
{
    int32_t status = Pmic_checkHandle(pmicHandle);
    uint8_t regData = 0U;

    // Set RESET_CNT_CLR bit field to 1 and write to RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RESET_CNT_CLR_SHIFT, PMIC_RESET_CNT_CLR_MASK, 1U);
        status = Pmic_ioTxByte_CS(pmicHandle, PMIC_RECOV_CNT_CONTROL_REGADDR, regData);
    }

    return status;
}
