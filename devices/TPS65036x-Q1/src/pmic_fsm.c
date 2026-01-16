#include "pmic.h"
#include "pmic_io.h"

#include "pmic_fsm.h"

#include "regmap/core.h"

static int32_t FSM_checkFsmCmd(uint8_t fsmCmd)
{
    int32_t status = PMIC_ST_SUCCESS;

    if ((fsmCmd != PMIC_SAFE_RECOVERY_REQUEST) &&
        (fsmCmd != PMIC_COLD_BOOT_REQUEST) &&
        (fsmCmd != PMIC_LOW_POWER_ENTRY_REQUEST) &&
        (fsmCmd != PMIC_OFF_REQUEST) &&
        (fsmCmd != PMIC_LOW_POWER_EXIT_REQUEST) &&
        (fsmCmd != PMIC_WARM_RESET_REQUEST))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    return status;
}

int32_t Pmic_fsmSetDevState(const Pmic_Handle_t *handle, uint8_t fsmCmd)
{
    int32_t status = Pmic_checkHandle(handle);

    if (status == PMIC_ST_SUCCESS)
    {
        status = FSM_checkFsmCmd(fsmCmd);
    }

    // Write FSM command to FSM_COMMAND_REG
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioTxByte_CS(handle, PMIC_FSM_COMMAND_REG_REG, fsmCmd);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetRecovCntThr(const Pmic_Handle_t *handle, uint8_t threshold)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold > PMIC_RESET_RECOV_CNT_THR_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read RECOV_CNT_REG_2
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_RECOV_CNT_REG_2_REG, &regData);
    }

    // Modify RECOV_CNT_THR and Write RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RECOV_CNT_THR_SHIFT, PMIC_RECOV_CNT_THR_MASK, threshold);
        status = Pmic_ioTxByte(handle, PMIC_RECOV_CNT_REG_2_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetRecovCntThr(const Pmic_Handle_t *handle, uint8_t *threshold)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_2 and extract RECOV_CNT_THR
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_RECOV_CNT_REG_2_REG, &regData);
        *threshold = Pmic_getBitField(regData, PMIC_RECOV_CNT_THR_SHIFT, PMIC_RECOV_CNT_THR_MASK);
    }

    return Pmic_logStatus(handle, status);
}

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
        status = Pmic_ioRxByte_CS(handle, PMIC_RECOV_CNT_REG_1_REG, &regData);
        *recovCnt = Pmic_getBitField(regData, PMIC_RECOV_CNT_SHIFT, PMIC_RECOV_CNT_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmClrRecovCnt(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Set RECOV_CNT_CLR bit field to 1 and write to RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RECOV_CNT_CLR_SHIFT, PMIC_RECOV_CNT_CLR_MASK, 1U);
        status = Pmic_ioTxByte_CS(handle, PMIC_RECOV_CNT_CONTROL_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetResetCntThr(const Pmic_Handle_t *handle, uint8_t threshold)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold > PMIC_RESET_RECOV_CNT_THR_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    // Read RECOV_CNT_REG_2
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte(handle, PMIC_RECOV_CNT_REG_2_REG, &regData);
    }

    // Modify RESET_CNT_THR and write RECOV_CNT_REG_2
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RESET_CNT_THR_SHIFT, PMIC_RESET_CNT_THR_MASK, threshold);
        status = Pmic_ioTxByte(handle, PMIC_RECOV_CNT_REG_2_REG, regData);
    }
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetResetCntThr(const Pmic_Handle_t *handle, uint8_t *threshold)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (threshold == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_2 and extract RESET_CNT_THR
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_RECOV_CNT_REG_2_REG, &regData);
        *threshold = Pmic_getBitField(regData, PMIC_RESET_CNT_THR_SHIFT, PMIC_RESET_CNT_THR_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetResetCnt(const Pmic_Handle_t *handle, uint8_t *resetCnt)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (resetCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RECOV_CNT_REG_1 and extract RESET_CNT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, PMIC_RECOV_CNT_REG_1_REG, &regData);
        *resetCnt = Pmic_getBitField(regData, PMIC_RESET_CNT_SHIFT, PMIC_RESET_CNT_MASK);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmClrResetCnt(const Pmic_Handle_t *handle)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    // Set RESET_CNT_CLR bit field to 1 and write to RECOV_CNT_CONTROL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_setBitField(&regData, PMIC_RESET_CNT_CLR_SHIFT, PMIC_RESET_CNT_CLR_MASK, 1U);
        status = Pmic_ioTxByte_CS(handle, PMIC_RECOV_CNT_CONTROL_REG, regData);
    }

    return Pmic_logStatus(handle, status);
}
