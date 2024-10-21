/**
 * @file common_test.c
 *
 * @brief Source file containing definitions of APIs used in all tests.
 */
#include "common_test.h"

void unityCharPut(uint8_t c)
{
    DebugP_log("%c", c);
    if (c == '\n')
    {
        DebugP_log("\r");
    }
}

void app_critSecStart(void)
{
    /* Empty - No RTOS */
}

void app_critSecStop(void)
{
    /* Empty - No RTOS */
}

void app_irqResponse(void)
{
    /* Empty - No IRQ response */
}

int32_t app_spiTransfer(const Pmic_CoreHandle_t *pmicHandle, uint32_t txBuf, uint32_t *rxBuf, uint8_t len)
{
    int32_t status = PMIC_ST_SUCCESS;
    MCSPI_Transaction spiTransaction;

    if ((pmicHandle == NULL) || (rxBuf == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Specify SPI transaction
        MCSPI_Transaction_init(&spiTransaction);
        spiTransaction.channel = gPmicMcspi1ChCfg[0U].chNum;
        spiTransaction.dataSize = 8U*len;           // Data size (in bits)
        spiTransaction.csDisable = TRUE;            // CS is de-asserted automatically at the end of the transfer
        spiTransaction.count = 1U;                  // Only one SPI transaction will occur
        spiTransaction.txBuf = (void *)(&txBuf);    // Buffer containing data to TX
        spiTransaction.rxBuf = (void *)(rxBuf);     // Buffer containing data to be RX'd
        spiTransaction.args = NULL;

        // Begin SPI transaction
        status = MCSPI_transfer(gMcspiHandle[PMIC_MCSPI1], &spiTransaction);

        // Convert platform-specific API return code to driver return code
        if (status == SystemP_SUCCESS)
        {
            status = PMIC_ST_SUCCESS;
        }
    }

    // Convert platform-specific API return code to driver return code
    if ((status == PMIC_ST_SUCCESS) && (spiTransaction.status != MCSPI_TRANSFER_COMPLETED))
    {
        status = PMIC_ST_ERR_SPI_COMM_FAIL;
    }

    return status;
}

void app_wait(uint16_t milliseconds)
{

}
