/******************************************************************************
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
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
 *  \file  pmic_io.c
 *
 *  \brief  This file contains LLD-Communication wrappers with CRC8 support for
 *          I2C/SPI
 */

#include <pmic_io_priv.h>

const uint8_t CRC_8_TABLE[PMIC_CRC_BUF_SIZE] =
{
      0,  94, 188, 226,  97,  63, 221, 131, 194, 156, 126,  32, 163, 253,
     31,  65, 157, 195,  33, 127, 252, 162,  64,  30,  95,   1, 227, 189,
     62,  96, 130, 220,  35, 125, 159, 193,  66,  28, 254, 160, 225, 191,
     93,   3, 128, 222,  60,  98, 190, 224,   2,  92, 223, 129,  99,  61,
    124,  34, 192, 158,  29,  67, 161, 255, 70 ,  24, 250, 164,  39, 121,
    155, 197, 132, 218,  56, 102, 229, 187,  89,   7, 219, 133, 103,  57,
    186, 228,   6,  88,  25,  71, 165, 251, 120,  38, 196, 154, 101,  59,
    217, 135,   4,  90, 184, 230, 167, 249,  27,  69, 198, 152, 122,  36,
    248, 166,  68,  26, 153, 199,  37, 123,  58, 100, 134, 216,  91,   5,
    231, 185, 140, 210,  48, 110, 237, 179,  81,  15,  78,  16, 242, 172,
     47, 113, 147, 205, 17 ,  79, 173, 243, 112,  46, 204, 146, 211, 141,
    111,  49, 178, 236,  14,  80, 175, 241,  19,  77, 206, 144, 114,  44,
    109,  51, 209, 143,  12,  82, 176, 238, 50 , 108, 142, 208,  83,  13,
    239, 177, 240, 174,  76,  18, 145, 207,  45, 115, 202, 148, 118,  40,
    171, 245,  23,  73,   8,  86, 180, 234, 105,  55, 213, 139, 87 ,   9,
    235, 181,  54, 104, 138, 212, 149, 203,  41, 119, 244, 170,  72,  22,
    233, 183,  85,  11, 136, 214,  52, 106,  43, 117,  51, 201,  74,  20,
    246, 168, 116,  42, 200, 150,  21,  75, 169, 247, 182, 232,  10,  84,
    215, 137, 107,  53
};

static uint8_t Pmic_getCRC8Val(uint8_t data)
{
    return CRC_8_TABLE[data];
}

/*!
 * \brief: Function call wrappers for LLD write API with CRC8 support
 *         This function does the following:
 *         1. If CRC8 is enabled, calculates CRC8 value for given data byte
 *         2. In case of SPI, forms SPI transfer header for PMIC to understand
 *             which register needs to be accessed - 2 byte header
 *             formation is as per TRM
 *         3. If the register to be addressed is Watchdog register, it updates
 *            the Slave and Register address as per TRM to properly communicate
 *            with PMIC Watchdog module and access it's registers
 *         4. Calls Application provided Transfer function to send the
 *            data byte, along with CRC8 if supported.
 *         5. Works with the valid PMIC instance else does not do any operation
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle
 * \param   regAddr           [IN]    Register address
 * \param   txData            [IN]    Data to be written
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_commIntf_sendByte(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 uint16_t        regAddr,
                                 uint8_t         txData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  buffLength = 1U;
    uint8_t  instType   = PMIC_MAIN_INST;
    uint8_t  txBuf[PMIC_IO_BUF_SIZE] = {0};

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPmicCoreHandle->pCommHandle))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPmicCoreHandle->pFnPmicCommIoWrite))
    {
        pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    }

    if((PMIC_ST_SUCCESS       == pmicStatus) &&
       ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
        (PMIC_INTF_DUAL_I2C   == pPmicCoreHandle->commMode)))
    {
        /* Setup Tx buffer as required for I2C IO with PMIC */
        txBuf[buffLength - 1U]  = txData;

        if(true == pPmicCoreHandle->crcEnable)
        {
            txBuf[buffLength] = Pmic_getCRC8Val(txData);

            /* Increase 1 more byte to store CRC8 */
            buffLength++;
        }

        if(0U != (regAddr & PMIC_WDG_PAGEADDR))
        {
            /* Update slave and register address for Watchdog write access */
            regAddr = (regAddr & PMIC_WDG_PAGEADDR_MASK);

            if(((PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode) &&
               (NULL != pPmicCoreHandle->pQACommHandle)) ||
               (PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode))
            {
                /* QA instance shall be used for WDOG in DUAL-I2C mode */
                instType = (uint8_t)PMIC_QA_INST;
            }
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {

        union spiBuffer_t
        {
            uint32_t n32;
            /* [0-1] = Header, [2] = Data, [3] = CRC8 */
            uint8_t  buf[PMIC_IO_BUF_SIZE];
        } *spiBuff;

        spiBuff = (union spiBuffer_t *)&txBuf;

        /* Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm */

        /* Bits 1-8   = ADDR[7:0] */
        spiBuff->n32  = (regAddr & 0xFFU);

        /* Bits 9-11  = PAGE[2:0] */
        spiBuff->n32 |= (regAddr & 0x700U);
        /* Bit 12  = 0 for Write Request */
        spiBuff->n32 &= ~PMIC_IO_REQ_RW;
        buffLength++;

        /* Bits 17-24 = WDATA[7:0] */
        spiBuff->n32 |= (txData << 16U);
        buffLength++;

        if(true == pPmicCoreHandle->crcEnable)
        {
            /* Bits 25-32 CRC */
            spiBuff->n32 |= (Pmic_getCRC8Val(txData) << 24U);

            /* Increment 1 more byte to store CRC8 */
            buffLength++;
        }
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = pPmicCoreHandle->pFnPmicCommIoWrite(pPmicCoreHandle,
                                                         instType,
                                                         regAddr,
                                                         txBuf,
                                                         buffLength);
    }

    return pmicStatus;
}

/*!
 * \brief: Function call wrappers for LLD write API with CRC8 support
 *         This function does the following:
 *         1. If CRC8 is enabled, calculates CRC8 value for given data byte
 *         2. In case of SPI, forms SPI transfer header for PMIC to understand
 *             which register needs to be accessed - 2 byte header
 *             formation is as per TRM
 *         3. If the register to be addressed is Watchdog register, it updates
 *            the Slave and Register address as per TRM to properly communicate
 *            with PMIC Watchdog module and access it's registers
 *         4. Calls Application provided Transfer function to recive  the
 *            data byte, along with CRC8 if supported.
 *         5. Copies received data byte into pRxBuffer byte buffer
 *         6. Works with the valid PMIC instance else does not do any operation
 *
 * \param   pPmicCoreHandle   [IN]    PMIC Interface Handle.
 * \param   regAddr           [IN]    Register address.
 * \param   pRxBuffer         [OUT]   BUffer to receive data
 *
 * \retval  PMIC_ST_SUCCESS in case of success or appropriate error code.
 *          For valid values \ref Pmic_ErrorCodes
 */
int32_t Pmic_commIntf_recvByte(Pmic_CoreHandle_t *pPmicCoreHandle,
                                 uint16_t        regAddr,
                                 uint8_t         *pRxBuffer)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  buffLength = 1U;
    uint8_t  rxBuf[PMIC_IO_BUF_SIZE]  = {0};
    uint8_t  instType   = PMIC_MAIN_INST;

    if(NULL == pPmicCoreHandle)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPmicCoreHandle->pCommHandle))
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPmicCoreHandle->pFnPmicCommIoRead))
    {
        pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
        (PMIC_INTF_DUAL_I2C   == pPmicCoreHandle->commMode)))
    {
        if(true == pPmicCoreHandle->crcEnable)
        {
            /* increment 1 more byte to read CRC8 */
            buffLength++;
        }

        if(0U != (regAddr & PMIC_WDG_PAGEADDR))
        {
            /* If register is of Watchdog, update slave and register address */
            regAddr   = (regAddr & PMIC_WDG_PAGEADDR_MASK);

            if(((PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode) &&
               (NULL != pPmicCoreHandle->pQACommHandle)) ||
               (PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode))
            {
                /* QACommHandle shall be used for WDOG in DUAL-I2C mode */
                instType = (uint8_t)PMIC_QA_INST;
            }
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        union spiBuffer_t
        {
            uint32_t n32;
            /* [0-1] = Header, [2] = Data, [3] = CRC8 */
            uint8_t  buf[PMIC_IO_BUF_SIZE];
        } *spiBuf;

        spiBuf = (union spiBuffer_t *)&rxBuf;

        /* Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm */

        /* Bits 1-8   = ADDR[7:0] */
        spiBuf->n32  = (regAddr & 0xFFU);
        buffLength++;

        /* Bits 9-11  = PAGE[2:0] */
        spiBuf->n32 |= (regAddr & 0x700U);

        /* Bit 12  = 1 for Read Request */
        spiBuf->n32 |= PMIC_IO_REQ_RW;

        /* Increment 1 more byte for 8-bit data read from PMIC register */
        buffLength++;

        if(true == pPmicCoreHandle->crcEnable)
        {
            /* Increment 1 more byte to read CRC8 */
            buffLength++;
        }
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = pPmicCoreHandle->pFnPmicCommIoRead(pPmicCoreHandle,
                                                        instType,
                                                        regAddr,
                                                        rxBuf,
                                                        buffLength);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) && (true == pPmicCoreHandle->crcEnable))
    {
        if((rxBuf[buffLength - 2U]) !=
           (Pmic_getCRC8Val(rxBuf[buffLength - 1U])))
        {
            pmicStatus = PMIC_ST_ERR_DATA_IO_CRC;
        }

        buffLength--;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        /* Copy data which shall be in rxBuf[2]/rxBuf[0] to pRxBuffer */
        *pRxBuffer = rxBuf[buffLength - 1U];
    }

    return pmicStatus;
}
