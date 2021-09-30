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

#include <pmic_core_priv.h>
#include <pmic_wdg_priv.h>
#include <pmic_fsm_priv.h>
#include <pmic_power_priv.h>
#include <pmic_power_lp8764x_priv.h>
#include <pmic_irq_priv.h>
#include <pmic_esm_priv.h>
#include <pmic_rtc_priv.h>


/**
 *  Used CRC Polynomial:  x^8 + x^2 + x + 1
 *   Evalution of CRC Polynomial value from equation:
 *     1*x^8 + 0*x^7 + 0*x^6 + 0*x^5 + 0*x^4 + 0*x^3 + 1*x^2 + 1*x + 1
 *  =  1     + 0     + 0     + 0     + 0     + 0     + 1     + 1   + 1
 *  =  0x107(After discarding MSB bit) = 0x7
 *
 * CRC Polynomial value: 0x07, Initial Value: 0x00, Final Value: 0x00
 * link to generate custom CRC table from polynomial:
 * http://www.sunshine2k.de/coding/javascript/crc/crc_js.html
 */

const uint8_t CRC_8_TABLE[] =
{
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
    0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
    0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
    0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
    0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
    0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
    0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
    0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
    0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
    0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
    0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
    0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};


/*!
 * \brief: API to Get CRC8 data
 *
 * \param   data     [IN]     Data for which CRC is to be determined
 * \param   length   [IN]     Length of the buffer in bytes.
 * \retval  crc      [OUT]    CRC value for data.
 */
static uint8_t Pmic_getCRC8Val(const uint8_t *data, uint8_t length)
{
    uint8_t crc = PMIC_COMM_CRC_INITIAL_VALUE;
    uint8_t i;
    for(i = 0; i < length; i++)
    {
        crc = CRC_8_TABLE[data[i] ^ crc];
    }

    return crc;
}

/*!
 * \brief: API to get the status of whether user register is write protected or
 *         not
 */
static int32_t Pmic_getRegWriteProtectionStatus(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t           regAddr)
{
    uint8_t regData     = 0U;
    uint8_t regLockStat = 0U;
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle,
                                        PMIC_REGISTER_LOCK_REGADDR,
                                        &regData);

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        regLockStat = Pmic_getBitField(
                             regData,
                             PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_SHIFT,
                             PMIC_REGISTER_LOCK_REGISTER_LOCK_STATUS_READ_MASK);
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_REGISTER_STATUS_LOCK == regLockStat))
    {
        if((PMIC_SILICON_REV_ID_PG_2_0 == pPmicCoreHandle->pmicDevSiliconRev) &&
           ((regAddr >= PMIC_INT_TOP_REGADDR) &&
            (regAddr <= PMIC_INT_ESM_REGADDR)))
        {
            pmicStatus = PMIC_ST_SUCCESS;
        }
        else
        {
            if(PMIC_DEV_LEO_TPS6594X == pPmicCoreHandle->pmicDeviceType)
            {
                if(((regAddr >= PMIC_BUCK1_CTRL_REGADDR) &&
                    (regAddr <= PMIC_MISC_CTRL_REGADDR)) ||
                   (regAddr == PMIC_RTC_CTRL_1_REGADDR)  ||
                   (regAddr == PMIC_RTC_CTRL_2_REGADDR)  ||
                   (regAddr == PMIC_SOFT_REBOOT_REG_REGADDR))
                {
                    pmicStatus = PMIC_ST_ERR_REG_LOCKED_WR_FAIL;
                }
            }
            else
            {
                if(((regAddr >= PMIC_BUCK1_CTRL_REGADDR) &&
                    (regAddr <= PMIC_BUCK4_PG_WIN_REGADDR)) ||
                   ((regAddr >= PMIC_VCCA_VMON_CTRL_REGADDR) &&
                    (regAddr <= PMIC_MISC_CTRL_REGADDR)) ||
                   (regAddr == PMIC_VMON_CONF_REGADDR) ||
                   (regAddr == PMIC_STARTUP_CTRL_REGADDR))
                {
                    pmicStatus = PMIC_ST_ERR_REG_LOCKED_WR_FAIL;
                }
            }

            if((PMIC_ST_SUCCESS == pmicStatus) &&
               (((regAddr >= PMIC_RECOV_CNT_REG_2_REGADDR) &&
                 (regAddr <= PMIC_USER_SPARE_REGS_REGADDR)) ||
                ((regAddr >= PMIC_SCRATCH_PAD_REG_1_REGADDR) &&
                (regAddr <= PMIC_FSM_PFSM_DELAY_REG_4_REGADDR))))
            {
                pmicStatus = PMIC_ST_ERR_REG_LOCKED_WR_FAIL;
            }
        }
    }

    return pmicStatus;
}

/*
 * \brief  Function to validate pPmicCoreHandle, pCommHandle, pFnPmicCommIoWrite
 *         and pFnPmicCommIoRead and get Register Write Protection status
 */
static int32_t Pmic_validatePmicHandleGetRegWrProtectStat(
                                             Pmic_CoreHandle_t *pPmicCoreHandle,
                                             uint16_t          regAddr)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle->pCommHandle)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPmicCoreHandle->pFnPmicCommIoWrite))
    {
        pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = Pmic_getRegWriteProtectionStatus(pPmicCoreHandle,
                                                      regAddr);
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
                               uint16_t           regAddr,
                               uint8_t            txData)
{
    int32_t pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  buffLength = 1U;
    uint8_t  instType   = PMIC_MAIN_INST;
    uint8_t  txBuf[PMIC_IO_BUF_SIZE]   = {0};
    uint8_t  crcData[PMIC_IO_BUF_SIZE] = {0};
    uint8_t  crcDataLen = 0;
    uint16_t pmicRegAddr = regAddr;

    pmicStatus = Pmic_validatePmicHandleGetRegWrProtectStat(pPmicCoreHandle,
                                                            regAddr);

    if((PMIC_ST_SUCCESS       == pmicStatus) &&
       ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
        (PMIC_INTF_DUAL_I2C   == pPmicCoreHandle->commMode)))
    {
        /* Setup Tx buffer as required for I2C IO with PMIC */
        txBuf[buffLength - 1U]  = txData;

        if(0U != (regAddr & PMIC_WDG_PAGEADDR))
        {
            /* Update slave and register address for Watchdog write access */
            pmicRegAddr = (regAddr & PMIC_WDG_PAGEADDR_MASK);

            if(((PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode) &&
               (NULL != pPmicCoreHandle->pQACommHandle)) ||
               (PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode))
            {
                /* QA instance shall be used for WDOG in DUAL-I2C mode */
                instType = (uint8_t)PMIC_QA_INST;
            }
        }

        if(((bool)true) == pPmicCoreHandle->crcEnable)
        {
            if(PMIC_MAIN_INST == instType)
            {
                /* Store the slave address and I2C write request bit */
                crcData[crcDataLen] = (pPmicCoreHandle->slaveAddr << 1U);
                crcDataLen++;
            }
            else
            {
                /* Store the slave address and I2C write request bit */
                crcData[crcDataLen] = (pPmicCoreHandle->qaSlaveAddr << 1U);
                crcDataLen++;
            }

            /* Store register address */
            crcData[crcDataLen] = (uint8_t)pmicRegAddr;
            crcDataLen++;
            /* Store the data to be transferred */
            crcData[crcDataLen] = txData;
            crcDataLen++;
            /* Increase 1 more byte to store CRC8 */
            txBuf[buffLength]   = Pmic_getCRC8Val(crcData, crcDataLen);
            buffLength++;
        }
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        /*
         * Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         * explained in PMIC TRM
         */

        buffLength = 0;
        /* Set ADDR to txbuf[0] with ADDR[7:0] */
        txBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);
        buffLength++;

        /* Set PAGE to txBuf[1] 7:5 bits with PAGE[2:0] */
        txBuf[buffLength] = (uint8_t)(((pmicRegAddr >> 8U) & 0x7U) << 5U);

        /* Set R/W in txBuf[1] as bit-4, for Write Request */
        txBuf[buffLength] &= (uint8_t)(~PMIC_IO_REQ_RW);
        buffLength++;

        /* Set write data to txBuf[2], with WDATA[7:0] */
        txBuf[buffLength] = txData;
        buffLength++;

        if(((bool)true) == pPmicCoreHandle->crcEnable)
        {
            /* Set CRC data to txBuf[3], Bits 25-32 CRC */
            txBuf[buffLength] = Pmic_getCRC8Val(txBuf, buffLength);
            /* Increment 1 more byte to store CRC8 */
            buffLength++;
        }
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = pPmicCoreHandle->pFnPmicCommIoWrite(pPmicCoreHandle,
                                                         instType,
                                                         pmicRegAddr,
                                                         txBuf,
                                                         buffLength);
    }

    return pmicStatus;
}

static int32_t Pmic_validateCorehandle(const Pmic_CoreHandle_t *pPmicCoreHandle)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;

    if(NULL == pPmicCoreHandle->pCommHandle)
    {
        pmicStatus = PMIC_ST_ERR_NULL_PARAM;
    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (NULL == pPmicCoreHandle->pFnPmicCommIoRead))
    {
        pmicStatus = PMIC_ST_ERR_NULL_FPTR;
    }

    return pmicStatus;
}

/*
 * \brief  Function to read data from PMIC registers based on Comm IO interface
 *         I2C or SPI Interface
 */
static int32_t Pmic_commIoReadData(Pmic_CoreHandle_t *pPmicCoreHandle,
                                   uint16_t          *pRegAddr,
                                   uint8_t           *pBuffLength,
                                   uint8_t           *pRxBuf,
                                   uint8_t           *pInstType)
{
    int32_t  pmicStatus  = PMIC_ST_SUCCESS;
    uint8_t  buffLength  = *pBuffLength;
    uint16_t pmicRegAddr = *pRegAddr;
    uint16_t regAddr     = pmicRegAddr;
    uint8_t  instType    = *pInstType;

    pmicStatus = Pmic_validateCorehandle(pPmicCoreHandle);

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       ((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
        (PMIC_INTF_DUAL_I2C   == pPmicCoreHandle->commMode)))
    {
        if(((bool)true) == pPmicCoreHandle->crcEnable)
        {
            /* increment 1 more byte to read CRC8 */
            buffLength++;
        }

        if(0U != (regAddr & PMIC_WDG_PAGEADDR))
        {
            /* If register is of Watchdog, update slave and register address */
            pmicRegAddr   = (regAddr & PMIC_WDG_PAGEADDR_MASK);

            if(((PMIC_INTF_DUAL_I2C == pPmicCoreHandle->commMode) &&
               (NULL != pPmicCoreHandle->pQACommHandle)) ||
               (PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode))
            {
                /* QACommHandle shall be used for WDOG in DUAL-I2C mode */
                instType = (uint8_t)PMIC_QA_INST;
            }
        }

        if(0U != (regAddr & PMIC_SERIAL_IF_CONFIG_PAGEADDR))
        {
        /* Update slave and register address for SERIAL_IF_CONFIG read access */
            pmicRegAddr = (regAddr & PMIC_SERIAL_IF_CONFIG_PAGEADDR_MASK);

            /* NVM instance shall be used */
            instType = (uint8_t)PMIC_NVM_INST;
        }

        *pBuffLength = buffLength;
        *pRegAddr = pmicRegAddr;
        *pInstType = instType;

    }

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (PMIC_INTF_SPI == pPmicCoreHandle->commMode))
    {
        /*
         * Frame 3 Bytes with IO header+data as per PMIC SPI IO algorithm
         * explained in PMIC TRM
         */
        buffLength = 0U;
        /* Set ADDR to pRxBuf[0], with ADDR[7:0] */
        pRxBuf[buffLength] = (uint8_t)(pmicRegAddr & 0xFFU);
        buffLength++;

        /* Set PAGE to pRxBuf[1] 7:5 bits, with PAGE[2:0] */
        pRxBuf[buffLength] = (uint8_t)(((pmicRegAddr >> 8U) & 0x7U) << 5U);

        /* Set R/W in pRxBuf[1] as bit-4, for read Request */
        pRxBuf[buffLength] |= PMIC_IO_REQ_RW;
        buffLength++;

        /* Increment 1 more byte for 8-bit data read from PMIC register */
        buffLength++;

        if(((bool)true) == pPmicCoreHandle->crcEnable)
        {
            /* Increment 1 more byte to read CRC8 */
            buffLength++;
        }

        *pBuffLength = buffLength;
        *pRegAddr = pmicRegAddr;
    }

    if(PMIC_ST_SUCCESS == pmicStatus)
    {
        pmicStatus = pPmicCoreHandle->pFnPmicCommIoRead(pPmicCoreHandle,
                                                        instType,
                                                        pmicRegAddr,
                                                        pRxBuf,
                                                        buffLength);
    }

    return pmicStatus;
}

/*
 * \brief  Function to Copy I2C  data to crcData for I2C Interface
 */
static int32_t Pmic_commIoStoreI2cCrcData(
                                       const Pmic_CoreHandle_t *pPmicCoreHandle,
                                       uint16_t                 pmicRegAddr,
                                       uint8_t                  buffLength,
                                       uint8_t                  instType,
                                       const uint8_t           *pRxBuf)
{
    int32_t  pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  crcDataLen = 0U;
    uint8_t  crcData[PMIC_IO_BUF_SIZE] = {0};

    if(PMIC_NVM_INST == instType)
    {
        /* Store the slave address and I2C write request bit */
        crcData[crcDataLen] = (pPmicCoreHandle->nvmSlaveAddr << 1U);
        crcDataLen++;
    }
    else if(PMIC_QA_INST == instType)
    {
        /* Store the slave address and I2C write request bit */
        crcData[crcDataLen] = (pPmicCoreHandle->qaSlaveAddr << 1U);
        crcDataLen++;
    }
    else
    {
        /* Store the slave address and I2C write request bit */
        crcData[crcDataLen] = (pPmicCoreHandle->slaveAddr << 1U);
        crcDataLen++;
    }

    /* Store the Register address */
    crcData[crcDataLen] = (uint8_t)pmicRegAddr;
    crcDataLen++;

    if(PMIC_NVM_INST == instType)
    {
        /* Store the slave address and I2C Read request bit */
        crcData[crcDataLen] = (pPmicCoreHandle->nvmSlaveAddr << 1U) |
                              (PMIC_IO_READ);
        crcDataLen++;
    }
    else if(PMIC_QA_INST == instType)
    {
        /* Store the slave address and I2C Read request bit */
        crcData[crcDataLen] = (pPmicCoreHandle->qaSlaveAddr << 1U) |
                              (PMIC_IO_READ);
        crcDataLen++;
    }
    else
    {
        /* Store the slave address and I2C Read request bit */
        crcData[crcDataLen] = (pPmicCoreHandle->slaveAddr << 1U) |
                              (PMIC_IO_READ);
        crcDataLen++;
    }
    /* Store the data read */
    crcData[crcDataLen] = pRxBuf[buffLength - 2U];
    crcDataLen++;

    if((pRxBuf[buffLength - 1U]) != (Pmic_getCRC8Val(crcData, crcDataLen)))
    {
        pmicStatus = PMIC_ST_ERR_DATA_IO_CRC;
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
                               uint16_t           regAddr,
                               uint8_t           *pRxBuffer)
{
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t  buffLength = 1U;
    uint8_t  rxBuf[PMIC_IO_BUF_SIZE]   = {0};
    uint8_t  crcData[PMIC_IO_BUF_SIZE] = {0};
    uint8_t  instType   = PMIC_MAIN_INST;
    uint8_t  crcDataLen = 0U;
    uint16_t pmicRegAddr = regAddr;

    pmicStatus = Pmic_commIoReadData(pPmicCoreHandle,
                                     &pmicRegAddr,
                                     &buffLength,
                                     rxBuf,
                                     &instType);

    if((PMIC_ST_SUCCESS == pmicStatus) &&
       (((bool)true) == pPmicCoreHandle->crcEnable))
    {
        if(PMIC_INTF_SPI == pPmicCoreHandle->commMode)
        {
            /* Copy SPI frame data to crcData */
            for(crcDataLen = 0U;
                crcDataLen < (PMIC_IO_BUF_SIZE - 1U);
                crcDataLen++)
            {
                crcData[crcDataLen] = rxBuf[crcDataLen];
            }

            if((rxBuf[buffLength - 1U]) !=
                                         (Pmic_getCRC8Val(crcData, crcDataLen)))
            {
                pmicStatus = PMIC_ST_ERR_DATA_IO_CRC;
            }
        }
        else if((PMIC_INTF_SINGLE_I2C == pPmicCoreHandle->commMode) ||
                (PMIC_INTF_DUAL_I2C   == pPmicCoreHandle->commMode))
        {
            pmicStatus = Pmic_commIoStoreI2cCrcData(pPmicCoreHandle,
                                                    pmicRegAddr,
                                                    buffLength,
                                                    instType,
                                                    rxBuf);
        }
        else
        {
            pmicStatus = PMIC_ST_ERR_INV_PARAM;
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
