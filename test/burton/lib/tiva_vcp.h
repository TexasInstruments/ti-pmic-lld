/******************************************************************************
 * Copyright (c) 2023-2024 Texas Instruments Incorporated - http://www.ti.com
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
#ifndef TIVA_VCP_H_
#define TIVA_VCP_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* UART handle struct definition */
typedef struct uartHandle_s
{
    uint32_t sysctlPeriphUART;
    uint32_t sysctlPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t uartBase;
    uint8_t gpioTxPin;
    uint8_t gpioRxPin;
    uint32_t TxPinToUART;
    uint32_t RxPinToUART;
    uint32_t clkSrc;
    uint32_t clkSrcFreq;
    uint32_t baudRate;
} uartHandle_t;

/**
 * \brief Function to initialize the UART that's interfacing the virtual communication port.
 *
 * \param vcpHandle [IN] Handle to the virtual communication port UART
 */
void initializeVCP(const uartHandle_t *vcpHandle);

/**
 * \brief Function to initialize the virtual communication port UART handle
 *
 * \param vcpHandle [OUT] Handle to the virtual communication port UART
 */
void initializeVCPHandle(uartHandle_t *vcpHandle);

/**
 * \brief Function to transmit a string via UART
 *
 * \param UARTHandle    [IN]        Handle to the UART module
 * \param str           [IN]        String to transmit via UART
 */
void UARTStrPut(const uartHandle_t *UARTHandle, const uint8_t *str);

/**
 * \brief Function to transmit an unsigned 32-bit integer via UART
 *
 * \param UARTHandle    [IN]        Handle to the UART module
 * \param num           [IN]        Number to transmit over UART
 */
void UARTUint32Put(const uartHandle_t *UARTHandle, uint32_t num);

/**
 * \brief Function to transmit a signed 32-bit integer via UART
 *
 * \param UARTHandle    [IN]        Handle to the UART module
 * \param num           [IN]        Number to transmit over UART
 */
void UARTInt32Put(const uartHandle_t *UARTHandle, int32_t num);

/**
 * \brief This function is generally called at the beginning of a test application
 *        and is used to clear the terminal.
 *
 * \param uartHandle [IN] Handle to the UART that is interfacing the virtual communication port
 */
void clearConsole(const uartHandle_t *uartHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_VCP_H_ */
