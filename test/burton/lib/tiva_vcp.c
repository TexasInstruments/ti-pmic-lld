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
#include "tiva_priv.h"
#include "tiva_vcp.h"

void initializeVCP(const uartHandle_t *vcpHandle)
{
    // Enable the UART module for PC <--> MCU communication
    SysCtlPeripheralEnable(vcpHandle->sysctlPeriphUART);

    // Enable UART TX and RX pins' GPIO port
    SysCtlPeripheralEnable(vcpHandle->sysctlPeriphGPIO);

    // Ensure VCP GPIO port is ready to be configured
    while (!SysCtlPeripheralReady(vcpHandle->sysctlPeriphGPIO))
    {
    }

    // Configure VCP GPIO TX and RX pins for UART operation
    GPIOPinTypeUART(vcpHandle->gpioPortBase, vcpHandle->gpioTxPin);
    GPIOPinTypeUART(vcpHandle->gpioPortBase, vcpHandle->gpioRxPin);

    // Configure VCP GPIO TX and RX pins to UART functionality
    GPIOPinConfigure(vcpHandle->TxPinToUART);
    GPIOPinConfigure(vcpHandle->RxPinToUART);

    // Ensure VCP UART is ready to be configured
    while (!SysCtlPeripheralReady(vcpHandle->sysctlPeriphUART))
    {
    }

    // Disable the UART before configuration
    UARTDisable(vcpHandle->uartBase);

    // Configure the UART to the popular configuration:
    // 9600 baud, 8 bit data, one stop bit, no parity
    // w/ input clock PIOSC (16,000,000 Hz)
    UARTConfigSetExpClk(vcpHandle->uartBase,
                        vcpHandle->clkSrcFreq,
                        vcpHandle->baudRate,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTClockSourceSet(vcpHandle->uartBase, vcpHandle->clkSrc);

    // Enable the UART after configuration
    UARTEnable(vcpHandle->uartBase);
}

void initializeVCPHandle(uartHandle_t *vcpHandle)
{
    vcpHandle->sysctlPeriphUART = SYSCTL_PERIPH_UART0;
    vcpHandle->sysctlPeriphGPIO = SYSCTL_PERIPH_GPIOA;
    vcpHandle->gpioPortBase = GPIO_PORTA_BASE;
    vcpHandle->uartBase = UART0_BASE;
    vcpHandle->gpioTxPin = GPIO_PIN_1;
    vcpHandle->gpioRxPin = GPIO_PIN_0;
    vcpHandle->TxPinToUART = GPIO_PA1_U0TX;
    vcpHandle->RxPinToUART = GPIO_PA0_U0RX;
    vcpHandle->clkSrc = UART_CLOCK_PIOSC;
    vcpHandle->clkSrcFreq = 16000000;
    vcpHandle->baudRate = 9600;
}

void UARTStrPut(const uartHandle_t *UARTHandle, const uint8_t *str)
{
    if ((str != NULL) && (*str != '\0') && (UARTHandle != NULL))
    {
        while (*str != '\0')
        {
            UARTCharPut(UARTHandle->uartBase, *str);
            str++;
        }
    }
}

void UARTUint32Put(const uartHandle_t *UARTHandle, uint32_t num)
{
    uint8_t len = 0, i, digit;
    uint8_t str[32] = {0};

    if (UARTHandle != NULL)
    {
        if (num == 0)
        {
            UARTCharPut(UARTHandle->uartBase, '0');
        }
        else
        {
            while (num > 0)
            {
                digit = num % 10;
                str[len] = '0' + digit;

                len++;
                num /= 10;
            }

            for (i = 0; i < len; i++)
            {
                UARTCharPut(UARTHandle->uartBase, str[len - 1U - i]);
            }
        }
    }
}

void UARTInt32Put(const uartHandle_t *UARTHandle, int32_t num)
{
    uint8_t len = 0, i, digit;
    uint8_t str[31] = {0};

    if (UARTHandle != NULL)
    {
        if (num == 0)
        {
            UARTCharPut(UARTHandle->uartBase, '0');
        }
        else
        {
            // If num is negative, print a negative sign and apply two's complement
            if (num < 0)
            {
                UARTCharPut(UARTHandle->uartBase, '-');
                num ^= 0xFFFFFFFF;
                num += 1;
            }

            while (num != 0)
            {
                digit = num % 10;
                str[len] = '0' + digit;

                len++;
                num /= 10;
            }

            for (i = 0; i < len; i++)
            {
                UARTCharPut(UARTHandle->uartBase, str[len - 1U - i]);
            }
        }
    }
}

void clearConsole(const uartHandle_t *uartHandle)
{
    if (uartHandle != NULL)
    {
        UARTStrPut(uartHandle, "\033[2J");
        UARTStrPut(uartHandle, "\033[1;1H");
    }
}
