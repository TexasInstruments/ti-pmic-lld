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
#ifndef TIVA_GPIO_H
#define TIVA_GPIO_H

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct gpioPinHandle_s
{
    uint32_t sysPeriphGPIO;
    uint32_t gpioPortBase;
    uint32_t gpioPin;
    uint32_t gpioPinDir;
    uint32_t gpioShiftVal;
}gpioPinHandle_t;

/**
 * \brief Initializes an array of GPIO pin handles
 *
 * \param gpioPinHandle     [OUT]    Reference to array of handles to GPIO pins
 * \param bOutput           [IN]     This parameter set as true will configure every handle to be handles to output
 *                                   pins. Otherwise, it will configure every handle to be handles to input pins
 */
void initializeGpioPinHandles(gpioPinHandle_t *gpioPinHandle, bool bOutput);

/**
 * \brief Initialize all GPIO for use as input pins
 *
 * \param gpioPinHandle     [IN]    array of handles used to configure Tiva GPIO pins as input/output
 */
void initializeGpioPins(const gpioPinHandle_t *gpioPinHandle);

/**
 * \brief Initialize a GPIO for use as an input or output pin. Whether GPIO is input or output
 *        is dependent on the gpioPinDir struct member of gpioPinHandle parameter
 *
 * \param gpioPinHandle     [IN]    Handle to a GPIO pin
 */
void initializeGpioPin(const gpioPinHandle_t gpioPinHandle);

/**
 * \brief Initialize a GPIO handle for use as a handle to an MCU pin interfacing the PMIC nERR_MCU pin.
 *        Target pin will be PC6 on the Tiva.
 *
 * \param gpioPinHandle     [OUT]    GPIO pin handle to initialize
 */
void initializeEsmGpioOutputHandle(gpioPinHandle_t *gpioPinHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_GPIO_H */
