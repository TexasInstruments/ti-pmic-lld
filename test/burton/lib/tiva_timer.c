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
#include "tiva_timer.h"

void initializeTimerHandle(timerHandle_t *timerHandle)
{
    timerHandle->sysctlPeriphTimer = SYSCTL_PERIPH_TIMER0;
    timerHandle->timerBase = TIMER0_BASE;
}

void initializeTimer(const timerHandle_t *timerHandle)
{
    // Enable the target timer peripheral
    SysCtlPeripheralEnable(timerHandle->sysctlPeriphTimer);

    // Ensure timer is ready to be configured
    while (!SysCtlPeripheralReady(timerHandle->sysctlPeriphTimer))
    {
    }

    // Ensure timer is disabled so that other Timer APIs could configure the timer module
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
}

void delayTimeInMs(const timerHandle_t *timerHandle, uint16_t milliseconds)
{
    uint32_t cycles = 0;

    /*** Delay for specified number of seconds ***/

    // Disable timer before configuration
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);

    // Configure timer to be in full-width Periodic mode counting down
    TimerConfigure(timerHandle->timerBase, 0x0);
    TimerConfigure(timerHandle->timerBase, TIMER_CFG_PERIODIC);

    // Configure timer to generate a 1 second period
    TimerLoadSet(timerHandle->timerBase, TIMER_A, SysCtlClockGet());

    // Clear any pending timer interrupt
    TimerIntClear(timerHandle->timerBase, TIMER_TIMA_TIMEOUT);

    // Enable timer and start counting
    TimerEnable(timerHandle->timerBase, TIMER_A);

    while (milliseconds >= 1000)
    {
        // Wait for 1 second
        while (TimerIntStatus(timerHandle->timerBase, false) != TIMER_TIMA_TIMEOUT)
        {
        }

        // Clear timeout IRQ and decrement milliseconds
        TimerIntClear(timerHandle->timerBase, TIMER_TIMA_TIMEOUT);
        milliseconds -= 1000;
    }

    // Disable timer after target duration has been met
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);

    /*** Delay for specified fraction of a second ***/

    // Configure timer to be in full-width One-Shot mode counting down
    TimerConfigure(timerHandle->timerBase, 0x0);
    TimerConfigure(timerHandle->timerBase, TIMER_CFG_ONE_SHOT);

    cycles = (uint32_t)((SysCtlClockGet() / 1000) * milliseconds);
    TimerLoadSet(timerHandle->timerBase, TIMER_A, cycles);

    // Clear any pending timer interrupt
    TimerIntClear(timerHandle->timerBase, TIMER_TIMA_TIMEOUT);

    // Enable timer and start counting
    TimerEnable(timerHandle->timerBase, TIMER_A);

    // Wait the fraction of a second
    while (TimerIntStatus(timerHandle->timerBase, false) != TIMER_TIMA_TIMEOUT)
    {
    }

    // Clear flag
    TimerIntClear(timerHandle->timerBase, TIMER_TIMA_TIMEOUT);

    // Disable timer after target duration has been met
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
}
