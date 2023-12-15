#include "tiva_priv.h"
#include "tiva_timer.h"

void initializeTimerHandle(timerHandle_t *timerHandle)
{
    timerHandle->sysctlPeriphTimer = SYSCTL_PERIPH_TIMER0;
    timerHandle->timerBase = TIMER0_BASE;
}

void initializeTimer(timerHandle_t *timerHandle)
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

void delayTimeInMs(timerHandle_t *timerHandle, uint16_t milliseconds)
{
    uint32_t cycles = 0;
    uint32_t seconds = 0;

    seconds = milliseconds / 1000U;
    milliseconds %= 1000U;

    /*** Delay for specified fraction of a second ***/

    if (milliseconds != 0)
    {
        // Disable timer before configuration
        TimerDisable(timerHandle->timerBase, TIMER_BOTH);

        // Configure timer to be in full-width One-Shot mode counting down
        TimerConfigure(timerHandle->timerBase, 0x0);
        TimerConfigure(timerHandle->timerBase, TIMER_CFG_ONE_SHOT);

        cycles = (uint32_t)(((uint64_t)SysCtlClockGet() * (uint64_t)milliseconds) / (uint64_t)1000U);
        TimerLoadSet(timerHandle->timerBase, TIMER_A, cycles); // In full-width CH. A controls timer

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
    }

    /*** Delay for specified number of seconds ***/

    if (seconds != 0)
    {
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

        // Delay for specified seconds
        while (seconds != 0)
        {
            while (TimerIntStatus(timerHandle->timerBase, false) != TIMER_TIMA_TIMEOUT)
            {
            }

            TimerIntClear(timerHandle->timerBase, TIMER_TIMA_TIMEOUT);
            seconds--;
        }
    }

    // Disable timer after target duration has been met
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);
}
