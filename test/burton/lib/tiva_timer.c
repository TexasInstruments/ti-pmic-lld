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
