#include "tiva_priv.h"
#include "tiva_timer.h"

int32_t initializeTimerHandle(timerHandle_t *timerHandle)
{
    if (timerHandle == NULL)
        return INVALID_INPUT_PARAM;

    timerHandle->sysctlPeriphTimer = SYSCTL_PERIPH_TIMER0;
    timerHandle->timerBase         = TIMER0_BASE;

    return SUCCESS;
}

int32_t initializeTimer(timerHandle_t *timerHandle)
{
    if (timerHandle == NULL)
        return INVALID_INPUT_PARAM;

    // Enable the target timer peripheral
    SysCtlPeripheralEnable(timerHandle->sysctlPeriphTimer);

    // Ensure timer is ready to be configured
    while (!SysCtlPeripheralReady(timerHandle->sysctlPeriphTimer))
    {
    }

    // Disable timer before configuration
    TimerDisable(timerHandle->timerBase, TIMER_BOTH);

    // Write to configuration register the value 0x0000.0000
    TimerConfigure(timerHandle->timerBase, 0x0);

    // Configure the timer to be a full-width periodic timer with a .20s period
    TimerConfigure(timerHandle->timerBase, TIMER_CFG_PERIODIC);
    TimerLoadSet(timerHandle->timerBase, TIMER_A, SysCtlClockGet() / 5); // Channel A controls the module

    // Enable timer after configuration
    TimerEnable(timerHandle->timerBase, TIMER_A);

    return SUCCESS;
}
