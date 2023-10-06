#ifndef TIVA_TIMER_H_
#define TIVA_TIMER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Timer handle struct definition */
typedef struct timerHandle_s
{
    uint32_t sysctlPeriphTimer;
    uint32_t timerBase;
} timerHandle_t;

int32_t initializeTimer(timerHandle_t *timerHandle);
int32_t initializeTimerHandle(timerHandle_t *timerHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_TIMER_H_ */
