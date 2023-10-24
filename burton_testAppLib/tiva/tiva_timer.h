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

/**
 * \brief Function to initialize a timer handle.
 *
 * \param timerHandle [OUT] Handle to timer module
 */
void initializeTimerHandle(timerHandle_t *timerHandle);

/**
 * \brief Given a timer handle, this function initializes timer module 0 on the Tiva.
 *
 * \param timerHandle [IN] Handle to timer module
 */
void initializeTimer(timerHandle_t *timerHandle);

/**
 * \brief This function is used to generate a specified delay in milliseconds
 *
 * \param timerHandle   [IN]    Handle to timer module
 * \param milliseconds  [IN]    Duration of delay in milliseconds
 */
void delayTimeInMs(timerHandle_t *timerHandle, uint16_t milliseconds);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_TIMER_H_ */
