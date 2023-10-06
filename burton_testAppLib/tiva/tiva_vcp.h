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
    uint32_t gpioTxPin;
    uint32_t gpioRxPin;
    uint32_t TxPinToUART;
    uint32_t RxPinToUART;
    uint32_t clkSrc;
    uint32_t clkSrcFreq;
    uint32_t baudRate;
} uartHandle_t;

int32_t initializeVCP(uartHandle_t *vcpHandle);
int32_t initializeVCPHandle(uartHandle_t *vcpHandle);
int32_t UARTStrPut(uartHandle_t *UARTHandle, uint8_t *str);
int32_t UARTInt32Put(uartHandle_t *UARTHandle, int32_t num);
int32_t UARTUint32Put(uartHandle_t *UARTHandle, uint32_t num);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_VCP_H_ */
