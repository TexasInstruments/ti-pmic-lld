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
