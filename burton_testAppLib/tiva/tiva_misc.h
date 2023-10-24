#ifndef TIVA_MISC_H_
#define TIVA_MISC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * \brief Convert a uint32_t to binary, represented as a string
 *
 * \param num   [IN]        Number to convert to binary
 *
 * \return      uint8_t*    Address of the first non-null character in the binary string
 */
uint8_t *uint32ToBinaryStr(uint32_t num);

/**
 * \brief Convert a uint32_t to hex, represented as a string
 *
 * \param num   [IN]        Number to convert to hex
 *
 * \return      uint8_t*    Address of the first non-null character in the hex string
 */
uint8_t *uint32toHexStr(uint32_t num);

/**
 * \brief This function is generally called at the beginning of a test application
 *        and is used to clear the terminal.
 *
 * \param uartHandle [IN] Handle to the UART that is interfacing the virtual communication port
 */
void clearConsole(uartHandle_t *uartHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_MISC_H_ */
