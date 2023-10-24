#include "tiva_priv.h"
#include "tiva_misc.h"

/**
 * \brief Convert a uint32_t to binary, represented as a string
 *
 * \param num   [IN]        Number to convert to binary
 *
 * \return      uint8_t*    Address of the first non-null character in the binary string
 */
uint8_t *uint32ToBinaryStr(uint32_t num)
{
    // Variable declaration/initialization
    uint8_t        i = 0;
    uint32_t       bit = 0;
    static uint8_t binaryStr[33]; // last index is for null terminator

    // Reset variables
    for (i = 0; i < 33; i++)
    {
        binaryStr[i] = '\0';
    }

    // Account for case where num == 0
    i = 0;
    if (num == 0)
    {
        binaryStr[i] = '0';
        return binaryStr;
    }

    // Convert num to binary (represented as a string)
    while (num > 0)
    {
        // Get next least significant bit
        bit = num & 0b1;

        // Store into next last position of binaryStr
        binaryStr[31 - i] = '0' + bit;

        i++;
        num >>= 0b1;
    }

    // Return the location that's holding the first non-NULL value in binaryStr
    i--;
    return (binaryStr + (31 - i));
}

/**
 * \brief Convert a uint32_t to hex, represented as a string
 *
 * \param num   [IN]        Number to convert to hex
 *
 * \return      uint8_t*    Address of the first non-null character in the hex string
 */
uint8_t *uint32toHexStr(uint32_t num)
{
    // Variable declaration/initialization
    uint8_t        i = 0;
    uint32_t       hexDigit = 0;
    static uint8_t hexStr[9]; // last index reserved for NULL terminator

    // Reset variables
    for (i = 0; i < 9; i++)
    {
        hexStr[i] = '\0';
    }

    // Account for case where num == 0
    i = 0;
    if (num == 0)
    {
        hexStr[i] = '0';
        return hexStr;
    }

    // Convert num to hex (represented as a string)
    while (num > 0)
    {
        // Obtain next most least significant hex digit
        hexDigit = num & 0xF;

        // Store the ASCII representation of hex digit in next last spot of hexStr
        hexStr[7 - i] = (hexDigit > 9) ? ('A' + (hexDigit - 10)) : ('0' + hexDigit);

        i++;
        num >>= 4;
    }

    // return the location that's holding the first non-NULL value in hexStr
    i--;
    return (hexStr + (7 - i));
}

/**
 * \brief This function is generally called at the beginning of a test application
 *        and is used to clear the terminal.
 *
 * \param uartHandle [IN] Handle to the UART that is interfacing the virtual communication port
 */
void clearConsole(uartHandle_t *uartHandle)
{
    if (uartHandle == NULL)
        return;

    UARTStrPut(uartHandle, "\033[2J");
    UARTStrPut(uartHandle, "\033[1;1H");
}
