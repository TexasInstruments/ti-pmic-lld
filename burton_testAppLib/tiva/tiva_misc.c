#include "tiva_priv.h"
#include "tiva_misc.h"

int32_t convertUint32toBinaryStr(uint8_t **str, uint32_t num)
{
    // Variable declaration/initialization
    uint8_t        i   = 0;
    uint32_t       bit = 0;
    static uint8_t binaryStr[33]; // last bit is for null terminator

    // Parameter check
    if (str == NULL)
        return INVALID_INPUT_PARAM;

    // Reset variables
    *str = NULL;
    for (i = 0; i < 33; i++)
    {
        binaryStr[i] = '\0';
    }

    // Account for case where num == 0
    i = 0;
    if (num == 0)
    {
        binaryStr[i] = '0';
        *str         = binaryStr;

        return SUCCESS;
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

    // Deference pointer to string pointer and make the string pointer point
    // to the location that's holding the first non-NULL value in binaryStr
    i--;
    *str = binaryStr + (31 - i);

    return SUCCESS;
}

int32_t convertUint32toHexStr(uint8_t **str, uint32_t num)
{
    // Variable declaration/initialization
    uint8_t        i        = 0;
    uint32_t       hexDigit = 0;
    static uint8_t hexStr[9]; // last bit reserved for NULL terminator

    // Parameter check
    if (str == NULL)
        return INVALID_INPUT_PARAM;

    // Reset variables
    *str = NULL;
    for (i = 0; i < 9; i++)
    {
        hexStr[i] = '\0';
    }

    // Account for case where num == 0
    i = 0;
    if (num == 0)
    {
        hexStr[i] = '0';
        *str      = hexStr;

        return SUCCESS;
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

    // Deference pointer to string pointer and make the string pointer point
    // to the location that's holding the first non-NULL value in hexStr
    i--;
    *str = hexStr + (7 - i);

    return SUCCESS;
}

void clearConsole(uartHandle_t *uartHandle)
{
    if (uartHandle == NULL)
        return;

    UARTStrPut(uartHandle, "\033[2J");
    UARTStrPut(uartHandle, "\033[1;1H");
}
