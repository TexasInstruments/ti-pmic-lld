#ifndef TIVA_MISC_H_
#define TIVA_MISC_H_

#ifdef __cplusplus
extern "C"
{
#endif

int32_t convertUint32toHexStr(uint8_t **str, uint32_t num);
int32_t convertUint32toBinaryStr(uint8_t **str, uint32_t num);
void    clearConsole(uartHandle_t *uartHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */

#endif /* TIVA_MISC_H_ */
