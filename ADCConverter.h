#include <ti/drivers/UART.h>

#ifndef ADCCONVERTER_H_
#define ADCCONVERTER_H_

void startADCTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag, UART_Handle uartHandle);

#endif /* ADCCONVERTER_H_ */
