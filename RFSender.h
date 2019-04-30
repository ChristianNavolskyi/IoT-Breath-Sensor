#include <ti/drivers/UART.h>

#ifndef RFSENDER_H_
#define RFSENDER_H_

void startRFTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag, UART_Handle uartHandle);

#endif /* RFSENDER_H_ */
