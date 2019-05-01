#ifndef RFSENDER_H_
#define RFSENDER_H_

#include <ti/drivers/UART.h>


void startRFTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag);

#endif /* RFSENDER_H_ */
