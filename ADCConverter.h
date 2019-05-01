#ifndef ADCCONVERTER_H_
#define ADCCONVERTER_H_

#include <ti/drivers/UART.h>


void startADCTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag);

#endif /* ADCCONVERTER_H_ */
