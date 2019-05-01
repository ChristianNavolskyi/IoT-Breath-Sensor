#include "Board.h"
#include "stdio.h"

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <xdc/runtime/System.h>


#define TASKSTACKSIZE    (1024)
#define ADCBUFFERSIZE    (1)
#define ADC_UART_WRITE_BUFFER_SIZE (100)

/***** ADC Params and Variables *****/
Task_Struct adcTask;
Char adcTaskStack[TASKSTACKSIZE];

ADCBuf_Handle adcBuf;
ADCBuf_Conversion continuousConversionChannel;
uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];

// Variables for thread communication
static uint32_t *resultPtr;
static uint8_t *resultPresentFlag;

UART_Handle adcUart;


void print(const char* string, ...) {
    va_list args;
    va_start(args, string);

    uint_fast16_t size = 0;
    char buffer[ADC_UART_WRITE_BUFFER_SIZE];

    size = System_sprintf(buffer, string, args);
    UART_write(adcUart, buffer, size + 1);

    va_end(args);
}


void waitUntilRFTransmitterHasReadValue() {
    print("ADC: Start waiting\r\n");

    //while(*resultPresentFlag == 1);

    print("ADC: Waiting finished\r\n");
}

void setValueAndFlagForRFTransmitter(uint32_t value) {
    *resultPtr = value;
    *resultPresentFlag = 1;

    print("ADC: Setting value: %d\r\n", *resultPtr);
}

void startNextADCConversion() {
    if (ADCBuf_convert(adcBuf, &continuousConversionChannel, 1) != ADCBuf_STATUS_SUCCESS) {
        System_abort("Did not start conversion process correctly\n");
    }
}

void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel) {
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    print("ADC: Received value: %d\r\n", microVoltBuffer[0]);

    waitUntilRFTransmitterHasReadValue();
    setValueAndFlagForRFTransmitter(microVoltBuffer[0]);
    startNextADCConversion();
}

void configureConversionChannel() {
    continuousConversionChannel.arg = NULL;
    continuousConversionChannel.adcChannel = CC1350_LAUNCHXL_ADCVDDS;
    continuousConversionChannel.sampleBuffer = sampleBufferOne;
    continuousConversionChannel.sampleBufferTwo = sampleBufferTwo;
    continuousConversionChannel.samplesRequestedCount = ADCBUFFERSIZE;
}

void setupADCBufAndParams(ADCBuf_Params *adcBufParams) {
    ADCBuf_Params_init(adcBufParams);
    adcBufParams->callbackFxn = adcBufCallback;
    adcBufParams->recurrenceMode = ADCBuf_RECURRENCE_MODE_ONE_SHOT;
    adcBufParams->returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams->samplingFrequency = 10;
}

void setupADCUart() {
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.writeMode = UART_MODE_CALLBACK;
    uartParams.baudRate = 115200;

    adcUart = UART_open(Board_UART, &uartParams);

    if (adcUart == NULL) {
        /* UART_open() failed */
        System_abort("ADC: UART could not be opened.");
    }
}

void *startADCConversion(void* arg0) {
    setupADCUart();

    ADCBuf_Params adcBufParams;

    setupADCBufAndParams(&adcBufParams);
    adcBuf = ADCBuf_open(Board_ADCBuf0, &adcBufParams);

    configureConversionChannel();

    if (!adcBuf){
        print("ADC: Ending ADC\r\n");

        System_abort("adcBuf did not open correctly\n");
    }

    startNextADCConversion();
}

void startADCTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag) {
    resultPtr = adcValuePtr;
    resultPresentFlag = adcValuePresentFlag;

//    while (1) {
//        const unsigned char hello[] = "Hello World\n";
//
//        UART_write(adcUart, hello, sizeof(hello));
//    }

    Board_initADCBuf();

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &adcTaskStack;

    Task_construct(&adcTask, (Task_FuncPtr) startADCConversion, &taskParams, NULL);
}
