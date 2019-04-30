#include "Board.h"

#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include <xdc/runtime/System.h>


#define TASKSTACKSIZE    (768)
#define ADCBUFFERSIZE    (1)
#define UART_WRITE_BUFFER_SIZE (30)

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

uint_fast16_t uartOutputBufferSize = 0;
static UART_Handle uart;
char uartWriteBuffer[UART_WRITE_BUFFER_SIZE];

void waitUntilRFTransmitterHasReadValue() {
    while(*resultPresentFlag == 1);
}

void setValueAndFlagForRFTransmitter(uint32_t value) {
    *resultPtr = value;
    *resultPresentFlag = 1;
}

void startNextADCConversion() {
    if (ADCBuf_convert(adcBuf, &continuousConversionChannel, 1) != ADCBuf_STATUS_SUCCESS) {
        System_abort("Did not start conversion process correctly\n");
    }
}

void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel) {
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    uartOutputBufferSize = System_sprintf(uartWriteBuffer, "Received value: $d\r\n", microVoltBuffer[0]);
    UART_write(uart, uartWriteBuffer, uartOutputBufferSize + 1);

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

void startADCConversion(UArg arg0, UArg arg1) {
    uartOutputBufferSize = System_sprintf(uartWriteBuffer, "Start ADC conversion\r\n");
    UART_write(uart, uartWriteBuffer, uartOutputBufferSize + 1);

    ADCBuf_Params adcBufParams;

    setupADCBufAndParams(&adcBufParams);
    adcBuf = ADCBuf_open(Board_ADCBuf0, &adcBufParams);

    configureConversionChannel();

    if (!adcBuf){
        System_abort("adcBuf did not open correctly\n");
    }

    startNextADCConversion();

    /*
     * Go to sleep in the foreground thread forever. The data will be collected
     * and transfered in the background thread
     */
    Task_sleep(BIOS_WAIT_FOREVER);
}

void startADCTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag, UART_Handle uartHandle) {
    resultPtr = adcValuePtr;
    resultPresentFlag = adcValuePresentFlag;
    uart = uartHandle;

    Board_initADCBuf();

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &adcTaskStack;

    uartOutputBufferSize = System_sprintf(uartWriteBuffer, "Construct ADC task\r\n");
    UART_write(uart, uartWriteBuffer, uartOutputBufferSize + 1);

    Task_construct(&adcTask, (Task_FuncPtr) startADCConversion, &taskParams, NULL);
}
