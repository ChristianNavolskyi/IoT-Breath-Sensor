#include "Board.h"

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Task.h>

#include "smartrf_settings/smartrf_settings.h"

#include <xdc/runtime/System.h>


/* Packet TX Configuration */
#define PAYLOAD_LENGTH      5 //30
#define PACKET_INTERVAL     (uint32_t)(40000*0.5f) /* Set packet interval to 500ms */
#define TASKSTACKSIZE       (1024)
#define RF_UART_WRITE_BUFFER_SIZE (100)

/***** RF Params and Variables *****/
Task_Struct rfTask;
Char rfTaskStack[TASKSTACKSIZE];

static RF_Object rfObject;
static RF_Handle rfHandle;
static uint8_t packet[PAYLOAD_LENGTH];
uint32_t time = 0;

// Variables for thread communication
static uint32_t *resultPtr;
static uint8_t *resultPresentFlag;

UART_Handle rfUart;
uint_fast16_t rfUartOutputBufferSize = 0;
char rfUartWriteBuffer[RF_UART_WRITE_BUFFER_SIZE];


unsigned createMask(unsigned a, unsigned b) {
   unsigned r = 0; unsigned i = 0;
   for (i=a; i<=b; i++)
       r |= 1 << i;

   return r;
}

void sendDataViaRf() {
    rfUartOutputBufferSize = System_sprintf(rfUartWriteBuffer, "RF: Sending value with memcpy: %d\r\n", *resultPtr);
    UART_write(rfUart, rfUartWriteBuffer, rfUartOutputBufferSize + 1);

    time += PACKET_INTERVAL;
    RF_cmdPropTx.startTime = time;

    memcpy(packet, resultPtr, sizeof(*resultPtr));

    RF_runCmd(rfHandle, (RF_Op*) &RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
}

void send(uint32_t value) {
    rfUartOutputBufferSize = System_sprintf(rfUartWriteBuffer, "RF: Sending value with shifting: %d\r\n", *resultPtr);
    UART_write(rfUart, rfUartWriteBuffer, rfUartOutputBufferSize + 1);

    // Set frequency
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */
    time = RF_getCurrentTime();

    uint32_t firstByte, secondByte, thirdByte, fourthByte;
    firstByte = createMask(0,7);
    secondByte = createMask(8,15);
    thirdByte = createMask(16,23);
    fourthByte = createMask(24,31);

    packet[0] = time ;
    packet[1] = firstByte & value ;
    packet[2] = secondByte & value >> 8;
    packet[3] = thirdByte & value >> 16;
    packet[4] = fourthByte & value >> 24;

    /* Set absolute TX time to utilize automatic power management */
    time += PACKET_INTERVAL;
    RF_cmdPropTx.startTime = time;

    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
}

void initialiseTransmissionParameters() {
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    if (rfHandle == NULL) {
        rfUartOutputBufferSize = System_sprintf(rfUartWriteBuffer, "RF: Ending\r\n");
        UART_write(rfUart, rfUartWriteBuffer, rfUartOutputBufferSize + 1);
    }
}

void waitUntilValueIsPresent() {
    rfUartOutputBufferSize = System_sprintf(rfUartWriteBuffer, "RF: Waiting\r\n");
    UART_write(rfUart, rfUartWriteBuffer, rfUartOutputBufferSize + 1);

    while (*resultPresentFlag == 0);
}

void sendValueAndSetFlag() {
    sendDataViaRf();
    *resultPresentFlag = 0;
}

void transmittValue() {
    waitUntilValueIsPresent();
    sendValueAndSetFlag();
    transmittValue();
}

void setupRFUart() {
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    rfUart = UART_open(Board_UART, &uartParams);

    if (rfUart == NULL) {
        /* UART_open() failed */
        System_abort("RF: UART could not be opened.");
    }
}

void *startRFTransmission(void* arg0) {
    setupRFUart();

    rfUartOutputBufferSize = System_sprintf(rfUartWriteBuffer, "RF: startRFTransmission\r\n");
    UART_write(rfUart, rfUartWriteBuffer, rfUartOutputBufferSize + 1);

    initialiseTransmissionParameters();
    transmittValue();
}

void startRFTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag) {
    resultPtr = adcValuePtr;
    resultPresentFlag = adcValuePresentFlag;

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &rfTaskStack;

    Task_construct(&rfTask, (Task_FuncPtr) startRFTransmission, &taskParams, NULL);
}
