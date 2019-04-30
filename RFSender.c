#include "Board.h"

#include <ti/drivers/rf/RF.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Task.h>

#include "smartrf_settings/smartrf_settings.h"

#include <xdc/runtime/System.h>


/* Packet TX Configuration */
#define PAYLOAD_LENGTH      4 //30
#define PACKET_INTERVAL     (uint32_t)(40000*0.5f) /* Set packet interval to 500ms */
#define TASKSTACKSIZE       (768)
#define UART_WRITE_BUFFER_SIZE (30)

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

uint_fast16_t rfUartOutputBufferSize = 0;
static UART_Handle uart;
char uartWriteBuffer[UART_WRITE_BUFFER_SIZE];


unsigned createMask(unsigned a, unsigned b) {
   unsigned r = 0; unsigned i = 0;
   for (i=a; i<=b; i++)
       r |= 1 << i;

   return r;
}

void sendDataViaRf() {
    rfUartOutputBufferSize = System_sprintf(uartWriteBuffer, "Sending value: $d\r\n", *resultPtr);
    UART_write(uart, uartWriteBuffer, rfUartOutputBufferSize + 1);

    time += PACKET_INTERVAL;
    RF_cmdPropTx.startTime = time;

    memcpy(packet, resultPtr, sizeof(*resultPtr));

    RF_runCmd(rfHandle, (RF_Op*) &RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
}

void send(uint32_t value) {
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
}

void waitUntilValueIsPresent() {
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

void startRFTransmission(UArg arg0, UArg arg1) {
    rfUartOutputBufferSize = System_sprintf(uartWriteBuffer, "Starting RF transmission\r\n");
    UART_write(uart, uartWriteBuffer, rfUartOutputBufferSize + 1);

    initialiseTransmissionParameters();
    transmittValue();
}

void startRFTask(uint32_t *adcValuePtr, uint8_t *adcValuePresentFlag, UART_Handle uartHandle) {
    rfUartOutputBufferSize = System_sprintf(uartWriteBuffer, "Construct RF task\r\n");
    UART_write(uartHandle, uartWriteBuffer, rfUartOutputBufferSize + 1);

    resultPtr = adcValuePtr;
    resultPresentFlag = adcValuePresentFlag;
    uart = uartHandle;

    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &rfTaskStack;

    Task_construct(&rfTask, (Task_FuncPtr) startRFTransmission, &taskParams, NULL);
}
