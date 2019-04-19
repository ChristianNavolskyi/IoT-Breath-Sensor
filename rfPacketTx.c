/*
 * Copyright (c) 2015-2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <stdint.h>
#include <string.h>

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/UART.h>

/* Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include "Board.h"

#include "smartrf_settings/smartrf_settings.h"

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};


/***** Defines *****/
#define TX_TASK_STACK_SIZE 1024
#define TX_TASK_PRIORITY   2

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      30
#define PACKET_INTERVAL     (uint32_t)(4000000*0.5f) /* Set packet interval to 500ms */

#define TASKSTACKSIZE    (768)
#define ADCBUFFERSIZE    (1)
#define UART_WRITE_BUFFER_SIZE (10)
#define UART_READ_BUFFER_SIZE (3)


Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];
uint32_t buffersCompletedCounter = 0;
uint_fast16_t uartOutputBufferSize = 0;

char uartWriteBuffer[UART_WRITE_BUFFER_SIZE];
char uartReadBuffer[UART_READ_BUFFER_SIZE];
char endLineSequence[] = "end";

uint32_t time = 0;

/* Driver handle shared between the task and the callback function */
UART_Handle uart;


/*
 * This function is called whenever a buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 *
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion,
    void *completedADCBuffer, uint32_t completedChannel) {

    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    uartOutputBufferSize = System_sprintf(uartWriteBuffer, "%d,%d;", time++, microVoltBuffer[0]);
    UART_write(uart, uartWriteBuffer, uartOutputBufferSize + 1);
    UART_read(uart, uartReadBuffer, UART_READ_BUFFER_SIZE);
}

void uartWriteCallback(UART_Handle handle, void *buf, size_t count) {
    return;
}

void uartReadCallback(UART_Handle handle, void *buf, size_t count) {
    int comparissonResult = strcmp(endLineSequence, buf);

    if (comparissonResult) {
        return;
    }

    uartOutputBufferSize = System_sprintf(uartWriteBuffer, "\r\n");
    UART_write(handle, uartWriteBuffer, uartOutputBufferSize + 1);
}

/*
 *  ======== conversionStartFxn ========
 *  Task for this function is created statically. See the project's .cfg file.
 */
void conversionStartFxn(UArg arg0, UArg arg1) {
    UART_Params uartParams;
    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion continuousConversionChannel;

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.writeMode = UART_MODE_CALLBACK;
    uartParams.writeCallback = uartWriteCallback;
    uartParams.readDataMode = UART_DATA_TEXT;
    uartParams.readMode = UART_MODE_CALLBACK;
    uartParams.readCallback = uartReadCallback;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    ADCBuf_Params_init(&adcBufParams);
    adcBufParams.callbackFxn = adcBufCallback;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams.samplingFrequency = 10;
    adcBuf = ADCBuf_open(Board_ADCBuf0, &adcBufParams);

    /* Configure the conversion struct */
    continuousConversionChannel.arg = NULL;
    continuousConversionChannel.adcChannel = CC1350_LAUNCHXL_ADCVDDS;
    continuousConversionChannel.sampleBuffer = sampleBufferOne;
    continuousConversionChannel.sampleBufferTwo = sampleBufferTwo;
    continuousConversionChannel.samplesRequestedCount = ADCBUFFERSIZE;

    if (!adcBuf){
        System_abort("adcBuf did not open correctly\n");
    }

    /* Start converting. */
    if (ADCBuf_convert(adcBuf, &continuousConversionChannel, 1) != ADCBuf_STATUS_SUCCESS) {
        System_abort("Did not start conversion process correctly\n");
    }

    /*
     * Go to sleep in the foreground thread forever. The data will be collected
     * and transfered in the background thread
     */
    Task_sleep(BIOS_WAIT_FOREVER);
}


/***** Prototypes *****/
static void txTaskFunction(UArg arg0, UArg arg1);



/***** Variable declarations *****/
static Task_Params txTaskParams;
Task_Struct txTask;    /* not static so you can see in ROV */
static uint8_t txTaskStack[TX_TASK_STACK_SIZE];

static RF_Object rfObject;
static RF_Handle rfHandle;

uint32_t time;
static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;
static PIN_Handle pinHandle;


/***** Function definitions *****/
void TxTask_init(PIN_Handle inPinHandle)
{
    pinHandle = inPinHandle;

    Task_Params_init(&txTaskParams);
    txTaskParams.stackSize = TX_TASK_STACK_SIZE;
    txTaskParams.priority = TX_TASK_PRIORITY;
    txTaskParams.stack = &txTaskStack;
    txTaskParams.arg0 = (UInt)1000000;

    Task_construct(&txTask, txTaskFunction, &txTaskParams, NULL);
}

static void txTaskFunction(UArg arg0, UArg arg1)
{
    uint32_t time;
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_ABSTIME;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */
    time = RF_getCurrentTime();
    while(1)
    {
        /* Create packet with incrementing sequence number and random payload */
        uint8_t i;
        for (i = 0; i < PAYLOAD_LENGTH; i++)
        {
            packet[i] = rand();
        }

        /* Set absolute TX time to utilize automatic power management */
        time += PACKET_INTERVAL;
        RF_cmdPropTx.startTime = time;

        /* Send packet */
        RF_EventMask result = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
        if (!(result & RF_EventLastCmdDone))
        {
            /* Error */
            while(1);
        }

        PIN_setOutputValue(pinHandle, Board_LED1,!PIN_getOutputValue(Board_LED1));
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions. */
    Board_initGeneral();
    Board_initADCBuf();
    Board_initUART();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if(!ledPinHandle)
    {
        System_abort("Error initializing board LED pins\n");
    }




    /* Construct BIOS objects */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr) conversionStartFxn,
        &taskParams, NULL);

    System_printf("Starting the ADC Continuous Sampling example\n"
        "System provider is set to SysMin. Halt the target to view any SysMin "
        "contents in ROV.\n");

    /* Initialize task */
    TxTask_init(ledPinHandle);

    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
