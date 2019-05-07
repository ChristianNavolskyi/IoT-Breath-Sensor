/*
 * Copyright (c) 2016-2019, Texas Instruments Incorporated
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

/*
 *  ======== adcsinglechannel.c ========
 */
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/ADCBuf.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>
#include <xdc/runtime/System.h>

#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Example/Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"


/* UART */
#define IS_DEBUGGING true
#define UART_BUFFER_SIZE (50)
char adcUartBuffer[UART_BUFFER_SIZE];
char rfUartBuffer[UART_BUFFER_SIZE];

/* ADC properties */
#define ADCBUFFERSIZE    (1)
uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];

/* RF properties */
#define PAYLOAD_LENGTH      3
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;

/* Sleep */
static uint8_t rfSleepTime = 5;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#ifdef POWER_MEASUREMENT
#if defined(Board_CC1350_LAUNCHXL)
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#endif
    PIN_TERMINATE
};

// ADC value and flag for thread communication
// TODO message queue
volatile static uint32_t adcValue = 0;
volatile static uint8_t valueFlag = 0; // 0 - no value present, 1 - value present


void printMessageWithArg(UART_Handle uart, char uartBuffer[], char const *fmt, int num, ...) {
    if (IS_DEBUGGING) {
        va_list args;
        va_start(args, num);

        uint32_t value = va_arg(args, uint32_t);

        uint32_t bufferSize = System_sprintf(uartBuffer, fmt, value);
        UART_write(uart, uartBuffer, bufferSize + 1);

        va_end(args);
    }
}

void printMessage(UART_Handle uart, char uartBuffer[], char const *fmt) {
    if (IS_DEBUGGING) {
        uint32_t bufferSize = System_sprintf(uartBuffer, fmt);
        UART_write(uart, uartBuffer, bufferSize + 1);
    }
}

/*
 * This function is called whenever an ADC buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel) {
    /* Adjust raw ADC values and convert them to microvolts */
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    printMessageWithArg(*(UART_Handle *) conversion->arg, adcUartBuffer, "ADC: Read value: %d\r\n", 1, 5);
}

/* Read values from adc */
void *adcThreadFunc(void *arg0) {
    UART_Handle uart = *(UART_Handle *) arg0;
    printMessage(uart, adcUartBuffer, "ADC: Thread started\r\n");

    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion adcConversion;

    /* Call driver init functions */
    ADCBuf_init();

    /* Set up an ADCBuf peripheral in ADCBuf_RECURRENCE_MODE_CONTINUOUS */
    ADCBuf_Params_init(&adcBufParams);
    adcBufParams.callbackFxn = adcBufCallback;
    adcBufParams.recurrenceMode = ADCBuf_RECURRENCE_MODE_CONTINUOUS;
    adcBufParams.returnMode = ADCBuf_RETURN_MODE_CALLBACK;
    adcBufParams.samplingFrequency = 1;
    adcBuf = ADCBuf_open(Board_ADCBUF0, &adcBufParams);

    /* Configure the conversion struct */
    adcConversion.arg = &uart;
    adcConversion.adcChannel = CC1350_LAUNCHXL_ADCBUF0CHANNEL0;
    adcConversion.sampleBuffer = sampleBufferOne;
    adcConversion.sampleBufferTwo = sampleBufferTwo;
    adcConversion.samplesRequestedCount = ADCBUFFERSIZE;

    if (adcBuf == NULL){
        /* ADCBuf failed to open. */
        printMessage(uart, adcUartBuffer, "ADC: Could not open ADC port\r\n");
        while(1);
    }

    /* Start converting. */
    printMessage(uart, adcUartBuffer, "ADC: Starting first conversion\r\n");
    if (ADCBuf_convert(adcBuf, &adcConversion, 1) != ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        printMessage(uart, adcUartBuffer, "ADC: Could not start conversion\r\n");
        while(1);
    }

    while(1) {
        sleep(10000);
    }
}

void handleTransmissionResult(UART_Handle uart, RF_EventMask terminationReason) {
    printMessage(uart, rfUartBuffer, "RF: Evaluate termination reason\r\n");
    switch(terminationReason) {
        case RF_EventLastCmdDone:
         // A stand-alone radio operation command or the last radio
         // operation command in a chain finished.
         break;
        case RF_EventCmdCancelled:
         // Command cancelled before it was started; it can be caused
         // by RF_cancelCmd() or RF_flushCmd().
            printMessage(uart, rfUartBuffer, "RF: Transmission cancelled\r\n");
         break;
        case RF_EventCmdAborted:
         // Abrupt command termination caused by RF_cancelCmd() or
         // RF_flushCmd().
            printMessage(uart, rfUartBuffer, "RF: Transmission aborted\r\n");
         break;
        case RF_EventCmdStopped:
         // Graceful command termination caused by RF_cancelCmd() or
         // RF_flushCmd().
            printMessage(uart, rfUartBuffer, "RF: Transmission stopped\r\n");
         break;
        default:
         // Uncaught error event
         printMessage(uart, rfUartBuffer, "RF: Uncaught error for termination\r\n");
         while(1);
    }

    uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
    printMessage(uart, rfUartBuffer, "RF: Evaluate status\r\n");
    switch(cmdStatus) {
        case PROP_DONE_OK:
         // Packet transmitted successfully
         break;
        case PROP_DONE_STOPPED:
         // received CMD_STOP while transmitting packet and finished
         // transmitting packet
            printMessage(uart, rfUartBuffer, "RF: Command stopped\r\n");
         break;
        case PROP_DONE_ABORT:
         // Received CMD_ABORT while transmitting packet
            printMessage(uart, rfUartBuffer, "RF: Command aborted\r\n");
         break;
        case PROP_ERROR_PAR:
         // Observed illegal parameter
            printMessage(uart, rfUartBuffer, "RF: Command parameters invalid\r\n");
         break;
        case PROP_ERROR_NO_SETUP:
         // Command sent without setting up the radio in a supported
         // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
            printMessage(uart, rfUartBuffer, "RF: Radio not set up\r\n");
         break;
        case PROP_ERROR_NO_FS:
         // Command sent without the synthesizer being programmed
            printMessage(uart, rfUartBuffer, "RF: Synthesizer not programmed\r\n");
         break;
        case PROP_ERROR_TXUNF:
         // TX underflow observed during operation
            printMessage(uart, rfUartBuffer, "RF: TX underflow\r\n");
         break;
        default:
         // Uncaught error event - these could come from the
         // pool of states defined in rf_mailbox.h
         printMessage(uart, rfUartBuffer, "RF: Uncaught error for command status\r\n");
         while(1);
    }

}

/*
 *  ======== threadFxn1 ========
 *  Open a ADC handle and get an array of sampling results after
 *  calling several conversions.
 */
void *rfThreadFunc(void *arg0) {
    UART_Handle uart = *(UART_Handle *) arg0;
    printMessage(uart, rfUartBuffer, "RF: Thread started\r\n");

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL) {
        printMessage(uart, rfUartBuffer, "RF: could not gain PIN access\r\n");
        while(1);
    }

    printMessage(uart, rfUartBuffer, "RF: start sending\r\n");

    #ifdef POWER_MEASUREMENT
    #if defined(Board_CC1350_LAUNCHXL)
     /* Route out PA active pin to Board_DIO30_SWPWR */
     PINCC26XX_setMux(ledPinHandle, Board_DIO30_SWPWR, PINCC26XX_MUX_RFC_GPO1);
    #endif
    #endif

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    while(1)
    {
        /* Create packet with incrementing sequence number and random payload */
        packet[0] = (uint8_t)(seqNumber >> 8);
        packet[1] = (uint8_t)(seqNumber++);

        uint8_t i;
        printMessage(uart, rfUartBuffer, "RF: Create random packet\r\n");
        for (i = 2; i < PAYLOAD_LENGTH; i++) {
            packet[i] = rand();
        }

        /* Send packet */
        printMessageWithArg(uart, rfUartBuffer, "RF: Sending packet: %d\r\n", 1, packet[2]);
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);

        handleTransmissionResult(uart, terminationReason);

        // TODO Pins
        sleep(rfSleepTime);
    }
}
