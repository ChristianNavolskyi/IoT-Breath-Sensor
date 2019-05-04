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
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

/* POSIX Header files */
#include <pthread.h>

/* Driver Header files */
#include <ti/drivers/ADC.h>
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/display/Display.h>

#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Example/Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"


/* ADC properties */
#define ADCBUFFERSIZE    (1)

ADC_Handle adcHandle;
uint16_t adcSample;


/* RF properties */
/* Packet TX Configuration */
#define PAYLOAD_LENGTH      6
#ifdef POWER_MEASUREMENT
#define PACKET_INTERVAL     5  /* For power measurement set packet interval to 5s */
#else
#define PACKET_INTERVAL     1000000  /* Set packet interval to 1000000µs */
#endif

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static uint8_t packet[PAYLOAD_LENGTH];
static uint16_t seqNumber;

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

// Display for UART
Display_Handle display;
#define IS_DEBUGGING true

void initDisplay() {
    Display_init();

    /* Open the display for output */
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
      /* Failed to open display driver */
      while (1);
    }
}

void printMessage(char *fmt, ...) {
    if (IS_DEBUGGING) {
        va_list args;
        va_start(args, fmt);

        printMessage(fmt, args);

        va_end(args);
    }
}

void initADB() {
    ADC_Params adcParams;

    ADC_Params_init(&adcParams);

    adcHandle = ADC_open(CC1350_LAUNCHXL_ADCVDDS, &adcParams);

    if (adcHandle == NULL){
        /* ADCBuf failed to open. */
        printMessage("ADC: Could not open ADC port\n");
        while(1);
    }
}

void initRF() {
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL) {
        printMessage("RF: could not gain PIN access\n");
        while(1);
    }

    printMessage("RF: start sending\n");

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
}

void handleRFResult(RF_EventMask terminationReason) {
    printMessage("RF: Evaluate termination reason\n");
    switch (terminationReason) {
    case RF_EventLastCmdDone:
        // A stand-alone radio operation command or the last radio
        // operation command in a chain finished.
        break;
    case RF_EventCmdCancelled:
        // Command cancelled before it was started; it can be caused
        // by RF_cancelCmd() or RF_flushCmd().
        printMessage("RF: Transmission cancelled\n");
        break;
    case RF_EventCmdAborted:
        // Abrupt command termination caused by RF_cancelCmd() or
        // RF_flushCmd().
        printMessage("RF: Transmission aborted\n");
        break;
    case RF_EventCmdStopped:
        // Graceful command termination caused by RF_cancelCmd() or
        // RF_flushCmd().
        printMessage("RF: Transmission stopped\n");
        break;
    default:
        // Uncaught error event
        printMessage("RF: Uncaught error for termination\n");
        while (1);
    }

    uint32_t cmdStatus = ((volatile RF_Op*) &RF_cmdPropTx)->status;
    printMessage("RF: Evaluate status\n");
    switch (cmdStatus) {
    case PROP_DONE_OK:
        // Packet transmitted successfully
        break;
    case PROP_DONE_STOPPED:
        // received CMD_STOP while transmitting packet and finished
        // transmitting packet
        printMessage("RF: Command stopped\n");
        break;
    case PROP_DONE_ABORT:
        // Received CMD_ABORT while transmitting packet
        printMessage("RF: Command aborted\n");
        break;
    case PROP_ERROR_PAR:
        // Observed illegal parameter
        printMessage("RF: Command parameters invalid\n");
        break;
    case PROP_ERROR_NO_SETUP:
        // Command sent without setting up the radio in a supported
        // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
        printMessage("RF: Radio not set up\n");
        break;
    case PROP_ERROR_NO_FS:
        // Command sent without the synthesizer being programmed
        printMessage("RF: Synthesizer not programmed\n");
        break;
    case PROP_ERROR_TXUNF:
        // TX underflow observed during operation
        printMessage("RF: TX underflow\n");
        break;
    default:
        // Uncaught error event - these could come from the
        // pool of states defined in rf_mailbox.h
        Display_printf(display, 0, 0,
                       "RF: Uncaught error for command status\n");
        while (1);
    }
}

uint32_t getADCValue() {
    int_fast16_t res = ADC_convert(adcHandle, &adcSample);

    if (res == ADC_STATUS_SUCCESS) {

        uint32_t result =  ADC_convertToMicroVolts(adcHandle, adcSample);

        printMessage("ADC: raw result: %d\n", adcSample);
        printMessage("ADC: convert result: %d uV\n", result);
    }
    else {
        printMessage("ADC: convert failed\n");
    }
}

void sendValue(uint32_t value) {
    packet[0] = (uint8_t)(seqNumber >> 8);
    packet[1] = (uint8_t)(seqNumber++);

    uint8_t i;
    printMessage("RF: Create random packet\n");
    for (i = 2; i < PAYLOAD_LENGTH; i++) {
        packet[i] = value >> ((i - 2) * 8);
    }

    /* Send packet */
    printMessage("RF: Sending packet: %d\n", packet[2]);
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityHigh, NULL, 0);

    handleRFResult(terminationReason);
}

void startSamplingLoop() {
    uint32_t samplingValue;

    while (true) {
        samplingValue = getADCValue();
        sendValue(samplingValue);

        sleep(5);
    }
}

/* Read values from ADC and send via RF (Sub1GHz) */
void *senderThread(void *arg0) {
    initDisplay();
    initADB();
    initRF();

    /* Start converting. */
    printMessage("ADC: Starting first conversion\n");
    startSamplingLoop();

    return;
}
