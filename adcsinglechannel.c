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
#include <ti/drivers/ADCBuf.h>
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

uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];


/* RF properties */
/* Packet TX Configuration */
#define PAYLOAD_LENGTH      30
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

// ADC value and flag for thread communication
volatile static uint32_t adcValue = 0;
volatile static uint8_t valueFlag = 0; // 0 - no value present, 1 - value present

/*
 * This function is called whenever an ADC buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel) {
    /* Adjust raw ADC values and convert them to microvolts */
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    Display_printf(*(Display_Handle *) conversion->arg, 0, 0, "ADC: Read value: %d\n", microVoltBuffer[0]);

//    if (ADCBuf_convert(handle, conversion, 1) != ADCBuf_STATUS_SUCCESS) {
//        /* Did not start conversion process correctly. */
//        Display_printf(display, 0, 0, "ADC: Could not start conversion\n");
//        while(1);
//    }
}

/* Read values from adc */
void *adcThreadFunc(void *arg0) {
    Display_Handle display = *(Display_Handle *) arg0;

    Display_printf(display, 0, 0, "ADC: Thread started\n");

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
    adcConversion.arg = &display;
    adcConversion.adcChannel = CC1350_LAUNCHXL_ADCBUF0CHANNELVSS;
    adcConversion.sampleBuffer = sampleBufferOne;
    adcConversion.sampleBufferTwo = sampleBufferTwo;
    adcConversion.samplesRequestedCount = ADCBUFFERSIZE;

    if (adcBuf == NULL){
        /* ADCBuf failed to open. */
        Display_printf(display, 0, 0, "ADC: Could not open ADC port\n");
        while(1);
    }

    /* Start converting. */
    Display_printf(display, 0, 0, "ADC: Starting first conversion\n");
    if (ADCBuf_convert(adcBuf, &adcConversion, 1) != ADCBuf_STATUS_SUCCESS) {
        /* Did not start conversion process correctly. */
        Display_printf(display, 0, 0, "ADC: Could not start conversion\n");
        while(1);
    }

    while(1) {
        sleep(1000);
    }
}

/*
 *  ======== threadFxn1 ========
 *  Open a ADC handle and get an array of sampling results after
 *  calling several conversions.
 */
void *rfThreadFunc(void *arg0) {
    Display_Handle display = *(Display_Handle *) arg0;

    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL) {
        Display_printf(display, 0, 0, "RF: could not gain PIN access\n");
        while(1);
    }

    Display_printf(display, 0, 0, "RF: start sending\n");

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
        Display_printf(display, 0, 0, "RF: Create random packet\n");
        for (i = 2; i < PAYLOAD_LENGTH; i++) {
            packet[i] = rand();
        }

        /* Send packet */
        Display_printf(display, 0, 0, "RF: Sending packet: %d\n", packet);
        RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);

        Display_printf(display, 0, 0, "RF: Evaluate termination reason\n");
        switch(terminationReason) {
            case RF_EventLastCmdDone:
             // A stand-alone radio operation command or the last radio
             // operation command in a chain finished.
             break;
            case RF_EventCmdCancelled:
             // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
                Display_printf(display, 0, 0, "RF: Transmission cancelled\n");
             break;
            case RF_EventCmdAborted:
             // Abrupt command termination caused by RF_cancelCmd() or
             // RF_flushCmd().
                Display_printf(display, 0, 0, "RF: Transmission aborted\n");
             break;
            case RF_EventCmdStopped:
             // Graceful command termination caused by RF_cancelCmd() or
             // RF_flushCmd().
                Display_printf(display, 0, 0, "RF: Transmission stopped\n");
             break;
            default:
             // Uncaught error event
             Display_printf(display, 0, 0, "RF: Uncaught error for termination\n");
             while(1);
        }

        uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropTx)->status;
        Display_printf(display, 0, 0, "RF: Evaluate status\n");
        switch(cmdStatus) {
            case PROP_DONE_OK:
             // Packet transmitted successfully
             break;
            case PROP_DONE_STOPPED:
             // received CMD_STOP while transmitting packet and finished
             // transmitting packet
                Display_printf(display, 0, 0, "RF: Command stopped\n");
             break;
            case PROP_DONE_ABORT:
             // Received CMD_ABORT while transmitting packet
                Display_printf(display, 0, 0, "RF: Command aborted\n");
             break;
            case PROP_ERROR_PAR:
             // Observed illegal parameter
                Display_printf(display, 0, 0, "RF: Command parameters invalid\n");
             break;
            case PROP_ERROR_NO_SETUP:
             // Command sent without setting up the radio in a supported
             // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
                Display_printf(display, 0, 0, "RF: Radio not set up\n");
             break;
            case PROP_ERROR_NO_FS:
             // Command sent without the synthesizer being programmed
                Display_printf(display, 0, 0, "RF: Synthesizer not programmed\n");
             break;
            case PROP_ERROR_TXUNF:
             // TX underflow observed during operation
                Display_printf(display, 0, 0, "RF: TX underflow\n");
             break;
            default:
             // Uncaught error event - these could come from the
             // pool of states defined in rf_mailbox.h
             Display_printf(display, 0, 0, "RF: Uncaught error for command status\n");
             while(1);
        }

        #ifndef POWER_MEASUREMENT
            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1, !PIN_getOutputValue(Board_PIN_LED1));
        #endif
            /* Power down the radio */
            RF_yield(rfHandle);

        #ifdef POWER_MEASUREMENT
            /* Sleep for PACKET_INTERVAL s */
            sleep(PACKET_INTERVAL);
        #else
            /* Sleep for PACKET_INTERVAL us */
            usleep(PACKET_INTERVAL);
        #endif
    }
}
