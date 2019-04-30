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
#include <string.h>
#include <stdio.h>
/***** RF Includes *****/
#include <stdlib.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <stdint.h>
/***** ADC Includes *****/
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
/* Drivers */
#include <ti/drivers/rf/RF.h>

/* Board Header files */
#include "Board.h"

#include "smartrf_settings/smartrf_settings.h"


/* Packet TX Configuration */

#define PAYLOAD_LENGTH      5 //30
#define PACKET_INTERVAL     (uint32_t)(4000000*0.5f) /* Set packet interval to 500ms */
#define TASKSTACKSIZE    (768)
#define ADCBUFFERSIZE    (1)

/***** ADC Params and Variables *****/
Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];
uint16_t sampleBufferOne[ADCBUFFERSIZE];
uint16_t sampleBufferTwo[ADCBUFFERSIZE];
uint32_t microVoltBuffer[ADCBUFFERSIZE];

/***** RF Params and Variables *****/
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint8_t packet[PAYLOAD_LENGTH];
uint32_t time = 0;


unsigned createMask(unsigned a, unsigned b)
{
   unsigned r = 0; unsigned i = 0;
   for (i=a; i<=b; i++)
       r |= 1 << i;

   return r;
}

void sendDataViaRf(uint32_t value);
void send(uint32_t value);

/*
 * This function is called whenever a buffer is full.
 * The content of the buffer is then converted into human-readable format and
 * sent to the PC via UART.
 *
 */
void adcBufCallback(ADCBuf_Handle handle, ADCBuf_Conversion *conversion, void *completedADCBuffer, uint32_t completedChannel) {
    ADCBuf_adjustRawValues(handle, completedADCBuffer, ADCBUFFERSIZE, completedChannel);
    ADCBuf_convertAdjustedToMicroVolts(handle, completedChannel, completedADCBuffer, microVoltBuffer, ADCBUFFERSIZE);

    send(microVoltBuffer[0]);
}

void sendDataViaRf(uint32_t value) {
    time += PACKET_INTERVAL;
    RF_cmdPropTx.startTime = time;

    memcpy(packet, &value, sizeof(value));

    RF_postCmd(rfHandle, (RF_Op*) &RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
}

void startADCConversion(UArg arg0, UArg arg1) {
    ADCBuf_Handle adcBuf;
    ADCBuf_Params adcBufParams;
    ADCBuf_Conversion continuousConversionChannel;

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

void initialiseTransmissionParameters()
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;
    RF_cmdPropTx.startTrigger.pastTrig = 1;
    RF_cmdPropTx.startTime = 0;

    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
}

void send(uint32_t value) {
    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Get current time */
    time = RF_getCurrentTime();

    uint32_t firstByte, secondByte, thirdByte, fourthByte;
    firstByte = createMask(0,7);
    secondByte = createMask(8,15);
    thirdByte = createMask(16,23);
    fourthByte = createMask(24,31);

    //unsigned result = ;
    /* Create packet with incrementing sequence number and random payload */
    uint8_t i;
    for (i = 0; i < PAYLOAD_LENGTH; i=i+5)
    {
        packet[i] = time ;
        packet[i+1] = firstByte & value ;
        packet[i+2] = secondByte & value >> 8;
        packet[i+3] = thirdByte & value >> 16;
        packet[i+4] = fourthByte & value >> 24;
    }

    /* Set absolute TX time to utilize automatic power management */
    time += PACKET_INTERVAL;
    RF_cmdPropTx.startTime = time;

    /* Send packet */
    RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropTx, RF_PriorityNormal, NULL, 0);
}

void startConversionTask() {
    Task_Params taskParams;
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;

    Task_construct(&task0Struct, (Task_FuncPtr) startADCConversion, &taskParams, NULL);
}

int main(void)
{
    /* Call board init functions. */
    Board_initGeneral();
    Board_initADCBuf();

    /* Initialize task */
    initialiseTransmissionParameters();

    startConversionTask();

    /* Start BIOS */
    BIOS_start();

    return (0);
}
