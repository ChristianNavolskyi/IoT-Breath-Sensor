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
 *  ======== main_tirtos.c ========
 */
#include <stdint.h>
#include <xdc/std.h>

/* Example/Board Header files */
#include <ti/drivers/Board.h>
#include <ti/drivers/UART.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

#include <Board.h>
#include <structs.h>

/* Stack size in bytes */
#define MAILBOX_SIZE 15
#define TASKSTACKSIZE   512

// Mailbox
MailboxMessageObject mailboxBuffer[MAILBOX_SIZE];
Mailbox_Struct mailboxStruct;

// Tasks
extern Void *adcTaskFunc(UArg arg0, UArg arg1);
extern Void *rfTaskFunc(UArg arg0, UArg arg1);

Task_Struct task0Struct, task1Struct;
Char task0Stack[TASKSTACKSIZE], task1Stack[TASKSTACKSIZE];


UART_Handle initUART() {
    UART_Handle uart;
    UART_Params uartParams;

    UART_init();

    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.writeMode = UART_MODE_BLOCKING;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

    return uart;
}

Mailbox_Handle initMailbox() {
    Mailbox_Params mailboxParams;
    Mailbox_Params_init(&mailboxParams);

    mailboxParams.buf = (Ptr)mailboxBuffer;
    mailboxParams.bufSize = sizeof(mailboxBuffer);

    Mailbox_construct(&mailboxStruct, sizeof(Message), MAILBOX_SIZE, &mailboxParams, NULL);
    return Mailbox_handle(&mailboxStruct);
}

/*
 *  ======== main ========
 */
int main(void) {
    Task_Params taskParams;

    /* Call driver init functions */
    Board_init();
    UART_Handle uart = initUART();
    Mailbox_Handle mailbox = initMailbox();

    ThreadHandles handles;
    handles.uart = &uart;
    handles.mailbox = &mailbox;

    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.arg0 = &handles;

    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr)adcTaskFunc, &taskParams, NULL);

    taskParams.stack = &task1Stack;
    Task_construct(&task1Struct, (Task_FuncPtr)rfTaskFunc, &taskParams, NULL);

    BIOS_start();

    return (0);
}
