#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Mailbox.h>

typedef struct Message {
    uint32_t counter;
    uint32_t value;
} Message;

typedef struct MailboxMessageObject {
    Mailbox_MbxElem elem;
    Message msg;

} MailboxMessageObject;

typedef struct ThreadHandles {
    UART_Handle *uart;
    Mailbox_Handle *mailbox;
} ThreadHandles;
