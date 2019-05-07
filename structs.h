#include <ti/drivers/UART.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Mailbox.h>

typedef struct {
    uint32_t counter;
    uint32_t value;
} Message;

typedef struct {
    UART_Handle *uart;
    Mailbox_Handle *mailbox;
    Event_Handle *event;
    UInt eventId;
} ThreadHandles;
