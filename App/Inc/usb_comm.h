#ifndef USB_COMM_H
#define USB_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "app_config.h"

/* Types */
typedef struct
{
    uint8_t data[USB_COMM_RX_BUFFER_SIZE];
    uint32_t length;
    bool pending;
} USB_CommandBuffer_t;

/* Exported functions ------------------------------------------------------- */
void USB_Comm_Init(void);
bool USB_Comm_ReceiveFromISR(const uint8_t* data, uint32_t length);
void USB_Comm_TxComplete(void);

void USB_Comm_Process(void);

bool USB_Comm_SendMessage(const char* message);
bool USB_Comm_SendData(const uint8_t* data, uint32_t length);
bool USB_Comm_FlushTx(void);

#endif /* USB_COMM_H */
