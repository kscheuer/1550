#include "usb_comm.h"

/**
 * @file    usb_comm.c
 * @brief   USB Communication Module Implementation
 * 
 * Provides deferred USB command processing to avoid ISR/main loop conflicts.
 * The RX ISR just buffers data and sets a flag; the main loop does the
 * actual processing and transmission.
 * 
 * @author  Your Name
 * @date    2025
 */

#include "usb_comm.h"
#include "thermal_control.h" /* Use existing functional header instead of app.h */
#include "laser_current.h"
#include "main.h" /* For HAL handles */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

/* ============================================================================
 * EXTERNAL DEPENDENCIES
 * ============================================================================ */

/* USB CDC transmit function from usbd_cdc_if.c */
extern uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

/**
 * @brief   Process incoming USB commands. 
 * FUNCTION NAME RETAINED FROM 638 REPO BUT CODE WAS WRITTEN BY GITHUB COPILOT. COULD BE IMPROVED  BUT USING FOR NOW. 
 * 
 * Supported Commands:
 * - help               : List available commands
 * - tr, TR             : Read current temperature
 * - tt, TT <val>       : Set target temperature (e.g., TT 25.0)
 * - tx, TX             : Stop/Disable TEC driver
 * - tp, TP=<val>       : Set Proportional gain (Kp)
 * - ti, TI=<val>       : Set Integral gain (Ki)
 * - td, TD=<val>       : Set Derivative gain (Kd)
 * - pid <P> <I> <D>    : Set all PID gains at once
 * - tec                : Read current DAC output value (0-4095)
 * - l, L <val>         : Set Laser Current DAC value
 */
/* ============================================================================
 * PRIVATE DATA
 * ============================================================================ */

/** Line buffer for accumulating characters until newline */
static volatile uint8_t line_buffer[USB_COMM_RX_BUFFER_SIZE];
static volatile uint32_t line_buffer_pos = 0;

/** Command queue for complete commands (after newline received) */
static volatile USB_CommandBuffer_t rx_queue[USB_COMM_CMD_QUEUE_SIZE];
static volatile uint8_t rx_queue_head = 0;  /* Next slot to write */
static volatile uint8_t rx_queue_tail = 0;  /* Next slot to read */
static volatile uint8_t rx_queue_count = 0; /* Number of pending commands */

/** TX buffer for outgoing data */
static uint8_t tx_buffer[USB_COMM_TX_BUFFER_SIZE];
static volatile uint32_t tx_buffer_length = 0;
static volatile bool tx_in_progress = false;
static volatile uint32_t tx_start_tick = 0;  /* For timeout detection */

/** TX timeout in milliseconds */
#define TX_TIMEOUT_MS  100

/** Temporary buffer for building responses */
static uint8_t tx_pending_buffer[USB_COMM_TX_BUFFER_SIZE];
static volatile uint32_t tx_pending_length = 0;

/** Echo buffer for characters to echo back */
static volatile uint8_t echo_buffer[64];
static volatile uint32_t echo_buffer_head = 0;
static volatile uint32_t echo_buffer_tail = 0;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

/* Case-insensitive string comparison helper */
static int str_icmp(const char *s1, const char *s2)
{
    while (*s1 && *s2)
    {
        char c1 = *s1;
        char c2 = *s2;
        if (c1 >= 'A' && c1 <= 'Z') c1 += ('a' - 'A');
        if (c2 >= 'A' && c2 <= 'Z') c2 += ('a' - 'A');
        if (c1 != c2) return (c1 - c2);
        s1++;
        s2++;
    }
    return (*s1 - *s2);
}

/* Case-insensitive string comparison with length limit helper */
static int str_nicmp(const char *s1, const char *s2, size_t n)
{
    if (n == 0) return 0;
    while (n-- > 0 && *s1 && *s2)
    {
        char c1 = *s1;
        char c2 = *s2;
        if (c1 >= 'A' && c1 <= 'Z') c1 += ('a' - 'A');
        if (c2 >= 'A' && c2 <= 'Z') c2 += ('a' - 'A');
        if (c1 != c2) return (c1 - c2);
        s1++;
        s2++;
    }
    if (n == 0) return 0; /* Length limit reached, equal so far */
    return (*s1 - *s2);
}

/* ============================================================================
 * COMMAND PROCESSING
 * ============================================================================ */

static void App_ProcessUSBCommand(uint8_t* cmd_buffer, uint32_t cmd_length) {
    /* Null-terminate the command buffer for string operations */
    if (cmd_length < USB_COMM_RX_BUFFER_SIZE) {
        cmd_buffer[cmd_length] = '\0';
    } else {
        cmd_buffer[USB_COMM_RX_BUFFER_SIZE - 1] = '\0';
    }

    /* Simple tokenizer */
    char* token = strtok((char*)cmd_buffer, " \r\n");
    if (token == NULL) return;

    if (str_icmp(token, "help") == 0) {
        USB_Comm_SendMessage("Available commands:\r\n");
        USB_Comm_SendMessage("  help              - Show this list\r\n");
        USB_Comm_SendMessage("  tr, TR            - Get current temperature\r\n");
        USB_Comm_SendMessage("  tt, TT <val>      - Set target temperature\r\n");
        USB_Comm_SendMessage("  tp=, TP=<val>     - Set Kp gain\r\n");
        USB_Comm_SendMessage("  ti=, TI=<val>     - Set Ki gain\r\n");
        USB_Comm_SendMessage("  td=, TD=<val>     - Set Kd gain\r\n");
        USB_Comm_SendMessage("  tx, TX            - Stop/Disable TEC\r\n");
        USB_Comm_SendMessage("  pid <kp> <ki> <kd> - Set all PID gains\r\n");
        USB_Comm_SendMessage("  tec               - Get TEC output (DAC value)\r\n");
        USB_Comm_SendMessage("  l, L <val>        - Set Laser Current DAC\r\n");
    } 
    else if (str_icmp(token, "tr") == 0) {
        float temp = Thermal_GetTemperature();
        char msg[64];
        snprintf(msg, sizeof(msg), "Temperature: %.2f C\r\n", temp);
        USB_Comm_SendMessage(msg);
    }
    else if (str_icmp(token, "tx") == 0) {
        Thermal_Stop();
        USB_Comm_SendMessage("TEC: Stopped (disabled)\r\n");
    }
    else if (str_icmp(token, "tt") == 0) {
        char* arg = strtok(NULL, " \r\n");
        if (arg) {
            float target = (float)atof(arg);
            Thermal_SetTarget(target);
            char msg[64];
            snprintf(msg, sizeof(msg), "Target set to: %.2f C\r\n", target);
            USB_Comm_SendMessage(msg);
        } else {
            USB_Comm_SendMessage("Error: Missing target value\r\n");
        }
    }
    else if (str_nicmp(token, "tp", 2) == 0) {
        /* Check for = or space */
        char* arg = NULL;
        char* eq = strchr(token, '=');
        if (eq) arg = eq + 1;
        else arg = strtok(NULL, " \r\n");
        
        if (arg && *arg) {
            float kp, ki, kd;
            Thermal_GetPIDGains(&kp, &ki, &kd);
            float new_val = (float)atof(arg);
            Thermal_SetPIDGains(new_val, ki, kd);
            char msg[64];
            snprintf(msg, sizeof(msg), "Kp set to: %.4f\r\n", new_val);
            USB_Comm_SendMessage(msg); 
        } else {
             USB_Comm_SendMessage("Error: Missing Kp value\r\n");
        }
    }
    else if (str_nicmp(token, "ti", 2) == 0) {
        char* arg = NULL;
        char* eq = strchr(token, '=');
        if (eq) arg = eq + 1;
        else arg = strtok(NULL, " \r\n");
        
        if (arg && *arg) {
            float kp, ki, kd;
            Thermal_GetPIDGains(&kp, &ki, &kd);
            float new_val = (float)atof(arg);
            Thermal_SetPIDGains(kp, new_val, kd);
            char msg[64];
            snprintf(msg, sizeof(msg), "Ki set to: %.4f\r\n", new_val);
            USB_Comm_SendMessage(msg); 
        } else {
             USB_Comm_SendMessage("Error: Missing Ki value\r\n");
        }
    }
    else if (str_nicmp(token, "td", 2) == 0) {
        char* arg = NULL;
        char* eq = strchr(token, '=');
        if (eq) arg = eq + 1;
        else arg = strtok(NULL, " \r\n");
        
        if (arg && *arg) {
            float kp, ki, kd;
            Thermal_GetPIDGains(&kp, &ki, &kd);
            float new_val = (float)atof(arg);
            Thermal_SetPIDGains(kp, ki, new_val);
            char msg[64];
            snprintf(msg, sizeof(msg), "Kd set to: %.4f\r\n", new_val);
            USB_Comm_SendMessage(msg); 
        } else {
             USB_Comm_SendMessage("Error: Missing Kd value\r\n");
        }
    }
    else if (str_icmp(token, "l") == 0) {
        /* Set Laser Current (DAC value) */
        char* arg = strtok(NULL, " \r\n");
        if (arg) {
            uint16_t dac_val = (uint16_t)atoi(arg);
            
            /* Convert integer DAC value (0-65535) to normalized float (0.0-1.0) */
            float normalized_val = (float)dac_val / 65535.0f;
            Laser_SetCurrent(normalized_val);
            
            char msg[64];
            snprintf(msg, sizeof(msg), "Set Laser DAC: %u\r\n", dac_val);
            USB_Comm_SendMessage(msg);
        } else {
             USB_Comm_SendMessage("Error: Missing DAC value\r\n");
        }
    }
    else if (str_icmp(token, "pid") == 0) {
        char* arg1 = strtok(NULL, " \r\n");
        char* arg2 = strtok(NULL, " \r\n");
        char* arg3 = strtok(NULL, " \r\n");
        
        if (arg1 && arg2 && arg3) {
            float kp = (float)atof(arg1);
            float ki = (float)atof(arg2);
            float kd = (float)atof(arg3);
            Thermal_SetPIDGains(kp, ki, kd);
            char msg[128];
            snprintf(msg, sizeof(msg), "PID Gains set: Kp=%.2f, Ki=%.2f, Kd=%.2f\r\n", kp, ki, kd);
            USB_Comm_SendMessage(msg);
        } else {
            USB_Comm_SendMessage("Error: Missing PID values\r\n");
        }
    }
    else if (str_icmp(token, "tec") == 0) {
        uint16_t tec = Thermal_GetTECOutput();
        char msg[64];
        snprintf(msg, sizeof(msg), "TEC DAC Output: %u\r\n", tec);
        USB_Comm_SendMessage(msg);
    }
    else {
        char msg[64];
        snprintf(msg, sizeof(msg), "Unknown command: %s\r\n", token);
        USB_Comm_SendMessage(msg);
    }
    
    USB_Comm_SendMessage("> ");
}

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */

void USB_Comm_Init(void)
{
    /* Clear line buffer */
    line_buffer_pos = 0;
    
    /* Clear RX queue */
    for (int i = 0; i < USB_COMM_CMD_QUEUE_SIZE; i++)
    {
        rx_queue[i].pending = false;
        rx_queue[i].length = 0;
    }
    rx_queue_head = 0;
    rx_queue_tail = 0;
    rx_queue_count = 0;
    
    /* Clear TX buffers */
    tx_buffer_length = 0;
    tx_in_progress = false;
    tx_pending_length = 0;
    
    /* Clear echo buffer */
    echo_buffer_head = 0;
    echo_buffer_tail = 0;
}

/* ============================================================================
 * ISR INTERFACE
 * ============================================================================ */

/**
 * @brief   Queue a character for echo (called from ISR)
 * @param   c   Character to echo
 */
static void QueueEcho(uint8_t c)
{
    uint32_t next_head = (echo_buffer_head + 1) % sizeof(echo_buffer);
    if (next_head != echo_buffer_tail)
    {
        echo_buffer[echo_buffer_head] = c;
        echo_buffer_head = next_head;
    }
}

/**
 * @brief   Queue a complete command line
 * @param   data    Command data (without newline)
 * @param   length  Length of command
 * @return  true if queued successfully
 */
static bool QueueCommand(const uint8_t* data, uint32_t length)
{
    if (rx_queue_count >= USB_COMM_CMD_QUEUE_SIZE)
    {
        return false;  /* Queue full */
    }
    
    if (length > USB_COMM_RX_BUFFER_SIZE)
    {
        length = USB_COMM_RX_BUFFER_SIZE;
    }
    
    memcpy((void*)rx_queue[rx_queue_head].data, data, length);
    rx_queue[rx_queue_head].length = length;
    rx_queue[rx_queue_head].pending = true;
    
    rx_queue_head = (rx_queue_head + 1) % USB_COMM_CMD_QUEUE_SIZE;
    rx_queue_count++;
    
    return true;
}

bool USB_Comm_ReceiveFromISR(const uint8_t* data, uint32_t length)
{
    /* Process each received character */
    for (uint32_t i = 0; i < length; i++)
    {
        uint8_t c = data[i];
        
        /* Check for line terminators (CR or LF) */
        if (c == '\r' || c == '\n')
        {
            /* Echo newline sequence */
            QueueEcho('\r');
            QueueEcho('\n');
            
            /* Only process if we have data in the buffer */
            if (line_buffer_pos > 0)
            {
                /* Queue the complete command */
                QueueCommand((const uint8_t*)line_buffer, line_buffer_pos);
                line_buffer_pos = 0;
            }
            /* Skip consecutive CR/LF */
            continue;
        }
        
        /* Handle backspace (BS=0x08 or DEL=0x7F) */
        if (c == 0x08 || c == 0x7F)
        {
            if (line_buffer_pos > 0)
            {
                line_buffer_pos--;
                /* Echo backspace sequence: BS, space, BS (erases character) */
                QueueEcho(0x08);
                QueueEcho(' ');
                QueueEcho(0x08);
            }
            continue;
        }
        
        /* Add character to line buffer if there's room */
        if (line_buffer_pos < USB_COMM_RX_BUFFER_SIZE - 1)
        {
            line_buffer[line_buffer_pos++] = c;
            /* Echo the character */
            QueueEcho(c);
        }
        /* else: buffer full, discard character */
    }
    
    return true;
}

/* ============================================================================
 * MAIN LOOP INTERFACE (INTERNAL)
 * ============================================================================ */

static bool HasPendingCommand(void)
{
    return (rx_queue_count > 0);
}

static bool GetPendingCommand(uint8_t* buffer, uint32_t max_length, uint32_t* length)
{
    if (rx_queue_count == 0)
    {
        return false;
    }
    
    /* Get command from tail of queue */
    uint32_t cmd_len = rx_queue[rx_queue_tail].length;
    
    if (cmd_len > max_length)
    {
        cmd_len = max_length;  /* Truncate if buffer too small */
    }
    
    memcpy(buffer, (void*)rx_queue[rx_queue_tail].data, cmd_len);
    *length = cmd_len;
    
    return true;
}

static void CommandProcessed(void)
{
    if (rx_queue_count == 0)
    {
        return;
    }
    
    /* Clear the processed slot */
    rx_queue[rx_queue_tail].pending = false;
    rx_queue[rx_queue_tail].length = 0;
    
    /* Advance tail pointer (circular) */
    rx_queue_tail = (rx_queue_tail + 1) % USB_COMM_CMD_QUEUE_SIZE;
    rx_queue_count--;
}

void USB_Comm_Process(void)
{
    /* First, send any pending echo characters */
    while (echo_buffer_tail != echo_buffer_head)
    {
        /* Try to queue echo character for transmission */
        uint8_t c = echo_buffer[echo_buffer_tail];
        if (USB_Comm_SendData(&c, 1))
        {
            echo_buffer_tail = (echo_buffer_tail + 1) % sizeof(echo_buffer);
        }
        else
        {
            break;  /* TX buffer full, try again later */
        }
    }
    
    /* Try to flush any pending TX data (including echoes) */
    USB_Comm_FlushTx();
    
    /* Only process ONE command per loop iteration to avoid flooding TX */
    /* Also skip if TX is busy or buffer is getting full */
    if (HasPendingCommand() && !tx_in_progress && tx_pending_length < USB_COMM_TX_BUFFER_SIZE / 2)
    {
        uint8_t cmd_buffer[USB_COMM_RX_BUFFER_SIZE];
        uint32_t cmd_length;
        
        if (GetPendingCommand(cmd_buffer, sizeof(cmd_buffer), &cmd_length))
        {
            /* Process the command (this may queue response data) */
            App_ProcessUSBCommand(cmd_buffer, cmd_length);
            
            /* Mark command as processed */
            CommandProcessed();
            
            /* Try to send any response that was queued */
            USB_Comm_FlushTx();
        }
    }
}

/* ============================================================================
 * TRANSMISSION INTERFACE
 * ============================================================================ */

bool USB_Comm_SendMessage(const char* message)
{
    if (message == NULL)
    {
        return false;
    }
    
    uint32_t len = strlen(message);
    return USB_Comm_SendData((const uint8_t*)message, len);
}

bool USB_Comm_SendData(const uint8_t* data, uint32_t length)
{
    if (data == NULL || length == 0)
    {
        return false;
    }
    
    /* Check if there's room in the pending buffer */
    if (tx_pending_length + length > USB_COMM_TX_BUFFER_SIZE)
    {
        /* Try to flush first */
        USB_Comm_FlushTx();
        
        /* Check again */
        if (tx_pending_length + length > USB_COMM_TX_BUFFER_SIZE)
        {
            return false;  /* Still no room */
        }
    }
    
    /* Append to pending buffer */
    memcpy(&tx_pending_buffer[tx_pending_length], data, length);
    tx_pending_length += length;
    
    return true;
}

bool USB_Comm_FlushTx(void)
{
    /* Nothing to send? */
    if (tx_pending_length == 0)
    {
        return true;
    }
    
    /* Previous transmission still in progress? */
    if (tx_in_progress)
    {
        /* Check for timeout - if stuck for too long, force reset */
        uint32_t now = HAL_GetTick();
        if ((now - tx_start_tick) > TX_TIMEOUT_MS)
        {
            /* TX stuck - force reset */
            tx_in_progress = false;
            tx_buffer_length = 0;
        }
        else
        {
            return false;  /* Still waiting */
        }
    }
    
    /* Copy pending data to TX buffer */
    memcpy(tx_buffer, (void*)tx_pending_buffer, tx_pending_length);
    tx_buffer_length = tx_pending_length;
    
    /* Clear pending buffer */
    tx_pending_length = 0;
    
    /* Attempt transmission */
    uint8_t result = CDC_Transmit_HS(tx_buffer, (uint16_t)tx_buffer_length);
    
    if (result == 0)  /* USBD_OK */
    {
        tx_in_progress = true;
        tx_start_tick = HAL_GetTick();
        return true;
    }
    else
    {
        /* Transmission failed (busy or error) - data remains in tx_buffer
         * We'll try again on next flush. For now, copy back to pending. */
        memcpy((void*)tx_pending_buffer, tx_buffer, tx_buffer_length);
        tx_pending_length = tx_buffer_length;
        return false;
    }
}

/**
 * @brief   Called when USB TX is complete (from CDC_TransmitCplt_HS)
 * 
 * This clears the tx_in_progress flag so the next transmission can proceed.
 */
void USB_Comm_TxComplete(void)
{
    tx_in_progress = false;
    tx_buffer_length = 0;
}
