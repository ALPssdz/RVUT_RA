#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define UART_CTRL        g_uart9_ctrl
#define UART_CFG         g_uart9_cfg

#define LINE_BUFFER_SIZE 256U
#define TX_BUFFER_SIZE   256U
#define LOOP_DELAY_MS    5U
#define HEARTBEAT_MS     1000U

static char g_line_buffer[LINE_BUFFER_SIZE];
static char g_tx_buffer[TX_BUFFER_SIZE];

static volatile bool g_uart_tx_done = true;
static volatile uint32_t g_rx_write_index = 0;
static volatile uint32_t g_rx_read_index = 0;
static volatile uint8_t g_rx_ring[LINE_BUFFER_SIZE];

static uint32_t g_ms_ticks = 0;
static uint32_t g_last_heartbeat_ms = 0;
static uint32_t g_line_len = 0;
static uint32_t g_heartbeat_seq = 0;

static void app_loop(void);
static void uart_send_line(const char * line);
static bool uart_pop_char(char * c);
static void process_rx_char(char c);
static void process_line(const char * line);
static uint32_t millis(void);
static void delay_ms(uint32_t ms);

void hal_entry(void)
{
    memset(g_line_buffer, 0, sizeof(g_line_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));

    R_SCI_B_UART_Open(&UART_CTRL, &UART_CFG);

    uart_send_line("\r\nRA8P1 UART9 echo test started. baud=2000000 8N1\r\n");

    while (1)
    {
        app_loop();
        delay_ms(LOOP_DELAY_MS);
        g_ms_ticks += LOOP_DELAY_MS;
    }
}

void UART9_Callback(uart_callback_args_t * p_args)
{
    if (NULL == p_args)
    {
        return;
    }

    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
            uint32_t next = (g_rx_write_index + 1U) % LINE_BUFFER_SIZE;
            if (next != g_rx_read_index)
            {
                g_rx_ring[g_rx_write_index] = (uint8_t) p_args->data;
                g_rx_write_index = next;
            }
            break;
        }

        case UART_EVENT_TX_COMPLETE:
        {
            g_uart_tx_done = true;
            break;
        }

        case UART_EVENT_ERR_PARITY:
        case UART_EVENT_ERR_FRAMING:
        case UART_EVENT_ERR_OVERFLOW:
        case UART_EVENT_BREAK_DETECT:
        {
            g_uart_tx_done = true;
            break;
        }

        default:
        {
            break;
        }
    }
}

void usb_pcdc_callback(usb_callback_args_t * p_args)
{
    (void) p_args;
}

static void app_loop(void)
{
    char c = '\0';

    while (uart_pop_char(&c))
    {
        process_rx_char(c);
    }

    if ((millis() - g_last_heartbeat_ms) >= HEARTBEAT_MS)
    {
        g_heartbeat_seq++;
        snprintf(g_tx_buffer,
                 sizeof(g_tx_buffer),
                 "RA8P1_UART9_HELLO seq=%lu uptime_ms=%lu\r\n",
                 (unsigned long) g_heartbeat_seq,
                 (unsigned long) millis());
        uart_send_line(g_tx_buffer);
        g_last_heartbeat_ms = millis();
    }
}

static void uart_send_line(const char * line)
{
    if (NULL == line)
    {
        return;
    }

    uint32_t len = (uint32_t) strlen(line);
    if (0U == len)
    {
        return;
    }

    uint32_t guard = 0U;
    while (!g_uart_tx_done && guard < 200U)
    {
        delay_ms(1U);
        guard++;
    }

    if (!g_uart_tx_done)
    {
        return;
    }

    g_uart_tx_done = false;
    if (FSP_SUCCESS != R_SCI_B_UART_Write(&UART_CTRL, (uint8_t const *) line, len))
    {
        g_uart_tx_done = true;
    }
}

static bool uart_pop_char(char * c)
{
    if (NULL == c)
    {
        return false;
    }

    if (g_rx_read_index == g_rx_write_index)
    {
        return false;
    }

    *c = (char) g_rx_ring[g_rx_read_index];
    g_rx_read_index = (g_rx_read_index + 1U) % LINE_BUFFER_SIZE;
    return true;
}

static void process_rx_char(char c)
{
    if ('\0' == c || '\r' == c)
    {
        return;
    }

    if ('\n' == c)
    {
        g_line_buffer[g_line_len] = '\0';
        if (g_line_len > 0U)
        {
            process_line(g_line_buffer);
        }
        g_line_len = 0U;
        memset(g_line_buffer, 0, sizeof(g_line_buffer));
        return;
    }

    if (g_line_len < (LINE_BUFFER_SIZE - 1U))
    {
        g_line_buffer[g_line_len] = c;
        g_line_len++;
    }
    else
    {
        g_line_len = 0U;
        memset(g_line_buffer, 0, sizeof(g_line_buffer));
        uart_send_line("RA8P1_UART9_LINE_OVERFLOW\r\n");
    }
}

static void process_line(const char * line)
{
    snprintf(g_tx_buffer,
             sizeof(g_tx_buffer),
             "RA8P1_UART9_ECHO uptime_ms=%lu text=\"%s\"\r\n",
             (unsigned long) millis(),
             line);
    uart_send_line(g_tx_buffer);
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void delay_ms(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}

#if BSP_TZ_SECURE_BUILD

FSP_CPP_HEADER
BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable();

BSP_CMSE_NONSECURE_ENTRY void template_nonsecure_callable()
{
}
FSP_CPP_FOOTER

#endif
