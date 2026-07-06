#include "hal_data.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define RA8_USB_CTRL g_basic0_ctrl
#define RA8_USB_CFG  g_basic0_cfg

#define USB_RX_SIZE       256U
#define LINE_BUFFER_SIZE  256U
#define TX_BUFFER_SIZE    256U
#define LOOP_DELAY_MS     5U
#define HEARTBEAT_MS      1000U

static uint8_t g_rx_buffer[USB_RX_SIZE];
static char g_line_buffer[LINE_BUFFER_SIZE];
static char g_tx_buffer[TX_BUFFER_SIZE];

static volatile bool g_usb_configured = false;
static volatile bool g_usb_read_done = false;
static volatile bool g_usb_write_done = true;
static volatile uint32_t g_usb_rx_size = 0;

static uint32_t g_ms_ticks = 0;
static uint32_t g_last_heartbeat_ms = 0;
static uint32_t g_line_len = 0;
static uint32_t g_heartbeat_seq = 0;

static void app_loop(void);
static void usb_start_read(void);
static void usb_send_line(const char * line);
static void usb_handle_rx(const uint8_t * data, uint32_t len);
static void process_line(const char * line);
static uint32_t millis(void);
static void delay_ms(uint32_t ms);

void hal_entry(void)
{
    memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
    memset(g_line_buffer, 0, sizeof(g_line_buffer));
    memset(g_tx_buffer, 0, sizeof(g_tx_buffer));

    R_USB_Open(&RA8_USB_CTRL, &RA8_USB_CFG);

    while (1)
    {
        app_loop();
        delay_ms(LOOP_DELAY_MS);
        g_ms_ticks += LOOP_DELAY_MS;
    }
}

void usb_pcdc_callback(usb_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case USB_STATUS_CONFIGURED:
        {
            g_usb_configured = true;
            g_usb_write_done = true;
            usb_start_read();
            break;
        }

        case USB_STATUS_READ_COMPLETE:
        {
            g_usb_rx_size = p_args->data_size;
            if (0U == g_usb_rx_size || g_usb_rx_size > USB_RX_SIZE)
            {
                g_usb_rx_size = USB_RX_SIZE;
            }
            g_usb_read_done = true;
            break;
        }

        case USB_STATUS_WRITE_COMPLETE:
        {
            g_usb_write_done = true;
            break;
        }

        case USB_STATUS_DETACH:
        {
            g_usb_configured = false;
            g_usb_read_done = false;
            g_usb_write_done = true;
            break;
        }

        default:
        {
            break;
        }
    }
}

static void app_loop(void)
{
    if (!g_usb_configured)
    {
        return;
    }

    if (g_usb_read_done)
    {
        g_usb_read_done = false;
        usb_handle_rx(g_rx_buffer, g_usb_rx_size);
        memset(g_rx_buffer, 0, sizeof(g_rx_buffer));
        usb_start_read();
    }

    if ((millis() - g_last_heartbeat_ms) >= HEARTBEAT_MS)
    {
        g_heartbeat_seq++;
        snprintf(g_tx_buffer,
                 sizeof(g_tx_buffer),
                 "RA8P1_SIM_HELLO seq=%lu uptime_ms=%lu\r\n",
                 (unsigned long) g_heartbeat_seq,
                 (unsigned long) millis());
        usb_send_line(g_tx_buffer);
        g_last_heartbeat_ms = millis();
    }
}

static void usb_start_read(void)
{
    if (!g_usb_configured)
    {
        return;
    }

    (void) R_USB_Read(&RA8_USB_CTRL, g_rx_buffer, USB_RX_SIZE, USB_CLASS_PCDC);
}

static void usb_send_line(const char * line)
{
    if (!g_usb_configured || NULL == line)
    {
        return;
    }

    uint32_t len = (uint32_t) strlen(line);
    if (0U == len)
    {
        return;
    }

    uint32_t guard = 0U;
    while (!g_usb_write_done && guard < 200U)
    {
        delay_ms(1U);
        guard++;
    }

    if (!g_usb_write_done)
    {
        return;
    }

    g_usb_write_done = false;
    (void) R_USB_Write(&RA8_USB_CTRL, (uint8_t *) line, len, USB_CLASS_PCDC);
}

static void usb_handle_rx(const uint8_t * data, uint32_t len)
{
    if (NULL == data || 0U == len)
    {
        return;
    }

    for (uint32_t i = 0U; i < len; i++)
    {
        char c = (char) data[i];

        if ('\0' == c || '\r' == c)
        {
            continue;
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
            continue;
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
        }
    }
}

static void process_line(const char * line)
{
    snprintf(g_tx_buffer,
             sizeof(g_tx_buffer),
             "RA8P1_ECHO uptime_ms=%lu text=\"%s\"\r\n",
             (unsigned long) millis(),
             line);
    usb_send_line(g_tx_buffer);
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
