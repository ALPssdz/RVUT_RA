# RA8P1 USB2.0 Type-C CDC Master Controller

> Deprecated for the current CPKHMI-RA8P1 bring-up path. The verified transport
> is the on-board J-Link OB virtual COM port wired to RA8P1 SCI9 at 2,000,000
> baud. Use `hal_entry_uart9_master.c` for the current integration firmware.

This document is for the RA8P1 firmware developer.

## Goal

RA8P1 acts as the system master controller. It communicates with the RK3588 / Orange Pi RF Agent through the RA8P1 board USB2.0 Type-C port.

Recommended USB mode:

```text
USB CDC ACM device
Orange Pi device node: /dev/ttyACM0
Protocol: JSON Lines + checksum
```

RA8P1 responsibilities:

```text
1. Enumerate as USB CDC ACM device.
2. Send START_SCAN to RK3588.
3. Receive HEARTBEAT / AGENT_READY / DETECTION_REPORT.
4. Decide CLEAR / CANDIDATE / ALERT.
5. Send MASTER_DECISION back to RK3588.
```

## Required FSP Configuration

Configure the Renesas FSP project with:

```text
USB Basic Driver
USB PCDC / CDC ACM Device Class
USB mode: Device / Peripheral
Callback function: usb_pcdc_callback
```

If the generated USB instance names are not `g_basic0_ctrl` and `g_basic0_cfg`, change these macros in the code:

```c
#define RA8_USB_CTRL    g_basic0_ctrl
#define RA8_USB_CFG     g_basic0_cfg
```

## Full RA8P1 Application Code

```c
/*
 * RA8P1 USB2.0 Type-C CDC Master Controller
 *
 * Function:
 * - Enumerates as USB CDC ACM device on RK3588 / Orange Pi
 * - Orange Pi sees this board as /dev/ttyACM0
 * - Receives HEARTBEAT / AGENT_READY / DETECTION_REPORT from RK3588
 * - Sends START_SCAN and MASTER_DECISION to RK3588
 *
 * Protocol:
 * - JSON Lines
 * - One JSON object per line
 * - Outgoing frames include checksum compatible with Python link_protocol.py
 *
 * Required FSP configuration:
 * - USB Basic Driver
 * - USB PCDC / CDC ACM Device Class
 * - Device / Peripheral mode
 * - Callback function name: usb_pcdc_callback
 */

#include "hal_data.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* -------------------------------------------------------------------------- */
/* FSP USB instance names                                                       */
/* Change these if your FSP generated names are different.                      */
/* -------------------------------------------------------------------------- */

#define RA8_USB_CTRL    g_basic0_ctrl
#define RA8_USB_CFG     g_basic0_cfg

/* -------------------------------------------------------------------------- */
/* Application constants                                                        */
/* -------------------------------------------------------------------------- */

#define USB_RX_CHUNK_SIZE        256
#define LINE_BUFFER_SIZE         512
#define TX_BUFFER_SIZE           512

#define DETECT_WINDOW_SIZE       3
#define DETECT_CONFIRM_COUNT     2
#define SDS_ALERT_THRESHOLD      1.0f

#define HEARTBEAT_TIMEOUT_MS     3000U
#define START_SCAN_PERIOD_MS     1000U

/* -------------------------------------------------------------------------- */
/* State definitions                                                            */
/* -------------------------------------------------------------------------- */

typedef enum
{
    RA_STATE_BOOT = 0,
    RA_STATE_WAIT_USB,
    RA_STATE_READY,
    RA_STATE_SCANNING,
    RA_STATE_CANDIDATE,
    RA_STATE_ALERT,
    RA_STATE_FAULT
} ra_state_t;

typedef enum
{
    MASTER_DECISION_CLEAR = 0,
    MASTER_DECISION_CANDIDATE,
    MASTER_DECISION_ALERT
} master_decision_t;

/* -------------------------------------------------------------------------- */
/* USB globals                                                                  */
/* -------------------------------------------------------------------------- */

static uint8_t g_usb_rx_chunk[USB_RX_CHUNK_SIZE];

static volatile bool g_usb_configured = false;
static volatile bool g_usb_read_done = false;
static volatile bool g_usb_write_done = true;
static volatile uint32_t g_usb_rx_size = 0;

/* -------------------------------------------------------------------------- */
/* Application globals                                                          */
/* -------------------------------------------------------------------------- */

static ra_state_t g_state = RA_STATE_BOOT;

static char g_line_buffer[LINE_BUFFER_SIZE];
static uint32_t g_line_len = 0;

static uint16_t g_seq = 0;

static uint32_t g_ms_ticks = 0;
static uint32_t g_last_heartbeat_ms = 0;
static uint32_t g_last_start_scan_ms = 0;

static bool g_detect_history[DETECT_WINDOW_SIZE] = {false, false, false};
static float g_sds_history[DETECT_WINDOW_SIZE] = {0.0f, 0.0f, 0.0f};
static uint32_t g_detect_index = 0;

static master_decision_t g_last_decision = MASTER_DECISION_CLEAR;

/* -------------------------------------------------------------------------- */
/* Forward declarations                                                         */
/* -------------------------------------------------------------------------- */

static void app_init(void);
static void app_loop(void);

static void usb_start_read(void);
static void usb_send_line(const char * line);
static void usb_handle_rx_chunk(const uint8_t * data, uint32_t len);

static void process_line(char * line);

static void handle_heartbeat(const char * line);
static void handle_agent_ready(const char * line);
static void handle_detection_report(const char * line);

static void send_start_scan(void);
static void send_master_decision(master_decision_t decision, const char * reason);

static master_decision_t decide_from_window(bool rf_detected, float sds, const char ** reason_out);

static uint16_t next_seq(void);
static uint32_t millis(void);
static void delay_ms(uint32_t ms);

static uint16_t checksum_string(const char * s);
static uint16_t checksum_start_scan(uint16_t seq);
static uint16_t checksum_master_decision(const char * decision,
                                         const char * reason,
                                         uint16_t seq,
                                         uint32_t timestamp_ms);

static bool json_get_bool(const char * json, const char * key, bool default_value);
static float json_get_float(const char * json, const char * key, float default_value);
static bool json_has_type(const char * json, const char * type_name);

static const char * decision_to_string(master_decision_t decision);

/* -------------------------------------------------------------------------- */
/* Entry point                                                                  */
/* -------------------------------------------------------------------------- */

void hal_entry(void)
{
    app_init();

    while (1)
    {
        app_loop();
        delay_ms(5);
    }
}

/* -------------------------------------------------------------------------- */
/* USB callback                                                                 */
/* Register this callback in FSP USB PCDC stack.                                */
/* -------------------------------------------------------------------------- */

void usb_pcdc_callback(usb_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case USB_STATUS_CONFIGURED:
        {
            g_usb_configured = true;
            g_usb_write_done = true;
            break;
        }

        case USB_STATUS_READ_COMPLETE:
        {
            g_usb_read_done = true;

            /*
             * Some FSP versions provide transfer size in p_args.
             * If unavailable in your generated headers, keep full chunk size
             * and rely on newline parsing. CDC packets are text lines here.
             */
#ifdef USB_GET_USE_TRANSFER_SIZE
            g_usb_rx_size = p_args->data_size;
#else
            g_usb_rx_size = USB_RX_CHUNK_SIZE;
#endif
            break;
        }

        case USB_STATUS_WRITE_COMPLETE:
        {
            g_usb_write_done = true;
            break;
        }

        case USB_STATUS_DETACH:
        case USB_STATUS_SUSPEND:
        {
            g_usb_configured = false;
            g_usb_write_done = true;
            g_state = RA_STATE_WAIT_USB;
            break;
        }

        default:
        {
            break;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* App init and loop                                                            */
/* -------------------------------------------------------------------------- */

static void app_init(void)
{
    g_state = RA_STATE_BOOT;

    memset(g_usb_rx_chunk, 0, sizeof(g_usb_rx_chunk));
    memset(g_line_buffer, 0, sizeof(g_line_buffer));

    g_line_len = 0;
    g_last_heartbeat_ms = 0;
    g_last_start_scan_ms = 0;
    g_last_decision = MASTER_DECISION_CLEAR;

    R_USB_Open(&RA8_USB_CTRL, &RA8_USB_CFG);

    g_state = RA_STATE_WAIT_USB;
}

static void app_loop(void)
{
    g_ms_ticks += 5U;

    if (!g_usb_configured)
    {
        g_state = RA_STATE_WAIT_USB;
        return;
    }

    if (g_state == RA_STATE_WAIT_USB)
    {
        g_state = RA_STATE_READY;
        usb_start_read();
        send_start_scan();
        g_state = RA_STATE_SCANNING;
    }

    if (g_usb_read_done)
    {
        g_usb_read_done = false;
        usb_handle_rx_chunk(g_usb_rx_chunk, g_usb_rx_size);
        memset(g_usb_rx_chunk, 0, sizeof(g_usb_rx_chunk));
        usb_start_read();
    }

    if ((millis() - g_last_start_scan_ms) >= START_SCAN_PERIOD_MS)
    {
        if (g_state == RA_STATE_READY || g_state == RA_STATE_SCANNING)
        {
            send_start_scan();
        }
    }

    if (g_last_heartbeat_ms > 0U)
    {
        if ((millis() - g_last_heartbeat_ms) > HEARTBEAT_TIMEOUT_MS)
        {
            g_state = RA_STATE_FAULT;
            send_master_decision(MASTER_DECISION_CLEAR, "HEARTBEAT_TIMEOUT");
        }
    }
}

/* -------------------------------------------------------------------------- */
/* USB helpers                                                                  */
/* -------------------------------------------------------------------------- */

static void usb_start_read(void)
{
    if (!g_usb_configured)
    {
        return;
    }

    R_USB_Read(&RA8_USB_CTRL,
               g_usb_rx_chunk,
               USB_RX_CHUNK_SIZE,
               USB_CLASS_PCDC);
}

static void usb_send_line(const char * line)
{
    if (!g_usb_configured)
    {
        return;
    }

    if (line == NULL)
    {
        return;
    }

    uint32_t len = (uint32_t) strlen(line);
    if (len == 0U)
    {
        return;
    }

    uint32_t guard = 0;
    while (!g_usb_write_done && guard < 100U)
    {
        delay_ms(1);
        guard++;
    }

    g_usb_write_done = false;

    R_USB_Write(&RA8_USB_CTRL,
                (uint8_t *) line,
                len,
                USB_CLASS_PCDC);
}

static void usb_handle_rx_chunk(const uint8_t * data, uint32_t len)
{
    if (data == NULL || len == 0U)
    {
        return;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        char c = (char) data[i];

        if (c == '\0')
        {
            continue;
        }

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            g_line_buffer[g_line_len] = '\0';

            if (g_line_len > 0U)
            {
                process_line(g_line_buffer);
            }

            g_line_len = 0;
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
            g_line_len = 0;
            memset(g_line_buffer, 0, sizeof(g_line_buffer));
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Protocol processing                                                          */
/* -------------------------------------------------------------------------- */

static void process_line(char * line)
{
    if (line == NULL)
    {
        return;
    }

    if (json_has_type(line, "HEARTBEAT"))
    {
        handle_heartbeat(line);
        return;
    }

    if (json_has_type(line, "AGENT_READY"))
    {
        handle_agent_ready(line);
        return;
    }

    if (json_has_type(line, "DETECTION_REPORT"))
    {
        handle_detection_report(line);
        return;
    }

    if (json_has_type(line, "FAULT_REPORT"))
    {
        g_state = RA_STATE_FAULT;
        send_master_decision(MASTER_DECISION_CLEAR, "AGENT_FAULT");
        return;
    }
}

static void handle_heartbeat(const char * line)
{
    (void) line;
    g_last_heartbeat_ms = millis();

    if (g_state == RA_STATE_READY)
    {
        g_state = RA_STATE_SCANNING;
    }
}

static void handle_agent_ready(const char * line)
{
    (void) line;
    g_last_heartbeat_ms = millis();

    if (g_state == RA_STATE_WAIT_USB || g_state == RA_STATE_READY)
    {
        send_start_scan();
        g_state = RA_STATE_SCANNING;
    }
}

static void handle_detection_report(const char * line)
{
    bool rf_detected = json_get_bool(line, "rf_detected", false);
    float sds = json_get_float(line, "sds", 0.0f);

    const char * reason = "NO_RF";
    master_decision_t decision = decide_from_window(rf_detected, sds, &reason);

    g_last_decision = decision;

    if (decision == MASTER_DECISION_ALERT)
    {
        g_state = RA_STATE_ALERT;
    }
    else if (decision == MASTER_DECISION_CANDIDATE)
    {
        g_state = RA_STATE_CANDIDATE;
    }
    else
    {
        g_state = RA_STATE_SCANNING;
    }

    send_master_decision(decision, reason);
}

/* -------------------------------------------------------------------------- */
/* Decision logic                                                               */
/* -------------------------------------------------------------------------- */

static master_decision_t decide_from_window(bool rf_detected, float sds, const char ** reason_out)
{
    g_detect_history[g_detect_index] = rf_detected;
    g_sds_history[g_detect_index] = sds;
    g_detect_index = (g_detect_index + 1U) % DETECT_WINDOW_SIZE;

    uint32_t hit_count = 0;
    float sds_sum = 0.0f;
    uint32_t sds_count = 0;

    for (uint32_t i = 0; i < DETECT_WINDOW_SIZE; i++)
    {
        if (g_detect_history[i])
        {
            hit_count++;
            sds_sum += g_sds_history[i];
            sds_count++;
        }
    }

    float sds_avg = 0.0f;
    if (sds_count > 0U)
    {
        sds_avg = sds_sum / (float) sds_count;
    }

    if (hit_count >= DETECT_CONFIRM_COUNT && sds_avg >= SDS_ALERT_THRESHOLD)
    {
        *reason_out = "RF_2_OF_3_SDS_PASS";
        return MASTER_DECISION_ALERT;
    }

    if (rf_detected)
    {
        *reason_out = "RF_CANDIDATE";
        return MASTER_DECISION_CANDIDATE;
    }

    *reason_out = "NO_RF";
    return MASTER_DECISION_CLEAR;
}

/* -------------------------------------------------------------------------- */
/* Outgoing messages                                                            */
/* -------------------------------------------------------------------------- */

static void send_start_scan(void)
{
    char tx[TX_BUFFER_SIZE];

    uint16_t seq = next_seq();
    uint16_t checksum = checksum_start_scan(seq);

    snprintf(tx,
             sizeof(tx),
             "{\"type\":\"START_SCAN\",\"seq\":%u,\"checksum\":%u}\n",
             (unsigned int) seq,
             (unsigned int) checksum);

    usb_send_line(tx);
    g_last_start_scan_ms = millis();
}

static void send_master_decision(master_decision_t decision, const char * reason)
{
    char tx[TX_BUFFER_SIZE];

    uint16_t seq = next_seq();
    uint32_t timestamp_ms = millis();

    const char * decision_str = decision_to_string(decision);
    if (reason == NULL)
    {
        reason = "UNKNOWN";
    }

    uint16_t checksum = checksum_master_decision(decision_str,
                                                reason,
                                                seq,
                                                timestamp_ms);

    /*
     * JSON output order does not need to be sorted.
     * The checksum is calculated using Python-compatible sorted-key body:
     * {"decision":"...","reason":"...","seq":N,"timestamp_ms":T,"type":"MASTER_DECISION"}
     */
    snprintf(tx,
             sizeof(tx),
             "{\"type\":\"MASTER_DECISION\",\"seq\":%u,\"decision\":\"%s\",\"reason\":\"%s\",\"timestamp_ms\":%lu,\"checksum\":%u}\n",
             (unsigned int) seq,
             decision_str,
             reason,
             (unsigned long) timestamp_ms,
             (unsigned int) checksum);

    usb_send_line(tx);
}

/* -------------------------------------------------------------------------- */
/* Checksum helpers                                                             */
/* Must match Python:
 * json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
 * then sum UTF-8 bytes & 0xFFFF.
 * -------------------------------------------------------------------------- */

static uint16_t checksum_string(const char * s)
{
    uint32_t sum = 0U;

    if (s == NULL)
    {
        return 0U;
    }

    while (*s != '\0')
    {
        sum += (uint8_t) (*s);
        s++;
    }

    return (uint16_t) (sum & 0xFFFFU);
}

static uint16_t checksum_start_scan(uint16_t seq)
{
    char body[128];

    snprintf(body,
             sizeof(body),
             "{\"seq\":%u,\"type\":\"START_SCAN\"}",
             (unsigned int) seq);

    return checksum_string(body);
}

static uint16_t checksum_master_decision(const char * decision,
                                         const char * reason,
                                         uint16_t seq,
                                         uint32_t timestamp_ms)
{
    char body[TX_BUFFER_SIZE];

    snprintf(body,
             sizeof(body),
             "{\"decision\":\"%s\",\"reason\":\"%s\",\"seq\":%u,\"timestamp_ms\":%lu,\"type\":\"MASTER_DECISION\"}",
             decision,
             reason,
             (unsigned int) seq,
             (unsigned long) timestamp_ms);

    return checksum_string(body);
}

/* -------------------------------------------------------------------------- */
/* Tiny JSON helpers                                                            */
/* These are intentionally simple because the message format is controlled.      */
/* -------------------------------------------------------------------------- */

static bool json_has_type(const char * json, const char * type_name)
{
    char pattern[96];

    if (json == NULL || type_name == NULL)
    {
        return false;
    }

    snprintf(pattern, sizeof(pattern), "\"type\":\"%s\"", type_name);
    return strstr(json, pattern) != NULL;
}

static bool json_get_bool(const char * json, const char * key, bool default_value)
{
    char pattern_true[96];
    char pattern_false[96];

    if (json == NULL || key == NULL)
    {
        return default_value;
    }

    snprintf(pattern_true, sizeof(pattern_true), "\"%s\":true", key);
    snprintf(pattern_false, sizeof(pattern_false), "\"%s\":false", key);

    if (strstr(json, pattern_true) != NULL)
    {
        return true;
    }

    if (strstr(json, pattern_false) != NULL)
    {
        return false;
    }

    return default_value;
}

static float json_get_float(const char * json, const char * key, float default_value)
{
    char pattern[64];
    char * p = NULL;
    float value = default_value;

    if (json == NULL || key == NULL)
    {
        return default_value;
    }

    snprintf(pattern, sizeof(pattern), "\"%s\":", key);
    p = strstr(json, pattern);

    if (p == NULL)
    {
        return default_value;
    }

    p += strlen(pattern);

    if (sscanf(p, "%f", &value) == 1)
    {
        return value;
    }

    return default_value;
}

/* -------------------------------------------------------------------------- */
/* Misc helpers                                                                 */
/* -------------------------------------------------------------------------- */

static const char * decision_to_string(master_decision_t decision)
{
    switch (decision)
    {
        case MASTER_DECISION_ALERT:
            return "ALERT";

        case MASTER_DECISION_CANDIDATE:
            return "CANDIDATE";

        case MASTER_DECISION_CLEAR:
        default:
            return "CLEAR";
    }
}

static uint16_t next_seq(void)
{
    g_seq++;
    if (g_seq == 0U)
    {
        g_seq = 1U;
    }

    return g_seq;
}

static uint32_t millis(void)
{
    return g_ms_ticks;
}

static void delay_ms(uint32_t ms)
{
    R_BSP_SoftwareDelay(ms, BSP_DELAY_UNITS_MILLISECONDS);
}
```

## Orange Pi Test Commands

After flashing the RA8P1 firmware and connecting the Type-C cable:

```bash
dmesg | tail -50
ls /dev/ttyACM*
python3 -m serial.tools.miniterm /dev/ttyACM0 921600
```

Expected device:

```text
/dev/ttyACM0
```

The RK3588 project configuration uses:

```python
RA8P1_LINK_PORT = "/dev/ttyACM0"
RA8P1_LINK_BAUDRATE = 921600
```
