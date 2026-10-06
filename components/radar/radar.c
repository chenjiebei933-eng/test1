#include "radar.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define RADAR_UART_PORT          UART_NUM_1
#define RADAR_RX_GPIO            GPIO_NUM_18
#define RADAR_BAUD_RATE          460800
#define RADAR_RX_BUFFER_SIZE     4096
#define RADAR_EVENT_QUEUE_SIZE   16
#define RADAR_IDLE_TIMEOUT_MS    1000
#define RADAR_STATUS_PERIOD_MS   2000
#define RADAR_READ_SIZE          256
#define RADAR_MAX_FRAME_SIZE     2048
#define RADAR_FRAME_HEADER_SIZE  40
#define RADAR_PACK_HEADER_SIZE   12
#define RADAR_CLOUD_UNIT_SIZE    8
#define RADAR_FLIGHT_UNIT_SIZE   14
#define RADAR_TYPE_POINT_CLOUD   0x00000001U
#define RADAR_TYPE_FLIGHT_PATH   0x00000003U

static const char *TAG = "radar";
static const uint8_t magic_header[8] = {
    0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07
};
static radar_event_callback_t radar_event_callback;
static QueueHandle_t radar_uart_queue;
static uint8_t radar_stream[RADAR_MAX_FRAME_SIZE];
static size_t radar_stream_len;

/* Only the receive task updates these counters. Report even when no valid
 * packet arrives, so a silent input is distinguishable from a stopped app. */
static struct {
    uint32_t rx_bytes, frames, targets;
    uint32_t breaks, frame_errors, parity_errors, overflows;
    uint32_t last_frames;
    TickType_t last_report;
    uint8_t sample[16];
    size_t sample_len;
} radar_stats;

/* Wire layouts are unchanged; decode bytes explicitly to avoid alignment
 * and native-endian assumptions when UART reads split fields. */
typedef struct {
    uint8_t header[8];
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} radar_frame_t;

typedef struct {
    uint32_t type;
    uint32_t length;
    uint16_t num;
    uint16_t xyzQFormat;
} radar_pack_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t xd;
    int16_t yd;
    int16_t xsize;
    int16_t ysize;
    uint8_t id;
    uint8_t peakValdB;
} radar_flight_unit_t;

static uint16_t radar_u16_le(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t radar_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8)
           | ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static int16_t radar_i16_le(const uint8_t *data)
{
    const uint16_t raw = radar_u16_le(data);
    const int32_t value = raw <= INT16_MAX ? (int32_t)raw : (int32_t)raw - 65536;
    return (int16_t)value;
}

static void radar_read_frame(radar_frame_t *frame, const uint8_t *data)
{
    memcpy(frame->header, data, sizeof(frame->header));
    frame->version = radar_u32_le(data + 8);
    frame->totalPacketLen = radar_u32_le(data + 12);
    frame->platform = radar_u32_le(data + 16);
    frame->frameNumber = radar_u32_le(data + 20);
    frame->timeCpuCycles = radar_u32_le(data + 24);
    frame->numDectedObj = radar_u32_le(data + 28);
    frame->numTLVs = radar_u32_le(data + 32);
    frame->subFrameNumber = radar_u32_le(data + 36);
}

static void radar_read_pack(radar_pack_t *pack, const uint8_t *data)
{
    pack->type = radar_u32_le(data);
    pack->length = radar_u32_le(data + 4);
    pack->num = radar_u16_le(data + 8);
    pack->xyzQFormat = radar_u16_le(data + 10);
}

static void radar_copy_flight_path(radar_flight_unit_t *unit, const uint8_t *data)
{
    unit->x = radar_i16_le(data);
    unit->y = radar_i16_le(data + 2);
    unit->xd = radar_i16_le(data + 4);
    unit->yd = radar_i16_le(data + 6);
    unit->xsize = radar_i16_le(data + 8);
    unit->ysize = radar_i16_le(data + 10);
    unit->id = data[12];
    unit->peakValdB = data[13];
}

/* Return false only for malformed frames, so the stream can search again
 * from the next byte. An unsupported TLV ends this frame without targets. */
static bool radar_unpackage(const uint8_t *data, size_t len)
{
    if (len < RADAR_FRAME_HEADER_SIZE
        || memcmp(data, magic_header, sizeof(magic_header)) != 0) {
        return false;
    }

    radar_frame_t frame;
    radar_read_frame(&frame, data);
    if (frame.totalPacketLen != len
        || frame.numTLVs > (len - RADAR_FRAME_HEADER_SIZE) / RADAR_PACK_HEADER_SIZE) {
        ESP_LOGW(TAG, "Invalid frame length or TLV count");
        return false;
    }

    /* Validate every known TLV before emitting any valid-frame/target data.
     * As in the original parser, traversal uses num and the unit size.
     * Do not reinterpret length or xyzQFormat, or trim flight targets to
     * numDectedObj: the header can describe point-cloud detections instead. */
    size_t offset = RADAR_FRAME_HEADER_SIZE;
    for (uint32_t i = 0; i < frame.numTLVs; ++i) {
        const size_t remaining = len - offset;
        if (remaining < RADAR_PACK_HEADER_SIZE) {
            ESP_LOGW(TAG, "Truncated TLV header");
            return false;
        }
        radar_pack_t pack;
        radar_read_pack(&pack, data + offset);
        size_t unit_size;
        switch (pack.type) {
            case RADAR_TYPE_POINT_CLOUD:
                unit_size = RADAR_CLOUD_UNIT_SIZE;
                break;
            case RADAR_TYPE_FLIGHT_PATH:
                unit_size = RADAR_FLIGHT_UNIT_SIZE;
                break;
            default:
                ESP_LOGW(TAG, "Unsupported TLV type=%" PRIu32 "; skipping frame",
                         pack.type);
                return true;
        }
        if (pack.num > (remaining - RADAR_PACK_HEADER_SIZE) / unit_size) {
            ESP_LOGW(TAG, "Truncated TLV targets");
            return false;
        }
        offset += RADAR_PACK_HEADER_SIZE + (size_t)pack.num * unit_size;
    }

    ++radar_stats.frames;
    ESP_LOGI(TAG, "frame=%" PRIu32 " objects=%" PRIu32 " tlvs=%" PRIu32,
             frame.frameNumber, frame.numDectedObj, frame.numTLVs);

    offset = RADAR_FRAME_HEADER_SIZE;
    for (uint32_t i = 0; i < frame.numTLVs; ++i) {
        radar_pack_t pack;
        radar_read_pack(&pack, data + offset);
        offset += RADAR_PACK_HEADER_SIZE;
        if (pack.type == RADAR_TYPE_POINT_CLOUD) {
            offset += (size_t)pack.num * RADAR_CLOUD_UNIT_SIZE;
            continue;
        }
        for (uint16_t target = 0; target < pack.num; ++target) {
            radar_flight_unit_t unit;
            radar_copy_flight_path(&unit, data + offset);
            ++radar_stats.targets;
            ESP_LOGI(TAG, "id=%u x=%d y=%d xd=%d yd=%d", (unsigned)unit.id,
                     (int)unit.x, (int)unit.y, (int)unit.xd, (int)unit.yd);
            if (radar_event_callback != NULL) {
                radar_event_callback(unit.x, unit.y, unit.xd, unit.yd);
            }
            offset += RADAR_FLIGHT_UNIT_SIZE;
        }
    }
    return true;
}

static void radar_reset_stream(void)
{
    radar_stream_len = 0;
}

static void radar_drop_bytes(size_t count)
{
    radar_stream_len -= count;
    if (radar_stream_len != 0) {
        memmove(radar_stream, radar_stream + count, radar_stream_len);
    }
}

static void radar_process_stream(void)
{
    while (radar_stream_len >= sizeof(magic_header)) {
        size_t start = 0;
        while (start + sizeof(magic_header) <= radar_stream_len
               && memcmp(radar_stream + start, magic_header, sizeof(magic_header)) != 0) {
            ++start;
        }
        /* Without a full magic word, retain the last seven bytes so a
         * header split across UART reads can still be recognized. */
        radar_drop_bytes(start);
        if (radar_stream_len < 16) {
            return;
        }

        const uint32_t packet_len = radar_u32_le(radar_stream + 12);
        if (packet_len < RADAR_FRAME_HEADER_SIZE || packet_len > RADAR_MAX_FRAME_SIZE) {
            ESP_LOGW(TAG, "Invalid packet length=%" PRIu32, packet_len);
            radar_drop_bytes(1);
            continue;
        }
        if (radar_stream_len < packet_len) {
            return;
        }
        if (radar_unpackage(radar_stream, packet_len)) {
            radar_drop_bytes(packet_len);
        } else {
            radar_drop_bytes(1);
        }
    }
}

static void radar_feed(const uint8_t *data, size_t len)
{
    while (len != 0) {
        size_t available = sizeof(radar_stream) - radar_stream_len;
        if (available == 0) {
            /* Normally a full buffer is already parsed, because accepted
             * totalPacketLen never exceeds the buffer. Recover defensively. */
            ESP_LOGW(TAG, "Frame buffer full; resynchronizing");
            radar_reset_stream();
            available = sizeof(radar_stream);
        }
        const size_t count = len < available ? len : available;
        memcpy(radar_stream + radar_stream_len, data, count);
        radar_stream_len += count;
        data += count;
        len -= count;
        radar_process_stream();
    }
}

static void radar_handle_idle_timeout(void)
{
    if (radar_stream_len == 0) {
        return;
    }
    /* A 2048-byte frame takes about 45 ms at 460800/8N1, and generates
     * receive events along the way. Only recover after a full second
     * without events, never on UART_DATA.timeout_flag. Scan stale bytes
     * for complete later frames before discarding incomplete remnants. */
    ESP_LOGW(TAG, "Incomplete frame timeout; resynchronizing");
    while (radar_stream_len != 0) {
        radar_drop_bytes(1);
        radar_process_stream();
    }
}

static void radar_reset_receiver(void)
{
    const esp_err_t err = uart_flush_input(RADAR_UART_PORT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART flush failed: %s", esp_err_to_name(err));
    }
    if (radar_uart_queue != NULL) {
        xQueueReset(radar_uart_queue);
    }
    radar_reset_stream();
}

static void radar_handle_uart_event(const uart_event_t *event)
{
    switch (event->type) {
        case UART_DATA: {
            uint8_t data[RADAR_READ_SIZE];
            size_t remaining = event->size;
            while (remaining != 0) {
                const size_t requested = remaining < sizeof(data) ? remaining : sizeof(data);
                const int received = uart_read_bytes(RADAR_UART_PORT, data,
                                                     (uint32_t)requested, 0);
                if (received < 0) {
                    ESP_LOGW(TAG, "UART read failed; resynchronizing");
                    radar_reset_receiver();
                    break;
                }
                if (received == 0) {
                    break;
                }
                radar_stats.rx_bytes += (uint32_t)received;
                const size_t space = sizeof(radar_stats.sample) - radar_stats.sample_len;
                const size_t sample_size = (size_t)received < space ? (size_t)received : space;
                memcpy(radar_stats.sample + radar_stats.sample_len, data, sample_size);
                radar_stats.sample_len += sample_size;
                radar_feed(data, (size_t)received);
                remaining -= (size_t)received;
            }
            break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
        case UART_FRAME_ERR:
        case UART_PARITY_ERR:
        case UART_BREAK:
            if (event->type == UART_BREAK) {
                ++radar_stats.breaks;
            } else if (event->type == UART_FRAME_ERR) {
                ++radar_stats.frame_errors;
            } else if (event->type == UART_PARITY_ERR) {
                ++radar_stats.parity_errors;
            } else {
                ++radar_stats.overflows;
            }
            ESP_LOGW(TAG, "UART receive error=%d; resynchronizing", (int)event->type);
            radar_reset_receiver();
            break;
        default:
            break;
    }
}

static void radar_report_status(void)
{
    const TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - radar_stats.last_report) < pdMS_TO_TICKS(RADAR_STATUS_PERIOD_MS)) {
        return;
    }
    radar_stats.last_report = now;
    ESP_LOGI(TAG, "status baud=%d rx_bytes=%" PRIu32 " frames=%" PRIu32
             " targets=%" PRIu32 " break=%" PRIu32 " frame_err=%" PRIu32
             " parity_err=%" PRIu32 " overflow=%" PRIu32 " rx_level=%d",
             RADAR_BAUD_RATE, radar_stats.rx_bytes, radar_stats.frames,
             radar_stats.targets, radar_stats.breaks, radar_stats.frame_errors,
             radar_stats.parity_errors, radar_stats.overflows, gpio_get_level(RADAR_RX_GPIO));
    if (radar_stats.frames == radar_stats.last_frames && radar_stats.sample_len != 0) {
        char hex[sizeof(radar_stats.sample) * 3 + 1];
        for (size_t i = 0; i < radar_stats.sample_len; ++i) {
            snprintf(hex + i * 3, sizeof(hex) - i * 3, "%02X ", (unsigned)radar_stats.sample[i]);
        }
        ESP_LOGW(TAG, "RX sample (no decoded frame in interval): %s", hex);
    }
    radar_stats.last_frames = radar_stats.frames;
    radar_stats.sample_len = 0;
}

static void radar_receive_task(void *argument)
{
    (void)argument;
    uart_event_t event;
    for (;;) {
        if (xQueueReceive(radar_uart_queue, &event,
                          pdMS_TO_TICKS(RADAR_IDLE_TIMEOUT_MS)) == pdTRUE) {
            radar_handle_uart_event(&event);
        } else {
            radar_handle_idle_timeout();
        }
        radar_report_status();
    }
}

esp_err_t radar_init(radar_event_callback_t handle)
{
    if (uart_is_driver_installed(RADAR_UART_PORT)) {
        return ESP_ERR_INVALID_STATE;
    }

    radar_uart_queue = NULL;
    radar_event_callback = NULL;
    radar_reset_stream();
    memset(&radar_stats, 0, sizeof(radar_stats));
    radar_stats.last_report = xTaskGetTickCount();
    esp_err_t err = uart_driver_install(RADAR_UART_PORT, RADAR_RX_BUFFER_SIZE,
                                       0, RADAR_EVENT_QUEUE_SIZE, &radar_uart_queue, 0);
    if (err != ESP_OK) {
        radar_uart_queue = NULL;
        return err;
    }

    const uart_config_t config = {
        .baud_rate = RADAR_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    err = uart_param_config(RADAR_UART_PORT, &config);
    if (err != ESP_OK) {
        goto fail;
    }
    err = uart_set_pin(RADAR_UART_PORT, UART_PIN_NO_CHANGE, RADAR_RX_GPIO,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        goto fail;
    }

    radar_event_callback = handle;
    if (xTaskCreate(radar_receive_task, "radar_rx", 4096, NULL, 5, NULL) != pdPASS) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }
    ESP_LOGI(TAG, "UART1 RX=GPIO18 %d baud 8N1; feedback on UART0 console", RADAR_BAUD_RATE);
    return ESP_OK;

fail:
    radar_event_callback = NULL;
    const esp_err_t cleanup_err = uart_driver_delete(RADAR_UART_PORT);
    if (cleanup_err != ESP_OK) {
        ESP_LOGE(TAG, "UART cleanup failed: %s", esp_err_to_name(cleanup_err));
    }
    radar_uart_queue = NULL;
    radar_reset_stream();
    return err;
}




