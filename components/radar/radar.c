#include "radar.h"

#include "le_common.h"

#include "asm/gpio.h"
#include "asm/uart_dev.h"

#include "stmgyro.h"

#include "screen.h"


#if LE_DEBUG_PRINT_EN
//#define log_info            y_printf
#define log_info(x, ...)  uprintf("[RADAR]" x " ", ## __VA_ARGS__)
#define log_info_hexdump  put_buf

#else
#define log_info(...)
#define log_info_hexdump(...)
#endif

static radar_event_callback_t redar_event_callback_handler = NULL;




static int pyuart_init_gpio(void)
{
    gpio_set_pull_up(IO_PORTB_08, 1);
    gpio_set_direction(IO_PORTB_08, 0);
    gpio_write(IO_PORTB_08, 1);
    
    gpio_set_pull_up(IO_PORTB_09, 1);
    gpio_set_direction(IO_PORTB_09, 0);
    gpio_write(IO_PORTB_09, 1);

    return 0;
}

#define UART_PORT           1
// #define UART_RX_SIZE        0x100
#define UART_TX_SIZE        256
#define UART_DB_SIZE        2048
#define UART_BAUD_RATE      115200

// static u8 pRxBuffer_static[UART_RX_SIZE] __attribute__((aligned(4)));       //rx memory
static u8 pTxBuffer_static[UART_TX_SIZE] __attribute__((aligned(4)));       //tx memory
static u8 devBuffer_static[UART_DB_SIZE] __attribute__((aligned(4)));       //dev DMA memory

static uart_bus_t *uart_bus = NULL;

static void uart_isr_cb(void *ut_bus, u32 status);

int radar_uart_init(void)
{
    struct uart_platform_data_t u_arg = {0};
    u_arg.tx_pin = IO_PORTA_01;
    u_arg.rx_pin = IO_PORTA_02;
    u_arg.rx_cbuf = devBuffer_static;
    u_arg.rx_cbuf_size = UART_DB_SIZE;
    u_arg.frame_length = UART_DB_SIZE;
    u_arg.rx_timeout = 20;  //ms,兼容波特率较低
    u_arg.isr_cbfun = uart_isr_cb;
    u_arg.baud = UART_BAUD_RATE;
    u_arg.is_9bit = 0;

    uart_bus = uart_dev_open(&u_arg, UART_PORT);

    if (uart_bus != NULL) {
        log_info("Init Done\n");
        return 0;
    } else {
        log_info("Init Error\n");
        return -1;
    }

    return 0;
}

int radar_uart_write(void)
{
    
}

#define RADAR_TYPE_POINT_CLOUD  0x00000001

typedef uint64_t radar_cloud_unit_t;

typedef struct __attribute__((aligned(1))) {
    uint32_t type;
    uint32_t length;
    uint16_t num;
    uint16_t xyzQFormat;
    
    // radar_cloud_unit_t *units;
} radar_cloud_pack_t;


#define RADAR_TYPE_FLIGHT_PATH  0x00000003

typedef struct __attribute__((aligned(1))) {
    int16_t x;
    int16_t y;
    int16_t xd;
    int16_t yd;
    int16_t xsize;
    int16_t ysize;
    uint8_t id;
    uint8_t peakValdB;
} radar_flight_unit_t;

typedef struct __attribute__((aligned(1))) {
    uint32_t type;
    uint32_t length;
    uint16_t num;
    uint16_t xyzQFormat;
} radar_flight_pack_t;



typedef struct __attribute__((aligned(1))) {
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

// Return Sizeof radar_cloud_point_t;
static uint8_t *radar_unpackage_point_cloud(uint8_t *pdata)
{
    radar_cloud_pack_t pack = {0};
    memcpy(&pack, pdata, sizeof(pack));
    // log_info("Cloud Points:");
    // log_info_hexdump(pdata + sizeof(pack), pack.num);
    return pdata + sizeof(radar_cloud_pack_t) + pack.num * sizeof(radar_cloud_unit_t);
}

static void *radar_copy_flight_path(radar_flight_unit_t *unit, uint8_t *pdata)
{
    int16_t tmp = 0;

    memcpy(&tmp, pdata, 2);
    unit->x = tmp;
    pdata += 2;
    memcpy(&tmp, pdata, 2);
    unit->y = tmp;
    pdata += 2;
    memcpy(&tmp, pdata, 2);
    unit->xd = tmp;
    pdata += 2;
    memcpy(&tmp, pdata, 2);
    unit->yd = tmp;
    pdata += 2;
    memcpy(&tmp, pdata, 2);
    unit->xsize = tmp;
    pdata += 2;
    memcpy(&tmp, pdata, 2);
    unit->ysize = tmp;
    pdata += 2;
    unit->id = *(pdata++);
    unit->peakValdB = *(pdata++);
}


static float radar_abs(float input)
{
    return input >= 0 ? input : -input;
}

#define XRANGE_DANGER       2.5
#define SECOND_DANGER       1

#define XRANGE_WARNNING     5
#define SECOND_WARNNING     3
static void radar_approach_sense(int16_t x, int16_t y, int16_t dx, int16_t dy)
{
    if (dy >= 0)    // Will Not Touch Zero Line
        return;
    
    float sec = (float)y / radar_abs(dy);
    float zx  = (float)x / 128 + (float)dx / 128 * sec;

    // ulog("<%d, %d>\r\n", (int)(sec * 1000), (int)(zx * 1000));

    if (sec <= SECOND_DANGER && radar_abs(zx) <= XRANGE_DANGER) {
        // Under Danger
        screen_display(SCREEN_PRIORITY_RADAR_DANGER, PYUART_IMG_BREAK, 2000);       // TODO: 入侵警报入口 <<!!!>>
    } else if (sec <= SECOND_WARNNING && radar_abs(zx) <= XRANGE_WARNNING) {
        // Under Warning
        screen_display(SCREEN_PRIORITY_RADAR_WARNNING, PYUART_IMG_WARNING, 2000);   // TODO: 入侵预警入口 <<!!!>>
    }
}

static uint8_t *radar_unpackage_flight_path(uint8_t *pdata)
{
    radar_flight_pack_t pack = {0};
    memcpy(&pack, pdata, sizeof(pack));

    pdata += sizeof(pack);
    for (int iunit = 0; iunit < pack.num; iunit++) {
        radar_flight_unit_t unit = {0};
        radar_copy_flight_path(&unit, pdata);
        // ulog("(%d,%d,%d,%d)\r\n", unit.x, unit.y, unit.xd, unit.yd);
        radar_approach_sense(unit.x, unit.y, unit.xd, unit.yd);
        pdata += sizeof(radar_flight_unit_t);
    }
    return pdata;
}

static const uint8_t magic_header[8] = { 0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07 };
static void radar_unpackage(uint8_t *pdata, uint16_t len)
{
    if (len < 40)
        return ;

    radar_frame_t frame = {0};
    memcpy(&frame, pdata, sizeof(radar_frame_t));
    if (0 != memcmp(frame.header, magic_header, 8)) {
        log_info("Frame Magic Error\n");
        return;
    }

    if (frame.totalPacketLen != len) {
        log_info("Frame Total Length Error\n");
        return;
    }

    // log_info("Frame Count: %d\n", frame.subFrameNumber);
    uint8_t *pframe = pdata + sizeof(radar_frame_t);
    for (int iframe = 0; iframe < 20/*frame.subFrameNumber*/; iframe++) {
        uint32_t type = 0;
        memcpy(&type, pframe, 4);
        switch (type) {
            case RADAR_TYPE_POINT_CLOUD:
                pframe = radar_unpackage_point_cloud(pframe);
                break;
            case RADAR_TYPE_FLIGHT_PATH:
                pframe = radar_unpackage_flight_path(pframe);
                break;
            default:
                // log_info("Frame Sub Type Error: %d\n", type);
                return;
        }
    }
}

static uint8_t package[2048] = {0};
static void uart_isr_cb(void *ut_bus, u32 status)
{
    // struct sys_event e;
    // printf("{##%d}", status);

    if (status == UT_RX_OT) {
        u32 len = uart_bus->read(package, 2048, 0);
        // log_info("Radar Receive %d Byte Data\n", len);

        // log_info_hexdump(package, len);
        radar_unpackage(package, len);
    }

    // if (/*status == UT_RX || */status == UT_RX_OT) {
    //     log_info("RECV OT\n");
    //     u8 line[128] = {0};
    //     log_info("%s\n", line);
    // }
}

int radar_init(radar_event_callback_t handle)
{
    redar_event_callback_handler = handle;

    radar_uart_init();
    pyuart_init_gpio();

    return 0;
}




