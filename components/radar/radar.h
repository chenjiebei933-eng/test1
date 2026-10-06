#ifndef RADAR_H
#define RADAR_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Raw signed fields from one flight-path target; no unit conversion. */
typedef void (*radar_event_callback_t)(int16_t x, int16_t y,
                                      int16_t xd, int16_t yd);

/*
 * Start UART1 receive on GPIO18 at 460800 baud (8N1), with no TX pin.
 * Valid frames/targets are logged through the existing UART0 console.
 * handle may be NULL; otherwise it runs once per target in the radar task
 * after the complete frame has been checked. Keep callbacks short.
 * A second initialization, or an already occupied UART1, returns
 * ESP_ERR_INVALID_STATE. On failure, resources acquired here are released.
 */
esp_err_t radar_init(radar_event_callback_t handle);

#ifdef __cplusplus
}
#endif

#endif /* RADAR_H */
