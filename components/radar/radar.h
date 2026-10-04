#ifndef __RADAR_H__
#define __RADAR_H__

#include "system/app_core.h"
#include "system/includes.h"

#include "app_config.h"

#define RADAR_USE_POINT_CLOUD   0
#define RADAR USE_FLIGHT_PATH   1

typedef void (*radar_event_callback_t)(int16_t x, int16_t y, int16_t xd, int16_t yd);

int radar_init(radar_event_callback_t handle);

#endif  // __RADAR_H__
