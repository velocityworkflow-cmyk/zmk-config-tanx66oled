#include <zephyr.h>
#include <logging/log.h>
#include "hall_effect.h"

LOG_MODULE_REGISTER(hall_socd, LOG_LEVEL_INF);

#define MAX_HALL_SENSORS 32

static bool sensor_state[MAX_HALL_SENSORS];
static int sensor_count = 0;

/* policy: 0 = neutral, 1 = up_priority, 2 = last */
static int policy = CONFIG_HALL_SOCD_POLICY;
static int last_pressed_id = -1;

void hall_socd_register_sensor_count(int count)
{
    if (count > MAX_HALL_SENSORS) count = MAX_HALL_SENSORS;
    sensor_count = count;
    for (int i = 0; i < sensor_count; ++i) sensor_state[i] = false;
}

/* Minimal SOCD: emit the sensor's mapped key-position state.
   If you have axis pairs (left/right), implement pair logic here using `policy`. */
static void emit_state(int id)
{
    /* Emit as key position so it integrates with keymap/layers */
    hall_emit_key_position((uint16_t)id, sensor_state[id]);
}

void hall_socd_register_press(int key_id)
{
    if (key_id < 0 || key_id >= sensor_count) return;
    sensor_state[key_id] = true;
    last_pressed_id = key_id;
    emit_state(key_id);
}

void hall_socd_register_release(int key_id)
{
    if (key_id < 0 || key_id >= sensor_count) return;
    sensor_state[key_id] = false;
    emit_state(key_id);
}
