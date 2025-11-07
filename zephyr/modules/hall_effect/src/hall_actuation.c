#include <zephyr.h>
#include <logging/log.h>
#include "hall_effect.h"

LOG_MODULE_REGISTER(hall_actuation, LOG_LEVEL_INF);

extern void hall_socd_register_press(int key_id);
extern void hall_socd_register_release(int key_id);

void hall_actuation_handle_event(int key_id, bool is_pressed)
{
    if (is_pressed) {
        LOG_DBG("actuation: press %d", key_id);
        hall_socd_register_press(key_id);
    } else {
        LOG_DBG("actuation: release %d", key_id);
        hall_socd_register_release(key_id);
    }
}
