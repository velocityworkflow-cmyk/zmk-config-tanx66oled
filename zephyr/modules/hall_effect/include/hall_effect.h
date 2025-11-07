#ifndef HALL_EFFECT_H
#define HALL_EFFECT_H

#include <zephyr.h>
#include <stdint.h>
#include <stdbool.h>

/* Module init (registered via SYS_INIT in driver) */
void hall_module_init(const struct device *dev);

/* Small wrappers to emit; implement these to integrate with ZMK event emitter.
   The example SOCD code calls hall_emit_key_position(key_pos, pressed). */
void hall_emit_keycode(uint16_t keycode, bool pressed);
void hall_emit_key_position(uint16_t key_pos, bool pressed);

/* Actuation pipeline entry used by driver */
void hall_actuation_handle_event(int key_id, bool is_pressed);

/* Rapid / SOCD hooks are defined in their own files */
void hall_rapid_on_transition(int key_id);
void hall_rapid_stop(int key_id);

/* SOCD sensor count hint */
void hall_socd_register_sensor_count(int count);

#endif /* HALL_EFFECT_H */
