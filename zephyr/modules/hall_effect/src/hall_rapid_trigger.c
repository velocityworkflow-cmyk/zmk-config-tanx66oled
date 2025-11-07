#include <zephyr.h>
#include <logging/log.h>
#include "hall_effect.h"

LOG_MODULE_REGISTER(hall_rapid, LOG_LEVEL_INF);

#define MAX_SENSORS 32

struct rapid_state {
    uint32_t last_transition_ms;
    bool in_rapid;
    struct k_timer repeat_timer;
    int key_id;
};

static struct rapid_state rstates[MAX_SENSORS];

static void repeat_timeout(struct k_timer *t)
{
    struct rapid_state *rs = CONTAINER_OF(t, struct rapid_state, repeat_timer);
    /* pulse: press then release */
    hall_emit_key_position(rs->key_id, true);
    hall_emit_key_position(rs->key_id, false);
}

void hall_rapid_on_transition(int key_id)
{
    uint32_t now = k_uptime_get_32();
    struct rapid_state *rs = &rstates[key_id];
    uint32_t dt = now - rs->last_transition_ms;
    rs->last_transition_ms = now;

    /* quick heuristic: if two transitions happen faster than 4 * sample interval -> rapid */
    if (dt < (CONFIG_HALL_ADC_SAMPLE_INTERVAL_MS * 4)) {
        if (!rs->in_rapid) {
            rs->in_rapid = true;
            rs->key_id = key_id;
            k_timer_init(&rs->repeat_timer, repeat_timeout, NULL);
            k_timer_user_data_set(&rs->repeat_timer, (void *)rs);
            k_timer_start(&rs->repeat_timer, K_MSEC(100), K_MSEC(100));
            LOG_INF("rapid mode start %d", key_id);
        }
    }
}

void hall_rapid_stop(int key_id)
{
    struct rapid_state *rs = &rstates[key_id];
    if (rs->in_rapid) {
        k_timer_stop(&rs->repeat_timer);
        rs->in_rapid = false;
        LOG_INF("rapid mode stop %d", key_id);
    }
}
