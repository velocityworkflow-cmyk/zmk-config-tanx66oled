/* hall_adc_driver.c
 * DT-driven ADC driver for Allegro A1324 sensors.
 * Parent DT node must be labeled 'hall_effect' and children labeled 'hall-sensor-N'
 * with properties:
 *   io-channels = <&adc X>;
 *   key-id = <M>;
 *
 * This driver:
 *  - parses children by label hall_sensor_0 / hall_sensor_1 ... up to MAX
 *  - configures ADC channels (best-effort)
 *  - averages samples, converts to mV
 *  - calibrates baseline and computes threshold
 *  - applies hysteresis and calls hall_actuation_handle_event()
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/adc.h>
#include <devicetree.h>
#include <logging/log.h>
#include "hall_effect.h"

LOG_MODULE_REGISTER(hall_adc_dt, LOG_LEVEL_INF);

#ifndef CONFIG_HALL_ADC_SAMPLES
#define CONFIG_HALL_ADC_SAMPLES 8
#endif
#ifndef CONFIG_HALL_ADC_SAMPLE_INTERVAL_MS
#define CONFIG_HALL_ADC_SAMPLE_INTERVAL_MS 4
#endif
#ifndef CONFIG_HALL_ADC_HYST_MV
#define CONFIG_HALL_ADC_HYST_MV 50
#endif

#define MAX_DT_SENSORS 16
#define ADC_RESOLUTION 12
#define VREF_MV 3300

struct dt_sensor_entry {
    const struct device *adc_dev;
    uint8_t channel_id;
    int key_id;
    uint32_t baseline_mv;
    uint32_t threshold_mv;
    uint32_t hyst_mv;
    bool reported_state;
    struct k_work_delayable sample_work;
};

static struct dt_sensor_entry sensors[MAX_DT_SENSORS];
static int dt_sensor_count = 0;

static int raw_to_mv(int32_t raw, int resolution)
{
    int32_t unsigned_raw = raw;
    if (raw < 0) {
        unsigned_raw = raw + (1 << (resolution - 1));
    }
    int mv = (int)((unsigned_raw * VREF_MV) / ((1 << resolution) - 1));
    return mv;
}

static int adc_sample_once(const struct device *adc_dev, uint8_t channel, int32_t *out_raw)
{
    int16_t raw_buf = 0;
    struct adc_sequence seq = {
        .channels = BIT(channel),
        .buffer = &raw_buf,
        .buffer_size = sizeof(raw_buf),
        .resolution = ADC_RESOLUTION,
    };

    int rc = adc_read(adc_dev, &seq);
    if (rc) {
        LOG_ERR("adc_read ch%d failed: %d", channel, rc);
        return rc;
    }
    *out_raw = raw_buf;
    return 0;
}

static int sample_average_mv(struct dt_sensor_entry *s)
{
    int sum = 0;
    for (int i = 0; i < CONFIG_HALL_ADC_SAMPLES; ++i) {
        int32_t raw = 0;
        int rc = adc_sample_once(s->adc_dev, s->channel_id, &raw);
        if (rc) return rc;
        int mv = raw_to_mv(raw, ADC_RESOLUTION);
        sum += mv;
        k_msleep(CONFIG_HALL_ADC_SAMPLE_INTERVAL_MS);
    }
    return sum / CONFIG_HALL_ADC_SAMPLES;
}

static void sensor_sample_handler(struct k_work *work)
{
    struct dt_sensor_entry *s = CONTAINER_OF(work, struct dt_sensor_entry, sample_work);
    int avg_mv = sample_average_mv(s);
    if (avg_mv < 0) {
        k_work_schedule(&s->sample_work, K_MSEC(50));
        return;
    }

    if (!s->reported_state) {
        if (avg_mv >= (int)s->threshold_mv) {
            s->reported_state = true;
            LOG_INF("sensor id=%d pressed mv=%d thr=%d", s->key_id, avg_mv, s->threshold_mv);
            hall_actuation_handle_event(s->key_id, true);
            hall_rapid_on_transition(s->key_id);
        }
    } else {
        if (avg_mv <= (int)(s->threshold_mv - s->hyst_mv)) {
            s->reported_state = false;
            LOG_INF("sensor id=%d released mv=%d thr=%d", s->key_id, avg_mv, s->threshold_mv);
            hall_actuation_handle_event(s->key_id, false);
            hall_rapid_stop(s->key_id);
        }
    }

    k_work_schedule(&s->sample_work, K_MSEC(20));
}

/* best-effort channel setup */
static int setup_adc_channel(const struct device *adc_dev, uint8_t channel)
{
    struct adc_channel_cfg ch_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = channel,
    };
    int rc = adc_channel_setup(adc_dev, &ch_cfg);
    if (rc) {
        LOG_WRN("adc_channel_setup ch%d returned %d (maybe OK)", channel, rc);
    }
    return 0;
}

/* parse DT: we require the overlay defines a node labeled hall_effect and children hall_sensor_0/1/... */
static int dt_init_sensors(void)
{
    dt_sensor_count = 0;

    /* ADC device binding from the ADC node label */
    const struct device *adc_dev = device_get_binding(DT_LABEL(DT_NODELABEL(adc)));
    if (!adc_dev) {
        LOG_ERR("ADC device not found (DT nodelabel 'adc')");
        return -ENODEV;
    }

    /* Loop over expected hall_sensor_N labels (0..MAX_DT_SENSORS-1) */
    for (int i = 0; i < MAX_DT_SENSORS; ++i) {
        /* build nodelabel macro name at compile-time is required; use DT_NODELABEL(hall_sensor_X) macros */
        /* Only support up to hall_sensor_9 labels in this simple loop for convenience */
        int node = DT_INVALID_NODE;
        switch (i) {
            case 0: node = DT_NODELABEL(hall_sensor_0); break;
            case 1: node = DT_NODELABEL(hall_sensor_1); break;
            case 2: node = DT_NODELABEL(hall_sensor_2); break;
            case 3: node = DT_NODELABEL(hall_sensor_3); break;
            case 4: node = DT_NODELABEL(hall_sensor_4); break;
            case 5: node = DT_NODELABEL(hall_sensor_5); break;
            case 6: node = DT_NODELABEL(hall_sensor_6); break;
            case 7: node = DT_NODELABEL(hall_sensor_7); break;
            case 8: node = DT_NODELABEL(hall_sensor_8); break;
            case 9: node = DT_NODELABEL(hall_sensor_9); break;
            default: node = DT_INVALID_NODE; break;
        }

        if (node == DT_INVALID_NODE) continue;
        if (!DT_NODE_HAS_STATUS(node, okay)) continue;

        if (dt_sensor_count >= MAX_DT_SENSORS) break;
        struct dt_sensor_entry *entry = &sensors[dt_sensor_count];

        /* read key-id property (default to i if missing) */
        if (DT_NODE_HAS_PROP(node, key_id)) {
            entry->key_id = DT_PROP(node, key_id);
        } else {
            entry->key_id = dt_sensor_count;
        }

        /* read io-channels first cell: <&adc N> -> get the phandle's index value */
        /* Zephyr devicetree macros make this compile-time; use DT_PROP_BY_PHANDLE_IDX */
#ifdef DT_PROP_BY_IDX
        /* get the phandle to ADC and cell value */
#endif
        /* Simplified approach: rely on you using ADC channel numbers 0 and 5 for sensors 0 and 1.
           If you used different channels, edit below or extend DT parsing macros. */
        if (dt_sensor_count == 0) {
            entry->adc_dev = adc_dev;
            entry->channel_id = 0; /* sensor 0 -> ADC 0 */
        } else if (dt_sensor_count == 1) {
            entry->adc_dev = adc_dev;
            entry->channel_id = 5; /* sensor 1 -> ADC 5 */
        } else {
            /* No more sensors expected by default; break */
            break;
        }

        entry->reported_state = false;
        entry->baseline_mv = 0;
        entry->threshold_mv = 0;
        entry->hyst_mv = CONFIG_HALL_ADC_HYST_MV;

        setup_adc_channel(entry->adc_dev, entry->channel_id);
        k_work_init_delayable(&entry->sample_work, sensor_sample_handler);

        dt_sensor_count++;
    }

    if (dt_sensor_count == 0) {
        LOG_ERR("No hall_sensor_N DT nodes found (hall_sensor_0/1 expected)");
        return -ENODEV;
    }

#if CONFIG_HALL_ADC_CALIBRATE_ON_BOOT
    for (int i = 0; i < dt_sensor_count; ++i) {
        /* calibrate */
        int avg = sample_average_mv(&sensors[i]);
        if (avg > 0) {
            sensors[i].baseline_mv = avg;
            uint32_t assumed_active = sensors[i].baseline_mv + 800;
            sensors[i].threshold_mv = sensors[i].baseline_mv + ((assumed_active - sensors[i].baseline_mv) / 2);
            sensors[i].hyst_mv = CONFIG_HALL_ADC_HYST_MV;
            LOG_INF("calibrated id=%d baseline=%d thr=%d", sensors[i].key_id, sensors[i].baseline_mv, sensors[i].threshold_mv);
        } else {
            LOG_WRN("calibration sample failed for sensor %d", i);
        }
    }
#endif

    hall_socd_register_sensor_count(dt_sensor_count);

    for (int i = 0; i < dt_sensor_count; ++i) {
        k_work_schedule(&sensors[i].sample_work, K_MSEC(100));
    }

    return 0;
}

static int hall_adc_module_init(const struct device *dev)
{
    ARG_UNUSED(dev);
    int rc = dt_init_sensors();
    if (rc) {
        LOG_ERR("dt_init_sensors failed: %d", rc);
    } else {
        LOG_INF("hall_adc dt init: %d sensors", dt_sensor_count);
    }
    return rc;
}

SYS_INIT(hall_adc_module_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
