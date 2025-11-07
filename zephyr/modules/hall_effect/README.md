Hall Effect module (A1324 ADC) for tanx66oled

Drop this module into the repo at `zephyr/modules/hall_effect/` and either add the overlay `boards/shields/tanx66oled/hall-sensors.overlay`
or merge the hall_effect node into your existing shield overlay.

Requirements:
- Parent DT node labeled `hall_effect`
- Children labeled `hall_sensor_0`, `hall_sensor_1`, ... with properties:
    io-channels = <&adc N>;
    key-id = <M>;

Notes:
- This driver currently expects sensors 0 and 1 mapped to ADC channel 0 and 5 by default.
- The driver calibrates on boot and computes thresholds. Tune Kconfig values in Kconfig or prj.conf.
- Replace or implement hall_emit_key_position() to integrate with ZMK event emitter if you want real key-position integration (the code currently calls that wrapper).
