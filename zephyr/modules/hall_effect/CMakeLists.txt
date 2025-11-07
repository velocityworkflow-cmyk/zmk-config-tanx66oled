cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hall_effect)

zephyr_library()
target_include_directories(app PRIVATE include)

zephyr_library_sources(
  src/hall_adc_driver.c
  src/hall_actuation.c
  src/hall_socd.c
  src/hall_rapid_trigger.c
)
