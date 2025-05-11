cmake_minimum_required(VERSION 3.22.0)

set(ZEPHYR_EXTRA_MODULES "${CMAKE_SOURCE_DIR}/modules/tmp102")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(
    tmp102_driver
    DESCRIPTION "Zephyr driver for tmp102"
    LANGUAGES C
    VERSION 1.0
)

target_sources(app PRIVATE src/main.c)