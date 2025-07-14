#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <zephyr/kernel.h>

// node references
#define TMP102_NODE DT_ALIAS(tmp_sensor)
#define APDS9960_NODE DT_ALIAS(apds_sensor)

// Threads
#define TMP102_THREAD_STACK_SIZE 1024
#define APDS9960_THREAD_STACK_SIZE 1024
#define TMP_OUTPUT_THREAD_STACK_SIZE 1024
#define LIGHT_INTENSITY_THREAD_STACK_SIZE 1024

// sleep
#define TMP102_THREAD_SLEEP_MS 500
#define APDS9960_THREAD_SLEEP_MS 500
#define MAIN_SLEEP_MS 50

// msg queue
#define MSGQ_SIZE 1

#endif

