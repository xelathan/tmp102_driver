#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/syscalls/device.h>
#include <zephyr/drivers/sensor.h>

#define TMP102_NODE DT_ALIAS(tmp_sensor)
#define SLEEP_MS 1000


/**
 * Threads
 */
#define TMP102_THREAD_STACK_SIZE 1024
#define SERIAL_THREAD_STACK_SIZE 1024
#define TMP102_THREAD_SLEEP_MS 100
#define SERIAL_THREAD_SLEEP_MS 500

K_THREAD_STACK_DEFINE(tmp102_stack, TMP102_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(serial_stack, SERIAL_THREAD_STACK_SIZE);

#define MSGQ_SIZE 10

K_MSGQ_DEFINE(msg_queue, sizeof(struct sensor_value), MSGQ_SIZE, 1);

const struct device* sensor = DEVICE_DT_GET(TMP102_NODE);

void tmp102_sample_thread_start(void* arg_1, void* arg_2, void* arg_3) {
    int ret;

    for (;;) {
        ret = sensor_sample_fetch(sensor);
        if (ret) {
            printk("Failed to fetch sample\r\n");
            continue;
        }

        struct sensor_value temp_value;

        ret = sensor_channel_get(sensor, SENSOR_CHAN_AMBIENT_TEMP, &temp_value);
		if (ret) {
			printk("sensor_channel_get failed ret %d\n", ret);
            continue;
		}

        printk("Fetching...\r\n");
         
        ret = k_msgq_put(&msg_queue, &temp_value, K_FOREVER);
        if (ret < 0) {
            printk("Failed to put temp_value into msgq\r\n");
        }

		k_msleep(TMP102_THREAD_SLEEP_MS);
    }
}

void serial_output_thread_start(void* arg1, void* arg2, void* arg3) {
    int ret;
    struct sensor_value temp_value;

    for (;;) {
        if (k_msgq_num_used_get(&msg_queue) == MSGQ_SIZE) {
            for (int i = 0; i < MSGQ_SIZE; i++) {
                ret = k_msgq_get(&msg_queue, &temp_value, K_FOREVER);
                if (ret < 0) {
                    printk("Failed to get temp_value from msgq\r\n");
                    continue;
                }
                printk("temp is %d (%d micro)\n", temp_value.val1,
                    temp_value.val2);
            }
        }
		k_msleep(SERIAL_THREAD_SLEEP_MS);
    }
}

int main(void) {
    struct sensor_value attr;
    int ret;

    if (!sensor) {
        printk("TMP102 not found in the device tree\r\n");
        return 0;
    }

    if (!device_is_ready(sensor)) {
        printk("TMP102 device not ready\r\n");
    }

    attr.val1 = 128;
    attr.val2 = 0;

    // set celcius temp range to measure
    ret = sensor_attr_set(
        sensor,
        SENSOR_CHAN_AMBIENT_TEMP,
        SENSOR_ATTR_FULL_SCALE,
        &attr
    );

    if (ret < 0) {
        printk("Failed to get sensor scale\r\n");
    }

    attr.val1 = 8;
	attr.val2 = 0;

    ret = sensor_attr_set(
        sensor,
        SENSOR_CHAN_AMBIENT_TEMP,
        SENSOR_ATTR_SAMPLING_FREQUENCY,
        &attr
    );

    if (ret < 0) {
        printk("Failed to set sensor sampling frequency\r\n");
        return 0;
    }

    struct k_thread tmp102_sample_thread;

    k_tid_t tmp102_sample_tid = k_thread_create(
        &tmp102_sample_thread,
        tmp102_stack,
        K_THREAD_STACK_SIZEOF(tmp102_stack),
        tmp102_sample_thread_start,
        NULL,
        NULL,
        NULL,
        7,
        0,
        K_NO_WAIT
    );

    if (tmp102_sample_tid < 0) {
        printk("Failed to create tmp102 thread\r\n");
        return 0;
    }

    struct k_thread serial_output_thread;

    k_tid_t serial_ouput_tid = k_thread_create(
        &serial_output_thread,
        serial_stack,
        K_THREAD_STACK_SIZEOF(serial_stack),
        serial_output_thread_start,
        NULL,
        NULL,
        NULL,
        7,
        0,
        K_NO_WAIT
    );

    if (serial_ouput_tid < 0) {
        printk("Failed to create serial output thread\r\n");
        return 0;
    }

    for (;;) {
        k_sleep(K_FOREVER);
    }

    return 0;
}