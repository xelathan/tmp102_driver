#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/syscalls/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/display.h>

#include <lvgl.h>
#include <string.h>
#include <app_config.h>

K_THREAD_STACK_DEFINE(tmp102_stack, TMP102_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(tmp_output_stack, TMP_OUTPUT_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(apds9960_stack, APDS9960_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(light_intensity_stack, LIGHT_INTENSITY_THREAD_STACK_SIZE);

K_MSGQ_DEFINE(apds_msg_queue, sizeof(struct sensor_value), MSGQ_SIZE, 1);
K_MSGQ_DEFINE(tmp_msg_queue, sizeof(struct sensor_value), MSGQ_SIZE, 1);

const struct device* sensor = DEVICE_DT_GET(TMP102_NODE);
const struct device* general_sensor = DEVICE_DT_GET(APDS9960_NODE);

lv_obj_t *tmp_label;
lv_obj_t *light_intensity_label;

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
         
        ret = k_msgq_put(&tmp_msg_queue, &temp_value, K_FOREVER);
        if (ret < 0) {
            printk("Failed to put temp_value into msgq\r\n");
        }

		k_msleep(TMP102_THREAD_SLEEP_MS);
    }
}

void apds9960_sample_thread_start(void* arg1, void* arg2, void* arg3) {
    struct sensor_value intensity;
    int ret;

    for (;;) {
        ret = sensor_sample_fetch(general_sensor);
        if (ret) {
            printk("APDS9960 sensor_sample_fetch failed: %d\n", ret);
            continue;
        }

        ret = sensor_channel_get(general_sensor, SENSOR_CHAN_LIGHT, &intensity);
        if (ret < 0) {
            printk("failed to get light data: %d\n", ret);
        }

        ret = k_msgq_put(&apds_msg_queue, &intensity, K_FOREVER);
        if (ret < 0) {
            printk("Failed to put light_value into msgq\r\n");
        }
        
        k_msleep(APDS9960_THREAD_SLEEP_MS);
    }
}

void tmp_output_thread_start(void* arg1, void* arg2, void* arg3) {
    int ret;
    struct sensor_value tmp_val;

    for (;;) {
        ret = k_msgq_get(&tmp_msg_queue, &tmp_val, K_FOREVER);
        if (ret < 0) {
            printk("Failed to get temp_value from queue\r\n");
            continue;
        }

        char buffer[16];      
        sprintf(buffer, "Tmp: %dC", tmp_val.val1);
        printk("%s\n", buffer);
        lv_label_set_text(tmp_label, buffer);

        k_msleep(500);
    }
}

void light_intensity_output_thread_start(void* arg1, void* arg2, void* arg3) {
    int ret;
    struct sensor_value light_val;

    for (;;) {
        ret = k_msgq_get(&apds_msg_queue, &light_val, K_FOREVER);
        if (ret < 0) {
            printk("Failed to get light int\r\n");
            continue;
        }

        char buffer[16];
        sprintf(buffer, "Light: %d", light_val.val1);
        printk("%s\n", buffer);
        lv_label_set_text(light_intensity_label, buffer);

        k_msleep(500);
    }
}

int main(void) {
    struct sensor_value attr;
    int ret;

    if (!sensor || !general_sensor) {
        printk("A device was not found in the device tree\r\n");
        return 0;
    }

    if (!device_is_ready(sensor) || !device_is_ready(general_sensor)) {
        printk("A device not ready\r\n");
        return 0;
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

    attr.val1 = 1;
    attr.val2 = 0;
    ret = sensor_attr_set(general_sensor, SENSOR_CHAN_LIGHT, SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
    if (ret < 0) {
        printk("Failed to enable ALS: %d\n", ret);
    }

    // // Enable Proximity
    // ret = sensor_attr_set(general_sensor, SENSOR_CHAN_PROX, SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
    // if (ret) {
    //     printk("Failed to enable proximity: %d\n", ret);
    // }

    // Set ALS gain (optional but recommended)
    attr.val1 = 1; // 1x gain
    ret = sensor_attr_set(general_sensor, SENSOR_CHAN_LIGHT, SENSOR_ATTR_GAIN, &attr);

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

    struct k_thread apds9960_sample_thread;
    k_tid_t apds9960_output_tid = k_thread_create(
        &apds9960_sample_thread,
        apds9960_stack,
        K_THREAD_STACK_SIZEOF(apds9960_stack),
        apds9960_sample_thread_start,
        NULL,
        NULL,
        NULL,
        7,
        0,
        K_NO_WAIT
    );

    if (apds9960_output_tid < 0) {
        printk("Failed to create apds thread\r\n");
        return 0;
    }

    struct k_thread tmp_output_thread;

    k_tid_t tmp_output_tid = k_thread_create(
        &tmp_output_thread,
        tmp_output_stack,
        K_THREAD_STACK_SIZEOF(tmp_output_stack),
        tmp_output_thread_start,
        NULL,
        NULL,
        NULL,
        7,
        0,
        K_NO_WAIT
    );

    if (tmp_output_tid < 0) {
        printk("Failed to create serial output thread\r\n");
        return 0;
    }

    struct k_thread light_output_thread;

     k_tid_t light_output_tid = k_thread_create(
        &light_output_thread,
        light_intensity_stack,
        K_THREAD_STACK_SIZEOF(light_intensity_stack),
        light_intensity_output_thread_start,
        NULL,
        NULL,
        NULL,
        7,
        0,
        K_NO_WAIT
    );

    if (light_output_tid < 0) {
        printk("Failed to create serial output thread\r\n");
        return 0;
    }

    const struct device* display;

    display = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!display || !device_is_ready(display)) {
        printk("failed to find device or device not ready");
        return 0;
    }
    struct display_capabilities caps;
    display_get_capabilities(display, &caps);
    printk("Display: %dx%d, supported formats: 0x%x\n", 
        caps.x_resolution, caps.y_resolution, caps.supported_pixel_formats);
        
    // Create a simple label
    tmp_label = lv_label_create(lv_scr_act());
    lv_label_set_text(tmp_label, "Tmp: C");
    lv_obj_align(tmp_label, LV_ALIGN_TOP_LEFT, 0, 5);

    light_intensity_label = lv_label_create(lv_scr_act());
    lv_label_set_text(light_intensity_label, "Light: ");
    lv_obj_align(light_intensity_label, LV_ALIGN_LEFT_MID, 0, 5);

    lv_timer_handler();
    ret = display_blanking_off(display);
    if (ret < 0) {
        printk("failed to blank display\n");
    }

    // Simple loop
    for (;;) {
        uint32_t sleep_ms = lv_timer_handler();
        k_msleep(MIN(sleep_ms, 100));
    }

    return 0;
}