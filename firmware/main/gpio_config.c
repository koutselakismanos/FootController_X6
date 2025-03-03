#include <esp_check.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"

#define BOOT_BUTTON GPIO_NUM_0
#define FOOTSWITCH_1 GPIO_NUM_4
#define FOOTSWITCH_2 GPIO_NUM_5
#define FOOTSWITCH_3 GPIO_NUM_6
#define FOOTSWITCH_4 GPIO_NUM_38
#define FOOTSWITCH_5 GPIO_NUM_47
#define FOOTSWITCH_6 GPIO_NUM_48

const gpio_num_t footswitches[] = {
    FOOTSWITCH_1,
    FOOTSWITCH_2,
    FOOTSWITCH_3,
    FOOTSWITCH_4,
    FOOTSWITCH_5,
    FOOTSWITCH_6
};

static QueueHandle_t gpio_evt_queue = NULL;

static void gpio_task_example(void *arg) {
    uint32_t gpio_num;
    while (true) {
        if (!xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            continue;
        }

        ESP_LOGI("gpio_task_exampke", "GPIO[%"PRIu32"] intr, val: %d", gpio_num, gpio_get_level(gpio_num));
    }
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    const uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

esp_err_t setup_footswitch_config(int8_t footswitch_num) {
    const gpio_config_t footswitch_config = {
        .pin_bit_mask = BIT64(footswitch_num),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    return gpio_config(&footswitch_config);
}


esp_err_t initialize_gpio_interrupts() {
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    if (gpio_evt_queue == 0) {
        ESP_LOGE("initialize_gpio_interrupts", "Failed to create queue.");
        return ESP_FAIL;
    }

    const BaseType_t xReturned = xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
    if (xReturned != pdPASS) {
        // Task creation failed. Log the error and return a custom error code
        ESP_LOGE("xTaskCreate", "Failed to create task. Error code: 0x%X", xReturned);
        return ESP_FAIL; // Return a custom error code indicating memory allocation failure
    }


    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_EDGE), "initialize_gpio_interrupts", "initialize_gpio_interrupts");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BOOT_BUTTON, gpio_isr_handler, BOOT_BUTTON), "initialize_gpio_interrupts", "initialize_gpio_interrupts");
    for (int i = 0; i < sizeof(footswitches) / sizeof(footswitches[0]); i++) {
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(footswitches[i], gpio_isr_handler, (void *)(i + 1)),
            "initialize_gpio_interrupts",
            "isr_handler_add failed on footswitch-%d",
            i+1
        );
    }

    return ESP_OK;
}

esp_err_t configure_gpios(void) {
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(BOOT_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t ret = gpio_config(&boot_button_config);
    ESP_RETURN_ON_ERROR(ret, "configure_gpios", "Boot button config failed");

    const gpio_num_t footswitches[] = {
        FOOTSWITCH_1,
        FOOTSWITCH_2,
        FOOTSWITCH_3,
        FOOTSWITCH_4,
        FOOTSWITCH_5,
        FOOTSWITCH_6
    };

    for (int i = 0; i < sizeof(footswitches) / sizeof(footswitches[0]); i++) {
        ret = setup_footswitch_config(footswitches[i]);
        ESP_RETURN_ON_ERROR(ret, "setup_gpios", "Footswitch %d config failed", i+1);
    }

    return initialize_gpio_interrupts();
}
