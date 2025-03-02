#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"


#define APP_BUTTON (GPIO_NUM_0) // Use BOOT signal by default

static QueueHandle_t gpio_evt_queue = NULL;

static void gpio_task_example(void *arg) {
    uint32_t io_num;
    while (true) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    ESP_LOGI("GPIO", "IRAM ISR");
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


void setup_gpios() {
    const gpio_config_t boot_button_config = {
        .pin_bit_mask = BIT64(APP_BUTTON),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = true,
        .pull_down_en = false,
    };
    ESP_ERROR_CHECK(gpio_config(&boot_button_config));

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(APP_BUTTON, gpio_isr_handler,  (void*) APP_BUTTON);


    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);
}
