#include <esp_check.h>
#include "class/midi/midi_device.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "esp_log.h"


#define DEBOUNCE_TIME_MS 1
#define HOLD_TIME_MS 1000

#define MIDI_CHANNEL 0

#define BOOT_BUTTON GPIO_NUM_0
#define FOOTSWITCH_1 GPIO_NUM_4
#define FOOTSWITCH_2 GPIO_NUM_5
#define FOOTSWITCH_3 GPIO_NUM_6
#define FOOTSWITCH_4 GPIO_NUM_38
#define FOOTSWITCH_5 GPIO_NUM_47
#define FOOTSWITCH_6 GPIO_NUM_48

static const gpio_num_t footswitches[] = {
    FOOTSWITCH_1,
    FOOTSWITCH_2,
    FOOTSWITCH_3,
    FOOTSWITCH_4,
    FOOTSWITCH_5,
    FOOTSWITCH_6
};

typedef struct {
    gpio_num_t gpio;
    uint8_t cc_number;
    uint8_t cc_value;
} footswitch_config_t;

// Array of footswitch configurations
static const footswitch_config_t footswitches_config[] = {
    {BOOT_BUTTON, 48, 64},
    {FOOTSWITCH_1, 63, 127},
    {FOOTSWITCH_2, 57, 65},
    {FOOTSWITCH_3, 22, 127},
    {FOOTSWITCH_4, 23, 127},
    {FOOTSWITCH_5, 22, 127},
    {FOOTSWITCH_6, 23, 127}
};


typedef struct {
    gpio_num_t gpio; // GPIO pin number
    bool debounced_state; // True if pressed (active low), false otherwise
    uint32_t press_start_tick; // Tick count when button was pressed
    uint32_t last_transition_tick; // Tick count of the last valid state transition
    volatile bool hold_triggered; // Hold event triggered flag
    TimerHandle_t hold_timer; // Timer for hold detection
} gpio_state_t;

static gpio_state_t gpio_states[7] = {
    {.gpio = BOOT_BUTTON},
    {.gpio = FOOTSWITCH_1},
    {.gpio = FOOTSWITCH_2},
    {.gpio = FOOTSWITCH_3},
    {.gpio = FOOTSWITCH_4},
    {.gpio = FOOTSWITCH_5},
    {.gpio = FOOTSWITCH_6},
};

static QueueHandle_t gpio_evt_queue = NULL;

// Function to send MIDI Control Change message
void send_midi_cc(uint8_t cc_number, uint8_t cc_value) {
    uint8_t midi_packet[4];
    midi_packet[0] = 0x0B; // Cable Number (0) | Code Index Number (0xB for Control Change)
    midi_packet[1] = 0xB0 | (MIDI_CHANNEL & 0x0F); // Status Byte: 0xB0 (Control Change) | channel number
    midi_packet[2] = cc_number; // Controller Number
    midi_packet[3] = cc_value; // Controller Value
    tud_midi_packet_write(midi_packet);
}

static void gpio_task_example(void* arg) {
    uint32_t gpio_num;
    while (true) {
        if (!xQueueReceive(gpio_evt_queue, &gpio_num, portMAX_DELAY)) {
            continue;
        }

        ESP_LOGI(
            "gpio_task_exampke",
            "GPIO[%"PRIu32"] intr, val: %d, last_transition_tick: %d",
            gpio_num,
            gpio_states[gpio_num].debounced_state,
            gpio_states[gpio_num].last_transition_tick
        );

        send_midi_cc(footswitches_config[gpio_num].cc_number, footswitches_config[gpio_num].cc_value);
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    const uint32_t gpio_pin = (uint32_t)arg;
    const TickType_t current_tick = xTaskGetTickCountFromISR();
    gpio_state_t* state = &gpio_states[gpio_pin];

    // Read current level (active low: 0 means pressed)
    const int level = gpio_get_level(state->gpio);
    const bool new_state = (level == 0);

    // Process only if the state is changing
    if (new_state != state->debounced_state) {
        // Check if enough time has passed for debouncing
        if ((current_tick - state->last_transition_tick) < pdMS_TO_TICKS(DEBOUNCE_TIME_MS)) {
            return;
        }
        // Update state and record the transition time
        state->debounced_state = new_state;
        state->last_transition_tick = current_tick;

        if (new_state) {
            // Button pressed: record press start tick
            state->press_start_tick = current_tick;
        }

        // Notify the task about the state change
        xQueueSendFromISR(gpio_evt_queue, &gpio_pin, NULL);
    }
}

esp_err_t setup_footswitch_config(int8_t gpio_num) {
    const gpio_config_t footswitch_config = {
        .pin_bit_mask = BIT64(gpio_num),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_ANYEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    return gpio_config(&footswitch_config);
}

esp_err_t initialize_gpio_interrupts() {
    gpio_evt_queue = xQueueCreate(100, sizeof(uint32_t));
    if (gpio_evt_queue == 0) {
        ESP_LOGE("initialize_gpio_interrupts", "Failed to create queue.");
        return ESP_FAIL;
    }

    const BaseType_t xReturned = xTaskCreate(gpio_task_example, "gpio_task_example", 4096, NULL, 1, NULL);
    if (xReturned != pdPASS) {
        // Task creation failed. Log the error and return a custom error code
        ESP_LOGE("xTaskCreate", "Failed to create task. Error code: 0x%X", xReturned);
        return ESP_FAIL; // Return a custom error code indicating memory allocation failure
    }


    ESP_RETURN_ON_ERROR(
        gpio_install_isr_service(ESP_INTR_FLAG_EDGE),
        "initialize_gpio_interrupts",
        "initialize_gpio_interrupts"
    );
    ESP_RETURN_ON_ERROR(
        gpio_isr_handler_add(BOOT_BUTTON, gpio_isr_handler, 0),
        "initialize_gpio_interrupts",
        "initialize_gpio_interrupts"
    );
    for (int i = 0; i < sizeof(footswitches) / sizeof(footswitches[0]); i++) {
        ESP_RETURN_ON_ERROR(
            gpio_isr_handler_add(footswitches[i], gpio_isr_handler, (void *)(i + 1)),
            "initialize_gpio_interrupts",
            "isr_handler_add failed on footswitch-%d",
            i + 1
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

    for (int i = 0; i < sizeof(footswitches) / sizeof(footswitches[0]); i++) {
        ret = setup_footswitch_config(footswitches[i]);
        ESP_RETURN_ON_ERROR(ret, "setup_gpios", "Footswitch %d config failed", i+1);
    }

    return initialize_gpio_interrupts();
}
