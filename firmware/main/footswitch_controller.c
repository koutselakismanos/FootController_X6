#include <esp_check.h>
#include "class/midi/midi_device.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Configuration ==============================================================
#define DEBOUNCE_TIME_MS     1
#define HOLD_TIME_MS         1000
#define MIDI_CHANNEL         0
#define MAX_LAYERS           2
#define NUM_FOOTSWITCHES     (sizeof(footswitch_config)/sizeof(footswitch_config[0]))
#define DEBOUNCE_TICKS       pdMS_TO_TICKS(DEBOUNCE_TIME_MS)
#define HOLD_EVENT_FLAG      0x80000000
#define TAG                  "footswitch_controller"

typedef enum {
    HOLD_ACTION_NONE,
    HOLD_ACTION_MOMENTARY_LAYER,
    HOLD_ACTION_TOGGLE_LAYER,
    HOLD_ACTION_SEND_MIDI
} hold_action_t;

typedef struct {
    gpio_num_t gpio;

    struct {
        uint8_t cc_number;
        uint8_t cc_value;
    } layers[MAX_LAYERS];

    hold_action_t hold_action;

    union {
        uint8_t target_layer;

        struct {
            uint8_t cc_number, cc_value;
        } midi;
    } hold_config;
} footswitch_config_t;

typedef struct {
    gpio_num_t gpio;
    bool pressed;
    uint32_t last_tick;
    TimerHandle_t hold_timer;
    bool hold_active;
} footswitch_state_t;

typedef struct {
    uint8_t current_layer;
    bool layer_locked;
} layer_state_t;

static const footswitch_config_t footswitch_config[] = {
    {
        GPIO_NUM_0,
        {
            {48, 64},
            {48, 127}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {48, 127}}
    },
    {
        GPIO_NUM_4, {
            {63, 127},
            {63, 0}
        },
        HOLD_ACTION_MOMENTARY_LAYER,
        {.target_layer = 1}
    },
    {
        GPIO_NUM_5, {
            {57, 65},
            {57, 127}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {99, 127}}
    },
};

static footswitch_state_t footswitch_states[NUM_FOOTSWITCHES];
static layer_state_t active_layer = {0, false};
static QueueHandle_t event_queue;

// MIDI Functions =============================================================
static void send_midi(uint8_t cc, uint8_t value) {
    uint8_t packet[] = {0x0B, 0xB0 | MIDI_CHANNEL, cc, value};
    tud_midi_packet_write(packet);
}

static void send_layer_midi(uint8_t idx) {
    const footswitch_config_t* cfg = &footswitch_config[idx];
    uint8_t value = cfg->layers[active_layer.current_layer].cc_value;
    send_midi(cfg->layers[active_layer.current_layer].cc_number, value);
}

// Layer Management ===========================================================
static void switch_layer(uint8_t layer, bool lock) {
    active_layer.current_layer = layer % MAX_LAYERS;
    active_layer.layer_locked = lock;
    ESP_LOGI(TAG, "Switched to %s layer %d", lock ? "locked" : "momentary", active_layer.current_layer);
}

// Event Processing ===========================================================
static void handle_hold(uint32_t idx) {
    const footswitch_config_t* cfg = &footswitch_config[idx];
    footswitch_state_t* state = &footswitch_states[idx];

    if (!state->pressed) return;

    switch (cfg->hold_action) {
    case HOLD_ACTION_MOMENTARY_LAYER:
        switch_layer(cfg->hold_config.target_layer, false);
        break;

    case HOLD_ACTION_TOGGLE_LAYER:
        switch_layer((active_layer.current_layer == cfg->hold_config.target_layer) ? 0 : cfg->hold_config.target_layer,
                     true);
        break;

    case HOLD_ACTION_SEND_MIDI:
        send_midi(cfg->hold_config.midi.cc_number, cfg->hold_config.midi.cc_value);
        break;

    default: break;
    }
    state->hold_active = true;
}

static void handle_press(uint32_t idx, bool pressed) {
    footswitch_state_t* state = &footswitch_states[idx];
    const footswitch_config_t* cfg = &footswitch_config[idx];

    if (pressed) {
        xTimerStart(state->hold_timer, 0);
        send_layer_midi(idx);
        return;
    }

    xTimerStop(state->hold_timer, 0);
    state->hold_active = false;

    if (cfg->hold_action == HOLD_ACTION_MOMENTARY_LAYER && !active_layer.layer_locked) {
        switch_layer(0, false);
    }
}

static void process_event(uint32_t event) {
    const bool is_hold = event & HOLD_EVENT_FLAG;
    const uint32_t idx = event & ~HOLD_EVENT_FLAG;

    if (idx >= NUM_FOOTSWITCHES) return;

    is_hold ? handle_hold(idx) : handle_press(idx, footswitch_states[idx].pressed);
}

static void handle_gpio_events(void* arg) {
    uint32_t event;
    while (true) {
        if (xQueueReceive(event_queue, &event, portMAX_DELAY)) {
            process_event(event);
        }
    }
}

// Hardware Abstraction =======================================================
static void IRAM_ATTR handle_interrupt(void* arg) {
    uint32_t idx = (uint32_t)arg;
    footswitch_state_t* state = &footswitch_states[idx];
    const TickType_t now = xTaskGetTickCountFromISR();
    const bool new_state = !gpio_get_level(state->gpio);

    if (new_state == state->pressed) return;
    if ((now - state->last_tick) < DEBOUNCE_TICKS) return;

    state->pressed = new_state;
    state->last_tick = now;
    xQueueSendFromISR(event_queue, &idx, NULL);
}

static void hold_timer_cb(TimerHandle_t timer) {
    uint32_t idx = (uint32_t)pvTimerGetTimerID(timer);
    uint32_t event = HOLD_EVENT_FLAG | idx;
    xQueueSend(event_queue, &event, 0);
}

static esp_err_t init_switch(uint32_t idx) {
    footswitch_state_t* state = &footswitch_states[idx];
    const gpio_num_t gpio = footswitch_config[idx].gpio;

    gpio_config_t cfg = {
        .pin_bit_mask = BIT64(gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "GPIO config failed");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(gpio, handle_interrupt, (void*)idx), TAG, "ISR add failed");

    state->hold_timer = xTimerCreate("hold", pdMS_TO_TICKS(HOLD_TIME_MS), pdFALSE, (void*)idx, hold_timer_cb);
    ESP_RETURN_ON_FALSE(state->hold_timer, ESP_FAIL, TAG, "Timer create failed");

    return ESP_OK;
}

esp_err_t initialize_footswitches(void) {
    event_queue = xQueueCreate(20, sizeof(uint32_t));
    ESP_RETURN_ON_FALSE(event_queue, ESP_FAIL, TAG, "Queue create failed");

    ESP_RETURN_ON_ERROR(gpio_install_isr_service(0), TAG, "ISR service failed");

    for (uint32_t i = 0; i < NUM_FOOTSWITCHES; i++) {
        footswitch_states[i] = (footswitch_state_t){
            .gpio = footswitch_config[i].gpio,
            .pressed = false,
            .last_tick = 0,
            .hold_timer = NULL,
            .hold_active = false
        };

        ESP_RETURN_ON_ERROR(init_switch(i), TAG, "Switch %d init failed", i);
    }

    BaseType_t task_err = xTaskCreate(handle_gpio_events, "footswitches", 4096, NULL, 1, NULL);
    ESP_RETURN_ON_FALSE(task_err == pdPASS, ESP_FAIL, TAG, "Task create failed");

    return ESP_OK;
}
