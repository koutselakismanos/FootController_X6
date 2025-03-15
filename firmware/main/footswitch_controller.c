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

static footswitch_config_t footswitch_config[] = {
    {
        GPIO_NUM_0,
        {
            {57, 126},
            {57, 0}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {57, 0}}
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
        if (cfg->hold_action == HOLD_ACTION_NONE) {
            send_layer_midi(idx);
            return;
        }

        xTimerStart(state->hold_timer, 0);
        return;
    }

    // When the key is released:
    // If the hold timer did not trigger a hold action, and a hold action is defined,
    // then consider it a tap and send the press event.
    if (!state->hold_active && cfg->hold_action != HOLD_ACTION_NONE) {
        send_layer_midi(idx);
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

#include <stdio.h>
#include <string.h>
#include "json_parser.h"
#include "nvs_flash.h"

#define TAG "config_parser"

// Example function to update configuration from JSON string
void update_config_from_json(const char* json_str) {
    jparse_ctx_t jctx;
    int ret;

    // Start parsing; json_parse_start allocates tokens, etc.
    ret = json_parse_start(&jctx, json_str, strlen(json_str));
    if (ret != OS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to start JSON parsing");
        return;
    }

    // Get the footswitch "id" from the JSON object
    int id = 0;
    ret = json_obj_get_int(&jctx, "id", &id);
    if (ret != OS_SUCCESS) {
        ESP_LOGE(TAG, "Missing or invalid 'id'");
        json_parse_end(&jctx);
        return;
    }
    uint32_t idx = id - 1; // convert id (1-based) to zero-index
    if (idx >= NUM_FOOTSWITCHES) {
        ESP_LOGE(TAG, "Invalid footswitch id: %d", id);
        json_parse_end(&jctx);
        return;
    }

    // Process the "layers" array
    int num_layers = 0;
    ret = json_obj_get_array(&jctx, "layers", &num_layers);
    if (ret == OS_SUCCESS && num_layers > 0) {
        for (uint32_t i = 0; i < (uint32_t)num_layers && i < MAX_LAYERS; i++) {
            // Get the object at index i in the layers array
            ret = json_arr_get_object(&jctx, i);
            if (ret != OS_SUCCESS) {
                ESP_LOGE(TAG, "Failed to get layer object at index %d", i);
                continue;
            }
            int cc_number = 0, cc_value = 0;
            ret = json_obj_get_int(&jctx, "cc_number", &cc_number);
            if (ret == OS_SUCCESS) {
                footswitch_config[idx].layers[i].cc_number = cc_number;
            }
            ret = json_obj_get_int(&jctx, "cc_value", &cc_value);
            if (ret == OS_SUCCESS) {
                footswitch_config[idx].layers[i].cc_value = cc_value;
            }
            // Leave the current layer object context
            json_obj_leave_object(&jctx);
        }
        // Leave the layers array context
        json_obj_leave_array(&jctx);
    }
    else {
        ESP_LOGW(TAG, "No layers array found or empty");
    }

    // Get the "hold_action" string
    char hold_action_str[32] = {0};
    ret = json_obj_get_string(&jctx, "hold_action", hold_action_str, sizeof(hold_action_str));
    if (ret == OS_SUCCESS) {
        if (strcmp(hold_action_str, "midi") == 0) {
            footswitch_config[idx].hold_action = HOLD_ACTION_SEND_MIDI;
        }
        else if (strcmp(hold_action_str, "momentary") == 0) {
            footswitch_config[idx].hold_action = HOLD_ACTION_MOMENTARY_LAYER;
        }
        else if (strcmp(hold_action_str, "toggle") == 0) {
            footswitch_config[idx].hold_action = HOLD_ACTION_TOGGLE_LAYER;
        }
        else {
            footswitch_config[idx].hold_action = HOLD_ACTION_NONE;
        }

        // Parse target_layer for layer actions
        if (footswitch_config[idx].hold_action == HOLD_ACTION_MOMENTARY_LAYER ||
            footswitch_config[idx].hold_action == HOLD_ACTION_TOGGLE_LAYER) {
            int target_layer = 0;
            ret = json_obj_get_int(&jctx, "target_layer", &target_layer);
            if (ret == OS_SUCCESS) {
                footswitch_config[idx].hold_config.target_layer = (uint8_t)target_layer;
            }
            else {
                ESP_LOGW(TAG, "target_layer not found for layer action");
            }
        }
    }
    else {
        ESP_LOGW(TAG, "hold_action not found; defaulting to NONE");
        footswitch_config[idx].hold_action = HOLD_ACTION_NONE;
    }

    // End parsing and free resources
    json_parse_end(&jctx);
    ESP_LOGI(TAG, "Configuration updated for footswitch %d", id);
}


#define NVS_NAMESPACE "config"

esp_err_t save_config_to_nvs(const char* json_str) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return err;
    }

    // Save the JSON string under the key "footswitch_config".
    err = nvs_set_str(nvs_handle, "footswitch_config", json_str);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing config to NVS");
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return err;
}

void load_config_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle");
        return;
    }

    size_t required_size = 0;
    err = nvs_get_str(nvs_handle, "footswitch_config", NULL, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "No saved config found");
        nvs_close(nvs_handle);
        return;
    }
    else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading config size");
        nvs_close(nvs_handle);
        return;
    }

    char* json_str = malloc(required_size);
    if (!json_str) {
        ESP_LOGE(TAG, "Failed to allocate memory for config");
        nvs_close(nvs_handle);
        return;
    }

    err = nvs_get_str(nvs_handle, "footswitch_config", json_str, &required_size);
    if (err == ESP_OK) {
        update_config_from_json(json_str);
    }
    else {
        ESP_LOGE(TAG, "Error reading config string");
    }

    free(json_str);
    nvs_close(nvs_handle);
}
