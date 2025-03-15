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
#define HOLD_TIME_MS         500
#define MIDI_CHANNEL         0
#define MAX_LAYERS           2
#define NUM_FOOTSWITCHES     7
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
    {
        GPIO_NUM_5, {
            {57, 65},
            {57, 127}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {99, 127}}
    },
    {
        GPIO_NUM_6, {
            {57, 65},
            {57, 127}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {99, 127}}
    },
    {
        GPIO_NUM_38, {
            {57, 65},
            {57, 127}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {99, 127}}
    },
    {
        GPIO_NUM_47, {
            {57, 65},
            {57, 127}
        },
        HOLD_ACTION_SEND_MIDI,
        {.midi = {99, 127}}
    },
    {
        GPIO_NUM_48, {
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

bool update_config_from_json(const char *json_str) {
    jparse_ctx_t jctx;
    int ret = json_parse_start(&jctx, json_str, strlen(json_str));
    if (ret != OS_SUCCESS) {
        ESP_LOGE(TAG, "Failed to start JSON parsing: error %d", ret);
        return false;
    }

    int config_array_size;
    if (json_obj_get_array(&jctx, "config", &config_array_size) != OS_SUCCESS) {
        ESP_LOGE(TAG, "Missing or invalid 'config' array in root object");
        json_parse_end(&jctx);
        return false;
    }

    ESP_LOGI(TAG, "Found %d config entries", config_array_size);

    for (int i = 0; i < config_array_size; i++) {
        if (json_arr_get_object(&jctx, i) != OS_SUCCESS) {
            ESP_LOGE(TAG, "Config entry %d is not a valid object", i);
            continue;
        }

        int id;
        if (json_obj_get_int(&jctx, "id", &id) != OS_SUCCESS) {
            ESP_LOGE(TAG, "Missing/invalid 'id' in config entry %d", i);
            json_arr_leave_object(&jctx);
            continue;
        }

        int index = id - 1;
        if (index < 0 || index >= 7) {
            ESP_LOGE(TAG, "Invalid ID %d in config (must be 1-8)", id);
            json_arr_leave_object(&jctx);
            continue;
        }

        // Parse layers
        int num_layers;
        if (json_obj_get_array(&jctx, "layers", &num_layers) != OS_SUCCESS) {
            ESP_LOGE(TAG, "Config %d (ID %d): Missing/invalid 'layers' array", i, id);
        } else {
            if (num_layers > MAX_LAYERS) {
                ESP_LOGW(TAG, "Config %d (ID %d): Too many layers (%d, max %d)",
                        i, id, num_layers, MAX_LAYERS);
            }

            for (int l = 0; l < num_layers && l < MAX_LAYERS; l++) {
                if (json_arr_get_object(&jctx, l) != OS_SUCCESS) {
                    ESP_LOGE(TAG, "Config %d (ID %d): Layer %d is not an object", i, id, l);
                    continue;
                }

                int cc_num;
                if (json_obj_get_int(&jctx, "cc_number", &cc_num) != OS_SUCCESS) {
                    ESP_LOGE(TAG, "Config %d (ID %d) layer %d: Missing/invalid cc_number", i, id, l);
                } else if (cc_num < 0 || cc_num > 127) {
                    ESP_LOGE(TAG, "Config %d (ID %d) layer %d: Invalid CC number %d (0-127)",
                            i, id, l, cc_num);
                } else {
                    footswitch_config[index].layers[l].cc_number = (uint8_t)cc_num;
                }

                int cc_val;
                if (json_obj_get_int(&jctx, "cc_value", &cc_val) != OS_SUCCESS) {
                    ESP_LOGE(TAG, "Config %d (ID %d) layer %d: Missing/invalid cc_value", i, id, l);
                } else if (cc_val < 0 || cc_val > 127) {
                    ESP_LOGE(TAG, "Config %d (ID %d) layer %d: Invalid CC value %d (0-127)",
                            i, id, l, cc_val);
                } else {
                    footswitch_config[index].layers[l].cc_value = (uint8_t)cc_val;
                }

                json_arr_leave_object(&jctx); // Leave layer object
            }
            json_obj_leave_array(&jctx); // Leave layers array
        }

        // Parse hold action
        char hold_action[20] = {0};
        footswitch_config[index].hold_action = HOLD_ACTION_NONE;

        if (json_obj_get_string(&jctx, "hold_action", hold_action, sizeof(hold_action)) != OS_SUCCESS) {
            ESP_LOGI(TAG, "Config %d (ID %d): No hold action specified", i, id);
        } else {
            if (strcmp(hold_action, "midi") == 0) {
                footswitch_config[index].hold_action = HOLD_ACTION_SEND_MIDI;

                if (json_obj_get_object(&jctx, "midi_cc") != OS_SUCCESS) {
                    ESP_LOGE(TAG, "Config %d (ID %d): Missing midi_cc object for MIDI hold action", i, id);
                } else {
                    int cc_num;
                    if (json_obj_get_int(&jctx, "number", &cc_num) != OS_SUCCESS) {
                        ESP_LOGE(TAG, "Config %d (ID %d): Missing/invalid midi_cc number", i, id);
                    } else if (cc_num < 0 || cc_num > 127) {
                        ESP_LOGE(TAG, "Config %d (ID %d): Invalid MIDI CC number %d", i, id, cc_num);
                    } else {
                        footswitch_config[index].hold_config.midi.cc_number = (uint8_t)cc_num;
                    }

                    int cc_val;
                    if (json_obj_get_int(&jctx, "value", &cc_val) != OS_SUCCESS) {
                        ESP_LOGE(TAG, "Config %d (ID %d): Missing/invalid midi_cc value", i, id);
                    } else if (cc_val < 0 || cc_val > 127) {
                        ESP_LOGE(TAG, "Config %d (ID %d): Invalid MIDI CC value %d", i, id, cc_val);
                    } else {
                        footswitch_config[index].hold_config.midi.cc_value = (uint8_t)cc_val;
                    }

                    json_obj_leave_object(&jctx); // Leave midi_cc object
                }
            }
            else if (strcmp(hold_action, "momentary_layer") == 0) {
                footswitch_config[index].hold_action = HOLD_ACTION_MOMENTARY_LAYER;
                int target_layer;
                if (json_obj_get_int(&jctx, "target_layer", &target_layer) != OS_SUCCESS) {
                    ESP_LOGE(TAG, "Config %d (ID %d): Missing target_layer for momentary action", i, id);
                } else if (target_layer < 0 || target_layer >= MAX_LAYERS) {
                    ESP_LOGE(TAG, "Config %d (ID %d): Invalid target_layer %d", i, id, target_layer);
                } else {
                    footswitch_config[index].hold_config.target_layer = (uint8_t)target_layer;
                }
            }
            else if (strcmp(hold_action, "toggle_layer") == 0) {
                footswitch_config[index].hold_action = HOLD_ACTION_TOGGLE_LAYER;
                int target_layer;
                if (json_obj_get_int(&jctx, "target_layer", &target_layer) != OS_SUCCESS) {
                    ESP_LOGE(TAG, "Config %d (ID %d): Missing target_layer for toggle action", i, id);
                } else if (target_layer < 0 || target_layer >= MAX_LAYERS) {
                    ESP_LOGE(TAG, "Config %d (ID %d): Invalid target_layer %d", i, id, target_layer);
                } else {
                    footswitch_config[index].hold_config.target_layer = (uint8_t)target_layer;
                }
            }
            else {
                ESP_LOGE(TAG, "Config %d (ID %d): Unknown hold_action '%s'", i, id, hold_action);
            }
        }

        json_arr_leave_object(&jctx); // Leave config item object
    }

    json_parse_end(&jctx);
    return true;
}



#include "nvs_flash.h"
#include "nvs.h"
#include <esp_log.h>

static const char *NVS_NAMESPACE = "midi_config";
static const char *NVS_KEY = "footswitches";

bool save_config_to_nvs(void) {
    esp_err_t err;
    nvs_handle_t handle;

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS namespace
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return false;
    }

    // Write entire config array as blob
    size_t config_size = sizeof(footswitch_config_t) * NUM_FOOTSWITCHES;
    err = nvs_set_blob(handle, NVS_KEY, footswitch_config, config_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error writing config blob: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    // Commit changes
    err = nvs_commit(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Configuration saved to NVS");
    return true;
}

bool load_config_from_nvs(void) {
    esp_err_t err;
    nvs_handle_t handle;
    size_t required_size = 0;

    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Open NVS namespace
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "No saved configuration found");
        } else {
            ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        }
        return false;
    }

    // Get blob size first
    err = nvs_get_blob(handle, NVS_KEY, NULL, &required_size);
    if (err != ESP_OK || required_size != sizeof(footswitch_config_t) * NUM_FOOTSWITCHES) {
        ESP_LOGE(TAG, "Invalid config size: %d (expected %d)",
                required_size, sizeof(footswitch_config_t) * NUM_FOOTSWITCHES);
        nvs_close(handle);
        return false;
    }

    // Read blob data
    err = nvs_get_blob(handle, NVS_KEY, footswitch_config, &required_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error reading config blob: %s", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Configuration loaded from NVS");
    return true;
}
