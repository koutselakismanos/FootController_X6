/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "class/midi/midi_device.h"
#include <device/usbd.h>
#include <tusb_config.h>

#include "driver/i2c_master.h"
#include <esp_err.h>

#include "tusb320.h"
#include "footswitch_controller.c"

#define TAG "main"

/************* I2C ****************/
#define I2C_MASTER_SCL_IO GPIO_NUM_17
#define I2C_MASTER_SDA_IO GPIO_NUM_18
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_TIMEOUT_MS 1000

#define TUSB320_ADDR 0x60      /*!< Address of the TUSB320 sensor */
#define TUSB320_DEVICE_ID 0x00 /*!< Register addresses of the "who am I" register */

/************* TinyUSB descriptors ****************/

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_MIDI = 2,
};

#define ITF_NUM_TOTAL 4
#define EPNUM_MIDI_IN 0x83
#define EPNUM_MIDI_OUT 0x03

#define EPNUM_CDC_IN 0x82
#define EPNUM_CDC_OUT 0x02
#define EPNUM_NOTIF_IN 0x81

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN + TUD_CDC_DESC_LEN)

/**
 * @brief String descriptor
 */
const char* string_descriptor[] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},
    // 0: is supported language is English (0x0409)
    CONFIG_TINYUSB_DESC_MANUFACTURER_STRING,
    // 1: Manufacturer
    CONFIG_TINYUSB_DESC_PRODUCT_STRING,
    // 2: Product
    CONFIG_TINYUSB_DESC_SERIAL_STRING,
    // 3: Serials, should use chip ID
    "CDC",
    // 4: CDC
    "Midi controller",
    // 5: MIDI
};

static const uint8_t configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_NOTIF_IN, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 5, EPNUM_MIDI_OUT, EPNUM_MIDI_IN, CFG_TUD_MIDI_EPSIZE),
};


// static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

#define JSON_BUFFER_SIZE 2048  // Adjust based on max expected JSON size

typedef struct {
    char buffer[JSON_BUFFER_SIZE];
    size_t head;
    size_t tail;
} json_ring_buffer_t;

static json_ring_buffer_t json_buf = {0};

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    ESP_LOGI(TAG, "CALLBACK");
    uint8_t chunk[CONFIG_TINYUSB_CDC_RX_BUFSIZE];

    size_t chunk_size = 0;
     // Read chunk from CDC
    if (tinyusb_cdcacm_read(itf, chunk, sizeof(chunk), &chunk_size) == ESP_OK) {
        // Append to ring buffer
        for (size_t i = 0; i < chunk_size; i++) {
            json_buf.buffer[json_buf.head] = chunk[i];
            json_buf.head = (json_buf.head + 1) % JSON_BUFFER_SIZE;

            // Prevent overflow
            if (json_buf.head == json_buf.tail) {
                json_buf.tail = (json_buf.tail + 1) % JSON_BUFFER_SIZE;
            }
        }
    }

    ESP_LOG_BUFFER_HEXDUMP(TAG, chunk, chunk_size, ESP_LOG_INFO);
    ESP_LOGI(TAG, "%c", json_buf.buffer[json_buf.head - 1]);
    if (json_buf.buffer[json_buf.head - 1] == '$') {
        ESP_LOGI(TAG, "Received a JSON string");
        json_buf.buffer[json_buf.head - 1] = 0;
        json_buf.head = json_buf.head - 1;
        update_config_from_json((const char *)json_buf.buffer);
        save_config_to_nvs();
    }

    // esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &chunk_size);
    // if (ret == ESP_OK)
    // {
    //     ESP_LOG_BUFFER_HEXDUMP(TAG, buf, chunk_size, ESP_LOG_INFO);
    //     ESP_LOGI(TAG, "chunk_size %d", chunk_size);
    //
    //     if (chunk_size > 0)
    //     {
    //         // Ensure the received data is null-terminated so it can be treated as a string.
    //         if (chunk_size < CONFIG_TINYUSB_CDC_RX_BUFSIZE)
    //         {
    //             buf[chunk_size] = '\0';
    //         }
    //         else
    //         {
    //             buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE] = '\0';
    //         }
    //
    //         ESP_LOGI(TAG, "Received JSON config: %s", buf);
    //
    //         // Update the configuration using the received JSON string.
    //         update_config_from_json((const char *)buf);
    //
    //         // Optionally, save the configuration to NVS so it persists after reset.
    //         if (save_config_to_nvs((const char *)buf) == ESP_OK) {
    //             ESP_LOGI(TAG, "Configuration saved to NVS");
    //         } else {
    //             ESP_LOGE(TAG, "Failed to save configuration to NVS");
    //         }
    //     }
    // }
}


esp_err_t setup_usb() {
    ESP_LOGI(TAG, "USB Initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = string_descriptor,
        .string_descriptor_count = sizeof(string_descriptor) / sizeof(string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = configuration_descriptor,
    };

    ESP_RETURN_ON_ERROR(tinyusb_driver_install(&tusb_cfg), "setup_usb", "tusb_driver_install");

    // Set up CDC for serial output
    // esp_tusb_init_console(ITF_NUM_CDC); // Redirects printf() to CDC

    // // Initialize MIDI (example using hypothetical MIDI functions)
    // tusb_midi_init();

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &tinyusb_cdc_rx_callback,
    };

    // const tinyusb_config_cdcacm_t acm_cfg = {0};
    ESP_RETURN_ON_ERROR(tusb_cdc_acm_init(&acm_cfg), "setup usb", "tusb_cdc_acm_init");
    ESP_RETURN_ON_ERROR(esp_tusb_init_console(TINYUSB_CDC_ACM_0), "setup usb", "esp_tusb_init_console"); // log to usb;

    ESP_LOGI(TAG, "USB Initialized");
    return ESP_OK;
}

void i2c_master_init(i2c_master_bus_handle_t* bus_handle, i2c_master_dev_handle_t* dev_handle) {
    const i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TUSB320_ADDR,
        .scl_speed_hz = 400000,
    };

    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_cfg, dev_handle));
}

static void midi_test_sender_task(void* arg) {
    while (true) {
        const uint8_t channel = 1;
        uint8_t packet[4];
        packet[0] = 0xB0;
        packet[1] = packet[0] | channel;
        ESP_LOGI(TAG, "Sending MIDI notification");
        tud_midi_packet_write((uint8_t[]){0x0B, 0xB0, 0x39, 0x35});
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void app_main() {
    ESP_ERROR_CHECK(setup_usb());

    // i2c_master_bus_handle_t bus_handle;
    // i2c_master_dev_handle_t dev_handle;
    // i2c_master_init(&bus_handle, &dev_handle);
    // ESP_LOGI(TAG, "I2C initialized successfully");
    load_config_from_nvs();
    ESP_ERROR_CHECK(initialize_footswitches());

    // static int count = 0;
    // while (true) {
    //     // if (tud_mounted())
    //     // {
    //     // if (tud_midi_available())
    //     // {
    //     //     uint8_t packet[4];
    //     //     tud_midi_packet_read(packet);
    //     //     ESP_LOGI(TAG, "%d %d %d %d", packet[0], packet[1], packet[2], packet[3]);
    //     // }
    //     // static bool send_hid_data = false;
    //     // if (send_hid_data) {
    //     // const uint8_t data_length = 1;
    //     // uint8_t data[data_length];
    //     // uint8_t reg_addr = REG_09;
    //     //
    //     // ESP_ERROR_CHECK_WITHOUT_ABORT(
    //     //     i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, data_length,
    //     //         I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS)
    //     // );
    //     //
    //     // reg09_state_t reg09_state;
    //     // reg09_state.attached_state = data[0] & ATTACHED_STATE_MASK;
    //     // reg09_state.cable_dir = data[0] & CABLE_DIR_MASK;
    //     // reg09_state.interrupt_status = data[0] & INTERRUPT_STATUS_MASK;
    //     // reg09_state.drp_duty_cycle = data[0] & DRP_DUTY_CYCLE_MASK;
    //     //
    //     // printf("reg09_state.attached_state = %d\n", reg09_state.attached_state);
    //     // printf("reg09_state.cable_dir= %d\n", reg09_state.cable_dir);
    //
    //     // ----------------------------------------
    //     const uint8_t channel = 1;
    //     uint8_t packet[4];
    //     packet[0] = 0xB0;
    //     packet[1] = packet[0] | channel;
    //     tud_midi_packet_write((uint8_t[]){0x0B, 0xB0, 0x39, 0x35});
    //     //
    //     // if (count % 2 == 0) {
    //     tud_midi_packet_write((uint8_t[]){0x0B, 0xB0, 0x39, 0x35});
    //     //     ESP_LOGI(TAG, "tunner off");
    //     // } else {
    //     //     tud_midi_packet_write((uint8_t[]){0x0B, 0xB0, 0x39, 0x72});
    //     //     ESP_LOGI(TAG, "tuner on");
    //     // }
    //     // count++;
    //     // }
    //     // send_hid_data = !gpio_get_level(APP_BUTTON);
    //     vTaskDelay(pdMS_TO_TICKS(100));
    // }


    // const BaseType_t xReturned = xTaskCreate(midi_test_sender_task, "midi_test_sender_task", 2048, NULL, 5, NULL);
    // if (xReturned != pdPASS) {
    //     // Task creation failed. Log the error and return a custom error code
    //     ESP_LOGE("xTaskCreate", "Failed to create task. Error code: 0x%X", xReturned);
    // }
}
