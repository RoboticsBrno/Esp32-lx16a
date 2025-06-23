/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ESP-IDF 5.4.1 compatible version of half_duplex_uart
 * Based on latest ESP-IDF UART example patterns
 */

#include "half_duplex_uart_541.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstring>

static const char *TAG = "half_duplex_uart_541";

namespace lx16a {
namespace half_duplex {

// Structure to hold half-duplex UART state
typedef struct {
    uart_port_t uart_num;
    gpio_num_t half_duplex_pin;
    SemaphoreHandle_t tx_mutex;
    bool is_installed;
} half_duplex_uart_context_t;

static half_duplex_uart_context_t uart_contexts[UART_NUM_MAX] = {};

esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, 
                             int queue_size, QueueHandle_t* uart_queue, int intr_alloc_flags) {
    if (uart_num >= UART_NUM_MAX) {
        ESP_LOGE(TAG, "Invalid UART number: %d", uart_num);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = ::uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, 
                                         queue_size, uart_queue, intr_alloc_flags);
    
    if (ret == ESP_OK) {
        uart_contexts[uart_num].uart_num = uart_num;
        uart_contexts[uart_num].is_installed = true;
        uart_contexts[uart_num].tx_mutex = xSemaphoreCreateMutex();
        
        if (uart_contexts[uart_num].tx_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create mutex for UART %d", uart_num);
            ::uart_driver_delete(uart_num);
            return ESP_ERR_NO_MEM;
        }
    }
    
    return ret;
}

esp_err_t uart_driver_delete(uart_port_t uart_num) {
    if (uart_num >= UART_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (uart_contexts[uart_num].tx_mutex) {
        vSemaphoreDelete(uart_contexts[uart_num].tx_mutex);
        uart_contexts[uart_num].tx_mutex = NULL;
    }
    
    uart_contexts[uart_num].is_installed = false;
    uart_contexts[uart_num].half_duplex_pin = GPIO_NUM_NC;
    
    return ::uart_driver_delete(uart_num);
}

esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config) {
    return ::uart_param_config(uart_num, uart_config);
}

esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num, 
                      int rts_io_num, int cts_io_num) {
    return ::uart_set_pin(uart_num, tx_io_num, rx_io_num, rts_io_num, cts_io_num);
}

void uart_set_half_duplex_pin(uart_port_t uart_num, gpio_num_t pin) {
    if (uart_num >= UART_NUM_MAX || pin == GPIO_NUM_NC) {
        ESP_LOGE(TAG, "Invalid parameters for half-duplex pin setup");
        return;
    }

    uart_contexts[uart_num].half_duplex_pin = pin;
    
    // Configure the pin for half-duplex operation
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT_OUTPUT_OD; // Open drain for half-duplex
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Pull-up for idle state
    gpio_config(&io_conf);
    
    // Set UART pins - use the same pin for both TX and RX in half-duplex mode
    ::uart_set_pin(uart_num, pin, pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    
    ESP_LOGI(TAG, "Half-duplex pin %d configured for UART %d", pin, uart_num);
}

static void switch_to_tx_mode(uart_port_t uart_num) {
    half_duplex_uart_context_t *ctx = &uart_contexts[uart_num];
    if (ctx->half_duplex_pin != GPIO_NUM_NC) {
        // Set pin as output for transmission
        gpio_set_direction(ctx->half_duplex_pin, GPIO_MODE_OUTPUT);
        // Small delay to ensure pin mode is switched
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void switch_to_rx_mode(uart_port_t uart_num) {
    half_duplex_uart_context_t *ctx = &uart_contexts[uart_num];
    if (ctx->half_duplex_pin != GPIO_NUM_NC) {
        // Set pin as input for reception
        gpio_set_direction(ctx->half_duplex_pin, GPIO_MODE_INPUT);
        // Small delay to ensure pin mode is switched
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

int uart_tx_chars(uart_port_t uart_num, const char* buffer, uint32_t len) {
    if (uart_num >= UART_NUM_MAX || buffer == NULL || len == 0) {
        return -1;
    }

    half_duplex_uart_context_t *ctx = &uart_contexts[uart_num];
    if (!ctx->is_installed || ctx->tx_mutex == NULL) {
        ESP_LOGE(TAG, "UART %d not properly initialized", uart_num);
        return -1;
    }

    // Take mutex to ensure exclusive access during transmission
    if (xSemaphoreTake(ctx->tx_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take TX mutex for UART %d", uart_num);
        return -1;
    }

    int bytes_written = -1;
    
    // Switch to TX mode
    switch_to_tx_mode(uart_num);
    
    // Wait for any previous transmission to complete
    esp_err_t ret = ::uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Wait for TX done failed: %s", esp_err_to_name(ret));
    }
    
    // Send data
    bytes_written = ::uart_write_bytes(uart_num, buffer, len);
    
    if (bytes_written > 0) {
        // Wait for transmission to complete
        ret = ::uart_wait_tx_done(uart_num, pdMS_TO_TICKS(100));
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Wait for TX completion failed: %s", esp_err_to_name(ret));
        }
    }
    
    // Switch back to RX mode
    switch_to_rx_mode(uart_num);
    
    // Release mutex
    xSemaphoreGive(ctx->tx_mutex);
    
    return bytes_written;
}

int uart_read_bytes(uart_port_t uart_num, void* buf, uint32_t length, TickType_t ticks_to_wait) {
    return ::uart_read_bytes(uart_num, buf, length, ticks_to_wait);
}

esp_err_t uart_flush_input(uart_port_t uart_num) {
    return ::uart_flush_input(uart_num);
}

esp_err_t uart_flush(uart_port_t uart_num) {
    return ::uart_flush(uart_num);
}

esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait) {
    return ::uart_wait_tx_done(uart_num, ticks_to_wait);
}

esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t* size) {
    return ::uart_get_buffered_data_len(uart_num, size);
}

esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate) {
    return ::uart_set_baudrate(uart_num, baudrate);
}

esp_err_t uart_get_baudrate(uart_port_t uart_num, uint32_t* baudrate) {
    return ::uart_get_baudrate(uart_num, baudrate);
}

esp_err_t uart_set_word_length(uart_port_t uart_num, uart_word_length_t data_bit) {
    return ::uart_set_word_length(uart_num, data_bit);
}

esp_err_t uart_get_word_length(uart_port_t uart_num, uart_word_length_t* data_bit) {
    return ::uart_get_word_length(uart_num, data_bit);
}

esp_err_t uart_set_stop_bits(uart_port_t uart_num, uart_stop_bits_t stop_bits) {
    return ::uart_set_stop_bits(uart_num, stop_bits);
}

esp_err_t uart_get_stop_bits(uart_port_t uart_num, uart_stop_bits_t* stop_bits) {
    return ::uart_get_stop_bits(uart_num, stop_bits);
}

esp_err_t uart_set_parity(uart_port_t uart_num, uart_parity_t parity_mode) {
    return ::uart_set_parity(uart_num, parity_mode);
}

esp_err_t uart_get_parity(uart_port_t uart_num, uart_parity_t* parity_mode) {
    return ::uart_get_parity(uart_num, parity_mode);
}

} // namespace half_duplex
} // namespace lx16a