/*
 * SPDX-FileCopyrightText: 2015-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 * ESP-IDF 5.4.1 compatible version of half_duplex_uart header
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
}
#endif

namespace lx16a {
namespace half_duplex {

/**
 * @brief Install UART driver and set the UART to the default configuration.
 *
 * UART ISR handler will be attached to the same CPU core that this function is running on.
 *
 * @note  Rx_buffer_size should be greater than UART_FIFO_LEN. Tx_buffer_size should be either zero or greater than UART_FIFO_LEN.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param rx_buffer_size UART RX ring buffer size.
 * @param tx_buffer_size UART TX ring buffer size.
 *        If set to zero, driver will not use TX buffer, TX function will block task until all data have been sent out.
 * @param queue_size UART event queue size/depth.
 * @param uart_queue UART event queue handle (out param). On success, a new queue handle is written here to provide
 *        access to UART events. If set to NULL, driver will not use an event queue.
 * @param intr_alloc_flags Flags used to allocate the interrupt. One or multiple (ORred)
 *        ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info. Do not set ESP_INTR_FLAG_IRAM here
 *        (the driver's ISR handler is not located in IRAM)
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size, int queue_size, QueueHandle_t* uart_queue, int intr_alloc_flags);

/**
 * @brief Uninstall UART driver.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_driver_delete(uart_port_t uart_num);

/**
 * @brief Set UART parameters.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param uart_config UART parameter settings
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config);

/**
 * @brief Configure pin for half-duplex operation.
 *
 * @param uart_num UART port number
 * @param pin GPIO pin number to use for half-duplex communication
 */
void uart_set_half_duplex_pin(uart_port_t uart_num, gpio_num_t pin);

/**
 * @brief Send data to the UART port with half-duplex mode switching.
 *
 * This function will automatically switch the configured pin to TX mode,
 * send the data, wait for transmission completion, then switch back to RX mode.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param buffer   data buffer address
 * @param len      data length to send
 *
 * @return
 *     - (-1) Parameter error
 *     - OTHERS (>=0) The number of bytes written
 */
int uart_tx_chars(uart_port_t uart_num, const char* buffer, uint32_t len);

/**
 * @brief Read data from UART.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param buf pointer to the buffer.
 * @param length data length
 * @param ticks_to_wait sTimeout, count in RTOS ticks
 *
 * @return
 *     - (-1) Error
 *     - OTHERS (>=0) The number of bytes read from UART FIFO
 */
int uart_read_bytes(uart_port_t uart_num, void* buf, uint32_t length, TickType_t ticks_to_wait);

/**
 * @brief Clear input buffer, discard all the data is in the ring-buffer.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_flush_input(uart_port_t uart_num);

/**
 * @brief UART flush. This will wait for TX FIFO to be empty and force flush the RX FIFO.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_flush(uart_port_t uart_num);

/**
 * @brief Wait until UART tx memory empty and the last char send ok (polling mode).
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param ticks_to_wait Timeout, count in RTOS ticks
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_TIMEOUT Timeout
 */
esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait);

/**
 * @brief Get the number of rxd bytes available to read from the UART FIFO.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param size Pointer to size. This will be set to the number of bytes available
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_get_buffered_data_len(uart_port_t uart_num, size_t* size);

/**
 * @brief Set UART baud rate.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param baudrate UART baud rate.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_baudrate(uart_port_t uart_num, uint32_t baudrate);

/**
 * @brief Get UART baud rate.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param baudrate Pointer to accept value of UART baud rate
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_get_baudrate(uart_port_t uart_num, uint32_t* baudrate);

/**
 * @brief Set UART data bits.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param data_bit UART data bits
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_word_length(uart_port_t uart_num, uart_word_length_t data_bit);

/**
 * @brief Get UART data bits.
 *
 * @param uart_num UART port number, the max port number is (UART_NUM_MAX -1).
 * @param data_bit Pointer to accept value of UART data bits.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_get_word_length(uart_port_t uart_num, uart_word_length_t* data_bit);

/**
 * @brief Set UART stop bits.
 *
 * @param uart_num  UART port number, the max port number is (UART_NUM_MAX -1).
 * @param stop_bits  UART stop bits
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Fail
 */
esp_err_t uart_set_stop_bits(uart_port_t uart_num, uart_stop_bits_t stop_bits);

/**
 * @brief Get UART stop bits.
 *
 * @param uart_num  UART port number, the max port number is (UART_NUM_MAX -1).
 * @param stop_bits  Pointer to accept value of UART stop bits.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Fail
 */
esp_err_t uart_get_stop_bits(uart_port_t uart_num, uart_stop_bits_t* stop_bits);

/**
 * @brief Set UART parity mode.
 *
 * @param uart_num   UART port number, the max port number is (UART_NUM_MAX -1).
 * @param parity_mode the enum of uart parity configuration
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_set_parity(uart_port_t uart_num, uart_parity_t parity_mode);

/**
 * @brief Get UART parity mode.
 *
 * @param uart_num   UART port number, the max port number is (UART_NUM_MAX -1).
 * @param parity_mode Pointer to accept value of UART parity mode.
 *
 * @return
 *     - ESP_OK   Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t uart_get_parity(uart_port_t uart_num, uart_parity_t* parity_mode);

} // namespace half_duplex
} // namespace lx16a
