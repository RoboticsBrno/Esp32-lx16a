#include "./EspUartBackend.hpp"
#include "./half_duplex_uart.h"

#include <esp_log.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "EspUartBackend"
#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace lx16a {


EspUartBackend::EspUartBackend() : m_uart_queue(NULL), m_uart_task(NULL) {}
EspUartBackend::~EspUartBackend() {
    if(m_uart_queue != NULL) {
        tx_request req = {};
        req.size = TX_REQUEST_SIZE_STOP_REQUEST;
        xQueueSendToFront(m_uart_queue, &req, portMAX_DELAY);
    }
}

void EspUartBackend::begin(uart_port_t uart, gpio_num_t pin, uint32_t tasks_stack_size) {
    m_uart = uart;
    m_uart_pin = pin;

    m_uart_queue = xQueueCreate(8, sizeof(struct tx_request));

    xTaskCreatePinnedToCore(&EspUartBackend::uartRoutineTrampoline, "rbservo_uart", tasks_stack_size, this, 1, &m_uart_task, 1);
}


void EspUartBackend::uartRoutineTrampoline(void* cookie) { ((EspUartBackend*)cookie)->uartRoutine(); }

void EspUartBackend::uartRoutine() {
    {
        const uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        ESP_ERROR_CHECK(half_duplex::uart_param_config(m_uart, &uart_config));
        ESP_ERROR_CHECK(half_duplex::uart_driver_install(m_uart, 256, 0, 0, NULL, 0));
        half_duplex::uart_set_half_duplex_pin(m_uart, m_uart_pin);
    }

    struct tx_request req;
    struct rx_response resp;
    auto tm_last = xTaskGetTickCount();
    constexpr auto min_delay = MS_TO_TICKS(15);
    while (true) {
        if (xQueueReceive(m_uart_queue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        
        if(req.size == TX_REQUEST_SIZE_STOP_REQUEST) {
            break;
        }

        const auto diff = xTaskGetTickCount() - tm_last;
        if (diff < min_delay) {
            vTaskDelay(min_delay - diff);
        }

        half_duplex::uart_tx_chars(m_uart, req.data, req.size);
        tm_last = xTaskGetTickCount();
        req.size = uartReceive((uint8_t*)req.data, sizeof(req.data));

        if (req.size != 0 && req.expect_response) {
            resp.size = uartReceive(resp.data, sizeof(resp.data));
        } else {
            resp.size = 0;
        }

        if (req.responseQueue) {
            xQueueSend(req.responseQueue, &resp, 300 / portTICK_PERIOD_MS);
        }
    }

    half_duplex::uart_driver_delete(m_uart);
    vQueueDelete(m_uart_queue);
    vTaskDelete(NULL);
}

size_t EspUartBackend::uartReceive(uint8_t* buff, size_t bufcap) {
    constexpr TickType_t wait_period = MS_TO_TICKS(4);
    constexpr TickType_t timeout = MS_TO_TICKS(20);

    size_t bufsize = 0;

    while (true) {
        size_t avail = 0;
        size_t need = 0;
        const size_t oldsize = bufsize;
        switch (oldsize) {
        case 0:
        case 1:
            need = 1;
            break;
        case 2:
            need = 3;
            break;
        case 5:
            need = buff[3] - 2;
            break;
        default:
            return bufsize;
        }

        if (need + oldsize > bufcap) {
            ESP_LOGE(TAG, "invalid packet size received: %d.\n", (int)buff[3]);
            return 0;
        }

        TickType_t waiting = 0;
        while (half_duplex::uart_get_buffered_data_len(m_uart, &avail) != ESP_OK || avail < need) {
            if (waiting >= timeout) {
                ESP_LOGE(TAG, "timeout when waiting for data!");
                return 0;
            }
            vTaskDelay(wait_period);
            waiting += wait_period;
        }

        int res = half_duplex::uart_read_bytes(m_uart, buff + oldsize, need, 0);
        if (res < 0 || ((size_t)res) != need) {
            ESP_LOGE(TAG, "invalid packet read: %d, aborting.\n", res);
            abort();
            return 0;
        }
        bufsize += need;

        if (oldsize < 2 && buff[oldsize] != 0x55)
            bufsize = 0;
    }
    return 0;
}

void EspUartBackend::send(const lw::Packet& pkt, QueueHandle_t responseQueue, bool expect_response, bool priority) {
    struct tx_request req = { 0 };
    req.size = (uint8_t)pkt._data.size();
    req.expect_response = expect_response;
    req.responseQueue = responseQueue;

    if (sizeof(req.data) < pkt._data.size()) {
        ESP_LOGE(TAG, "packet is too big, %u > %u", pkt._data.size(), sizeof(req.data));
        abort();
    }
    memcpy(req.data, pkt._data.data(), pkt._data.size());
    if (priority) {
        xQueueSendToFront(m_uart_queue, &req, portMAX_DELAY);
    } else {
        xQueueSendToBack(m_uart_queue, &req, portMAX_DELAY);
    }

}

};
