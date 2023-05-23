#pragma once

#include <driver/gpio.h>
#include "./BusBackend.hpp"

namespace lx16a {


class EspUartBackend : public BusBackend {
public:
    EspUartBackend();
    ~EspUartBackend();
    void begin(uart_port_t uart, gpio_num_t pin, uint32_t tasks_stack_size = 2560);

    virtual void send(const lw::Packet& pkt, QueueHandle_t responseQueue = NULL, bool expect_response = false, bool priority = false);

private:
    static void uartRoutineTrampoline(void* cookie);
    void uartRoutine();
    size_t uartReceive(uint8_t* buff, size_t bufcap);

    static const constexpr uint8_t TX_REQUEST_SIZE_STOP_REQUEST = 0xFE;
    struct tx_request {
        char data[16];
        uint8_t size;
        bool expect_response;
        QueueHandle_t responseQueue;
    };


    QueueHandle_t m_uart_queue;
    TaskHandle_t m_uart_task;
    uart_port_t m_uart;
    gpio_num_t m_uart_pin;
};

};
