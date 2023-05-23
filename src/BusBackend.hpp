#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "./lx16a.hpp"

namespace lx16a {

class BusBackend {
public:
    BusBackend() {}
    BusBackend(const BusBackend&) = delete;
    virtual ~BusBackend() {}

    struct rx_response {
        uint8_t data[16];
        uint8_t size;
    };

    // If used, responseQueue should be queue of struct rx_response
    virtual void send(const lw::Packet& pkt, QueueHandle_t responseQueue = NULL,
        bool expect_response = false, bool priority = false) = 0;
};

};
