#pragma once

#include <mutex>
#include <vector>

#include <driver/gpio.h>
#include <driver/pcnt.h>

#include "./angle.hpp"
#include "./lx16a.hpp"
#include "./BusBackend.hpp"
#include "./EspUartBackend.hpp"
#include <memory>

namespace lx16a {

class SmartServoBus {
public:
    struct AutoStopParams {
        // Maximum difference between expected and actual servo angle before the AutoStop Triggers
        // It is in centidegrees, (1deg == 100 centideg). Default: 2000 (20 degrees)
        int16_t max_diff_centideg;

        // How many times must the difference between expected and actual servo angle
        // fall outside of max_diff_centideg before the servo is stopped. Default: 3
        uint8_t max_diff_readings;
    };
    static const AutoStopParams DefaultAutoStopParams;

    SmartServoBus();
    ~SmartServoBus() {}

    void begin(uint8_t servo_count, uart_port_t uart, gpio_num_t pin, uint32_t tasks_stack_size = 2560);
    void begin(uint8_t servo_count, BusBackend *backend, uint32_t tasks_stack_size = 2560);

    void set(uint8_t id, Angle ang, float speed = 200.f, float speed_raise = 0.0015f);
    void limit(uint8_t id, Angle bottom, Angle top);

    Angle pos(uint8_t id);
    Angle posOffline(uint8_t id);

    void setAutoStop(uint8_t id, bool enable = true);
    void setAutoStopParams(const AutoStopParams& params = DefaultAutoStopParams) {
        m_auto_stop_params = params;
    }

    void setId(uint8_t newId, uint8_t destId = 254);
    uint8_t getId(uint8_t destId = 254);

private:
    SmartServoBus(const SmartServoBus&) = delete;

    static void regulatorRoutineTrampoline(void* cookie);
    void regulatorRoutine();
    bool regulateServo(QueueHandle_t responseQueue, size_t id, uint32_t timeSliceMs);

    struct servo_info {
        servo_info() {
            current = 0xFFFF;
            target = 0xFFFF;
            speed_coef = 0.f;
            speed_target = 0.f;
            speed_raise = 0.f;
            auto_stop = false;
            auto_stop_counter = 0;
        }

        bool hasValidCurrent() const { return current != 0xFFFF; }

        float speed_coef;
        float speed_target;
        float speed_raise;
        uint16_t current;
        uint16_t target;
        bool auto_stop;
        uint8_t auto_stop_counter;
    };

    void send(
        const lw::Packet& pkt, QueueHandle_t responseQueue = NULL, bool expect_response = false, bool priority = false);
    void sendAndReceive(const lw::Packet& pkt, BusBackend::rx_response& res, bool to_front = false);

    AutoStopParams m_auto_stop_params;

    std::vector<servo_info>
        m_servos;
    std::mutex m_mutex;

    BusBackend *m_backend;
    std::unique_ptr<EspUartBackend> m_espUartBackend;
};

};
