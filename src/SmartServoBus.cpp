#include "SmartServoBus.hpp"
#include <algorithm>
#include <chrono>
#include <cstring>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#include "./EspUartBackend.hpp"
#include "half_duplex_uart.h"

#define TAG "SmartServoBus"
#define MS_TO_TICKS(ms) ((portTICK_PERIOD_MS <= ms) ? (ms / portTICK_PERIOD_MS) : 1)

namespace lx16a {

const SmartServoBus::AutoStopParams SmartServoBus::DefaultAutoStopParams = {
    .max_diff_centideg = 2000,
    .max_diff_readings = 3,
};

SmartServoBus::SmartServoBus()
    : m_auto_stop_params(DefaultAutoStopParams) {}

void SmartServoBus::begin(uint8_t servo_count, uart_port_t uart, gpio_num_t pin, uint32_t tasks_stack_size) {
    m_espUartBackend.reset(new EspUartBackend());
    m_espUartBackend->begin(uart, pin, tasks_stack_size);

    begin(servo_count, m_espUartBackend.get(), tasks_stack_size);
}

void SmartServoBus::begin(uint8_t servo_count, BusBackend* backend, uint32_t tasks_stack_size) {
    if (backend != m_espUartBackend.get()) {
        m_espUartBackend.reset();
    }

    m_backend = backend;

    if (!m_servos.empty() || servo_count == 0)
        return;

    m_servos.resize(servo_count);

    TaskHandle_t task;
    xTaskCreate(&SmartServoBus::regulatorRoutineTrampoline, "rbservo_reg", tasks_stack_size, this, 2, &task);

    Angle val;
    for (uint8_t i = 0; i < servo_count; ++i) {
        for (int x = 0; x < 3; ++x) {
            val = pos(i);
            if (!val.isNaN()) {
                break;
            } else {
                ESP_LOGW(TAG, "failed to read servo %d pos, attempt %d", i, x + 1);
            }
        }

        if (val.isNaN()) {
            ESP_LOGE(TAG, "failed to read position from servo %d, it will not work!", i);
            continue;
        }

        const uint16_t deg_val = 100 * val.deg();

        m_mutex.lock();
        m_servos[i].current = deg_val;
        m_servos[i].target = deg_val;
        m_mutex.unlock();
    }
}

void SmartServoBus::setId(uint8_t newId, uint8_t destId) {
    auto pkt = lw::Packet::setId(destId, newId);
    send(pkt);
}

uint8_t SmartServoBus::getId(uint8_t destId) {
    BusBackend::rx_response resp;
    sendAndReceive(lw::Packet::getId(destId), resp);
    if (resp.size != 7)
        return 0xFF;
    return resp.data[5];
}

void SmartServoBus::set(uint8_t id, Angle ang, float speed, float speed_raise) {
    speed = std::max(1.f, std::min(240.f, speed)) / 10.f;
    const uint16_t angle = std::max(0.f, std::min(360.f, (float)ang.deg())) * 100;

    std::lock_guard<std::mutex> lock(m_mutex);

    auto& si = m_servos[id];
    if (!si.hasValidCurrent()) {
        const auto cur = pos(id);
        if (cur.isNaN()) {
            ESP_LOGE(TAG, "failed to get servo %d position, can't move it!", int(id));
            return;
        }
        const uint16_t deg_val = 100 * cur.deg();
        si.current = deg_val;
        si.target = deg_val;
    }

    if (si.current == angle)
        return;

    if ((si.current > si.target) != (si.current > angle)) {
        si.speed_coef = 0.f;
    }

    si.target = angle;
    si.speed_target = speed;
    si.speed_raise = speed_raise;
}

Angle SmartServoBus::pos(uint8_t id) {
    lw::Packet pkt(id, lw::Command::SERVO_POS_READ);

    BusBackend::rx_response resp;
    sendAndReceive(pkt, resp, true);
    if (resp.size != 0x08) {
        return Angle::nan();
    }

    uint16_t val = ((resp.data[6] << 8) | resp.data[5]);

    // The servo's angle counter can underflow when it moves
    // to the 0 position. Handle this case by reseting it to 0.
    if (val > 32767) {
        val = 0;
    } else if (val > 1000) {
        val = 1000;
    }

    return Angle::deg((float(val) / 1000.f) * 240.f);
}

Angle SmartServoBus::posOffline(uint8_t id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    auto& s = m_servos[id];
    if (s.current == 0xFFFF)
        return Angle::nan();
    return Angle::deg(Angle::_T(s.current) / 100.f);
}

void SmartServoBus::limit(uint8_t id, Angle bottom, Angle top) {
    auto pkt = lw::Servo::limit(id, bottom, top);
    send(pkt);
}

void SmartServoBus::setAutoStop(uint8_t id, bool enable) {
    m_mutex.lock();
    m_servos[id].auto_stop = enable;
    m_mutex.unlock();
}

void SmartServoBus::regulatorRoutineTrampoline(void* cookie) { ((SmartServoBus*)cookie)->regulatorRoutine(); }

void SmartServoBus::regulatorRoutine() {
    const size_t servos_cnt = m_servos.size();

    constexpr uint32_t msPerServo = 30;
    constexpr auto ticksPerServo = MS_TO_TICKS(msPerServo);
    const uint32_t msPerIter = servos_cnt * msPerServo;
    const auto ticksPerIter = MS_TO_TICKS(msPerIter);

    auto queue = xQueueCreate(1, sizeof(BusBackend::rx_response));
    while (true) {
        const auto tm_iter_start = xTaskGetTickCount();
        for (size_t i = 0; i < servos_cnt; ++i) {
            const auto tm_servo_start = xTaskGetTickCount();
            regulateServo(queue, i, msPerIter);
            const auto diff = xTaskGetTickCount() - tm_servo_start;
            if (diff < ticksPerServo) {
                vTaskDelay(ticksPerServo - diff);
            }
        }

        const auto diff = xTaskGetTickCount() - tm_iter_start;
        if (diff < ticksPerIter) {
            vTaskDelay(ticksPerIter - diff);
        }
    }
}

bool SmartServoBus::regulateServo(QueueHandle_t responseQueue, size_t id, uint32_t timeSliceMs) {
    float move_pos_deg;
    auto& s = m_servos[id];
    struct BusBackend::rx_response resp;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (s.current == s.target) {
            return false;
        }

        if (s.auto_stop) {
            lw::Packet pos_req(id, lw::Command::SERVO_POS_READ);
            send(pos_req, responseQueue, true);
            xQueueReceive(responseQueue, &resp, portMAX_DELAY);
            if (resp.size == 0x08) {
                const float val = (float)((resp.data[6] << 8) | resp.data[5]);
                const int16_t val_int = (val / 1000.f) * 24000.f;
                const int32_t diff = val_int - int32_t(s.current);
                if (std::abs(diff) > m_auto_stop_params.max_diff_centideg) {
                    if (++s.auto_stop_counter >= m_auto_stop_params.max_diff_readings) {
                        s.target = val_int + (diff > 0 ? -500 : 500);
                        s.auto_stop_counter = 0;
                    }
                } else if (s.auto_stop_counter != 0) {
                    s.auto_stop_counter = 0;
                }
            }
        }

        float speed = s.speed_target;
        if (s.speed_coef < 1.f) {
            s.speed_coef = std::min(1.f, s.speed_coef + (s.speed_raise * timeSliceMs));
            speed *= (s.speed_coef * s.speed_coef);
        }

        int32_t dist = std::abs(int32_t(s.target) - int32_t(s.current));
        dist = std::max(int32_t(1), std::min(dist, int32_t(speed * timeSliceMs)));
        if (dist > 0) {
            if (s.target < s.current) {
                s.current -= dist;
            } else {
                s.current += dist;
            }
        }

        if (dist <= 0 || s.current == s.target) {
            s.current = s.target;
            s.speed_coef = 0.f;
        }
        move_pos_deg = float(s.current) / 100.f;
    }

    const auto pkt = lw::Servo::move(id, Angle::deg(move_pos_deg), std::chrono::milliseconds(timeSliceMs - 5));
    send(pkt, responseQueue, false, true);

    if (xQueueReceive(responseQueue, &resp, 500 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(TAG, "Response to move packet not received!");
    }
    return true;
}

void SmartServoBus::send(const lw::Packet& pkt, QueueHandle_t responseQueue, bool expect_response, bool priority) {
    m_backend->send(pkt, responseQueue, expect_response, priority);
}

void SmartServoBus::sendAndReceive(const lw::Packet& pkt, struct BusBackend::rx_response& res, bool to_front) {
    memset(&res, 0, sizeof(BusBackend::rx_response));

    auto queue = xQueueCreate(1, sizeof(BusBackend::rx_response));
    send(pkt, queue, true);
    xQueueReceive(queue, &res, portMAX_DELAY);
    vQueueDelete(queue);
}

}; // namespace lx16a
