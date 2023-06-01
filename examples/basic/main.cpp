#ifdef LX16A_ARDUINO
#include <Arduino.h>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
static void delay(int ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
#endif

#include "SmartServoBus.hpp"

using namespace lx16a;

static int n = 0;
static SmartServoBus servoBus;

void setup() {
    // Servos on the bus must have sequential IDs, starting from 0 (not 1)!
    servoBus.begin(1, UART_NUM_2, GPIO_NUM_14);

    /*
    // Set servo Id (must be only one servo connected to the bus)
    servoBus.setId(0);
    while (true) {
        printf("GetId: %d\n", servoBus.getId());
        delay(1000);
    }
    */
}

void loop() {
    uint16_t angle = (n % 240);

    servoBus.set(0, Angle::deg(angle));
    printf("Move to %d \n", angle);

    delay(500);

    auto curPos = servoBus.pos(0);
    printf("Position at %f \n", curPos.deg());

    n += 15;
    delay(1000);
}

#ifndef ARDUINO
extern "C" void app_main() {
    setup();
    while (true) {
        loop();
        vTaskDelay(0);
    }
}
#endif
