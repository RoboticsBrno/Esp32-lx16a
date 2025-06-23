#pragma once

#include "esp_system.h"

#if (!defined(ESP_IDF_VERSION) || ESP_IDF_VERSION < 0x040400)
#include "half_duplex_uart_33.h"
#elif (ESP_IDF_VERSION >= 0x040400 && ESP_IDF_VERSION < 0x050000)
#include "half_duplex_uart_44.h"
#else
#include "half_duplex_uart_541.h"
#endif
