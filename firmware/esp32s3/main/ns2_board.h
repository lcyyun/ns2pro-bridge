#pragma once

#include "esp_err.h"

namespace ns2 {

esp_err_t board_controls_init();
void board_controls_task();

} // namespace ns2
