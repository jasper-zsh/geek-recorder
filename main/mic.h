#pragma once
#include "esp_err.h"
#include <stdbool.h>

esp_err_t mic_init(void);
void mic_start_task(void);
void mic_set_record_request(bool on);
bool mic_is_recording(void);

