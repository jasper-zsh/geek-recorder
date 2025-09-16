#pragma once
#include <stdbool.h>
#include <stddef.h>

void asr_ws_start(void);
bool asr_ws_connected(void);
bool asr_ws_streaming(void);
void asr_ws_send_audio(const void *data, size_t len);

