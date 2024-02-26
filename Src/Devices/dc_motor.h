#pragma once

#include <stdint.h>

void DCMotor_Init();
int64_t DCMotor_ReadEncoder(uint32_t motorId);
