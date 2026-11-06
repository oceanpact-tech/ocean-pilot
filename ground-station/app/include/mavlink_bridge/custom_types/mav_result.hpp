#pragma once

#include <cstdint>

namespace MavResult 
{
    constexpr uint8_t ACCEPTED             = 0;
    constexpr uint8_t TEMPORARILY_REJECTED = 1;
    constexpr uint8_t DENIED               = 2;
    constexpr uint8_t UNSUPPORTED          = 3;
    constexpr uint8_t FAILED               = 4;
    constexpr uint8_t IN_PROGRESS          = 5;
    constexpr uint8_t CANCELLED            = 6;
}