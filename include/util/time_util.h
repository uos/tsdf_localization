#pragma once

#include <chrono>

namespace fastsense
{

namespace util
{

using HighResTime = std::chrono::high_resolution_clock;
using HighResTimePoint = HighResTime::time_point;

namespace time
{
using secs_double = std::chrono::duration<double>;
}

}

} // namespace time
