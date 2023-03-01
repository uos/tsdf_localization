#include <util/imu_accumulator.h>

namespace mcl
{

void bound_angle(FLOAT_T& angle)
{
    if (angle >= M_PI)
    {
        angle = -M_PI + (angle - M_PI);
    }
    else if (angle < -M_PI)
    {
        angle = M_PI - (angle - M_PI);
    }
}

} // namespace mcl 