#include <tsdf_localization/util/imu_accumulator.h>

namespace tsdf_localization
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

} // namespace tsdf_localization 