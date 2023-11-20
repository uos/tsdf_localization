#ifndef INTCONSTANT_H
#define INTCONSTANT_H

#include <tsdf_localization/util/util.h>

namespace int_tsdf_localization
{
    using FIXED_T = int;

    constexpr int INT_RESOLUTION = 1024;
    constexpr int MATRIX_RESOLUTION = 1000;

    constexpr long MAX_RANGE_FIXED = tsdf_localization::MAX_RANGE * INT_RESOLUTION;
    constexpr long INV_MAX_RANGE_FIXED = tsdf_localization::INV_MAX_RANGE * INT_RESOLUTION;

    constexpr FIXED_T A_HIT_FIXED = tsdf_localization::A_HIT * INT_RESOLUTION;
    constexpr FIXED_T A_RAND_FIXED = tsdf_localization::A_RAND * INT_RESOLUTION;
    constexpr FIXED_T A_MAX_FIXED = tsdf_localization::A_MAX * INT_RESOLUTION;
} // namespace int_tsdf_localization

#endif // INTCONSTANT_H