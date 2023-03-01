#ifndef INTCONSTANT_H
#define INTCONSTANT_H

#include <util/util.h>

namespace int_mcl
{
    using FIXED_T = int;

    constexpr int INT_RESOLUTION = 1024;
    constexpr int MATRIX_RESOLUTION = 1000;

    constexpr long MAX_RANGE_FIXED = mcl::MAX_RANGE * INT_RESOLUTION;
    constexpr long INV_MAX_RANGE_FIXED = mcl::INV_MAX_RANGE * INT_RESOLUTION;

    constexpr FIXED_T A_HIT_FIXED = mcl::A_HIT * INT_RESOLUTION;
    constexpr FIXED_T A_RAND_FIXED = mcl::A_RAND * INT_RESOLUTION;
    constexpr FIXED_T A_MAX_FIXED = mcl::A_MAX * INT_RESOLUTION;
}

#endif