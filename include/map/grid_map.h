#ifndef GRIDMAP_H
#define GRIDMAP_H

#include <array>
#include <memory>
#include <vector>
#include <cmath>

#include <util/tsdf.h>
#include <int_mcl/int_constant.h>

#include <cuda/cuda_util.h>
#include <util/util.h>

namespace mcl
{

constexpr int CHUNK_SHIFT = 6;
constexpr int CHUNK_SIZE = 1 << CHUNK_SHIFT;

constexpr int MAP_SHIFT = 6;
constexpr int MAP_RESOLUTION = 1 << MAP_SHIFT;

constexpr FLOAT_T truncation = 600; //600;

template<typename DataT, typename IndexT>
class GridMap
{
public:
    GridMap(IndexT min_x , IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value = 0) :
    dim_x_(std::ceil(std::abs(max_x - min_x) / resolution)), 
    dim_y_(std::ceil(std::abs(max_y - min_y) / resolution)),
    dim_z_(std::ceil(std::abs(max_z - min_z) / resolution)),
    min_x_(min_x),
    min_y_(min_y),
    min_z_(min_z),
    max_x_(max_x),
    max_y_(max_y),
    max_z_(max_z),
    resolution_(resolution),
    init_value_(init_value)
    {

    }

    #ifdef __CUDACC__
        GridMap(const GridMap&) = default;
        GridMap& operator=(const GridMap&) = default;    
    #endif

    GridMap(GridMap&&) = default;
    GridMap& operator=(GridMap&&) = default;

    virtual void setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data) = 0;

    CUDA_CALLABLE_MEMBER virtual DataT getEntry(IndexT x, IndexT y, IndexT z) const = 0;
    CUDA_CALLABLE_MEMBER virtual void setEntry(IndexT x, IndexT y, IndexT z, DataT value) = 0;

    CUDA_CALLABLE_MEMBER virtual size_t size() = 0;

    CUDA_CALLABLE_MEMBER virtual size_t getDimX()
    {
        return dim_x_;
    }

    CUDA_CALLABLE_MEMBER virtual size_t getDimY()
    {
        return dim_y_;
    }

    CUDA_CALLABLE_MEMBER virtual size_t getDimZ()
    {
        return dim_z_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getMinX()
    {
        return min_x_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getMinY()
    {
        return min_y_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getMinZ()
    {
        return min_z_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getMaxX()
    {
        return max_x_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getMaxY()
    {
        return max_y_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getMaxZ()
    {
        return max_z_;
    }

    CUDA_CALLABLE_MEMBER virtual IndexT getResolution()
    {
        return resolution_;
    }

    CUDA_CALLABLE_MEMBER virtual DataT getInitValue()
    {
        return init_value_;
    }

protected:
    size_t dim_x_;
    size_t dim_y_;
    size_t dim_z_;

    IndexT min_x_;
    IndexT min_y_;
    IndexT min_z_;

    IndexT max_x_;
    IndexT max_y_;
    IndexT max_z_;

    IndexT resolution_;

    DataT init_value_;
};

} // namespace mcl

#endif