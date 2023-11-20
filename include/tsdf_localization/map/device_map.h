#ifndef DEVICE_MAP_H
#define DEVICE_MAP_H

#include <tsdf_localization/cuda/cuda_util.h>

namespace tsdf_localization
{

template<typename DataT, typename IndexT>
class DeviceMap
{
public: 
    CUDA_CALLABLE_MEMBER virtual DataT getEntry(IndexT x, IndexT y, IndexT z) const = 0;
    CUDA_CALLABLE_MEMBER virtual void setEntry(IndexT x, IndexT y, IndexT z, DataT value) = 0;
};

}

#endif