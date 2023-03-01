#ifndef CUDA_SUB_VOXEL_MAP_H
#define CUDA_SUB_VOXEL_MAP_H

#include <util/util.h>
#include <cuda/cuda_util.h>
#include <vector>



namespace mcl
{

using OCC_T = int;

constexpr FLOAT_T sub_voxel_size = 1.0;

template <typename DataT, typename IndexT>
class CudaSubVoxelMap
{
public:

    struct MapCoef
    {
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

    size_t up_dim_x_;
    size_t up_dim_y_;
    size_t up_dim_z_;
    size_t up_dim_2_;

    size_t sub_dim_;
    size_t sub_dim_2_;

    size_t grid_occ_size_;
    size_t data_size_;
    };

    HOST_CALLABLE_MEMBER CudaSubVoxelMap(IndexT min_x , IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value = 0);

    HOST_CALLABLE_MEMBER void setTrackReuse(bool track_reuse)
    {
        track_reuse_ = track_reuse;
    }

    HOST_CALLABLE_MEMBER bool trackReuse()
    {
        return track_reuse_;
    }

    HOST_CALLABLE_MEMBER bool isNewTrack()
    {
        return new_track_;
    }

    HOST_CALLABLE_MEMBER int getTrackCount()
    {
        return track_count_;
    }

    HOST_CALLABLE_MEMBER int getLastTrack()
    {
        return last_track_;
    }

    HOST_CALLABLE_MEMBER int getInvalidCount()
    {
        return invalid_count_;
    }

    HOST_CALLABLE_MEMBER void resetInvalidCount()
    {
        invalid_count_ = 0;
    }

    HOST_CALLABLE_MEMBER void resetOccReuse()
    {
        for (auto index = 0; index < map_coef.grid_occ_size_; ++index)
        {
            occ_reuse_[index] = 0;
        }
    }

    CUDA_CALLABLE_MEMBER DataT getEntry(IndexT x, IndexT y, IndexT z);
    CUDA_CALLABLE_MEMBER void setEntry(IndexT x, IndexT y, IndexT z, DataT value);

    CUDA_CALLABLE_MEMBER size_t size() 
    {
        return map_coef.data_size_ * sizeof(DataT) + map_coef.grid_occ_size_ * sizeof(long);
    }

    HOST_CALLABLE_MEMBER void setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data);

    HOST_CALLABLE_MEMBER ~CudaSubVoxelMap()
    {
        if (grid_occ_ != nullptr)
        {
            delete[] grid_occ_;
            grid_occ_ = nullptr;
        }

        if (data_ != nullptr)
        {
            delete[] data_;
            data_ = nullptr;
        }
    }

    CUDA_CALLABLE_MEMBER CudaSubVoxelMap(const CudaSubVoxelMap&) = delete;
    CUDA_CALLABLE_MEMBER CudaSubVoxelMap& operator=(const CudaSubVoxelMap&) = delete;

    HOST_CALLABLE_MEMBER CudaSubVoxelMap(CudaSubVoxelMap&& other)
    {
        move(other);
    }
    
    HOST_CALLABLE_MEMBER CudaSubVoxelMap& operator=(CudaSubVoxelMap&& other)
    {
        if (this == &other)
        {
            return *this;
        }

        move(other);
        return *this;
    }

    CUDA_CALLABLE_MEMBER OCC_T* rawGridOcc()
    {
        return grid_occ_;
    }

    CUDA_CALLABLE_MEMBER OCC_T** rawGridOccPtr()
    {
        return &grid_occ_;
    }

    HOST_CALLABLE_MEMBER long* occReuse()
    {
        return occ_reuse_;
    }

    CUDA_CALLABLE_MEMBER size_t gridOccSize() const
    {
        return map_coef.grid_occ_size_;
    }

    CUDA_CALLABLE_MEMBER size_t gridOccBytes() const
    {
        return map_coef.grid_occ_size_ * sizeof(OCC_T);
    }

    CUDA_CALLABLE_MEMBER DataT* rawData()
    {
        return data_;
    }

    CUDA_CALLABLE_MEMBER DataT** rawDataPtr()
    {
        return &data_;
    }

    CUDA_CALLABLE_MEMBER size_t subGridSize() const
    {
        return map_coef.sub_dim_ * map_coef.sub_dim_ * map_coef.sub_dim_;
    }

    CUDA_CALLABLE_MEMBER size_t subGridBytes() const
    {
        return subGridSize() * sizeof(DataT);
    }

    CUDA_CALLABLE_MEMBER size_t dataSize() const
    {
        return map_coef.data_size_;
    }

    CUDA_CALLABLE_MEMBER size_t dataBytes() const
    {
        return map_coef.data_size_ * sizeof(DataT);
    }

    CUDA_CALLABLE_MEMBER size_t getDimX()
    {
        return map_coef.dim_x_;
    }

    CUDA_CALLABLE_MEMBER size_t getDimY()
    {
        return map_coef.dim_y_;
    }

    CUDA_CALLABLE_MEMBER size_t getDimZ()
    {
        return map_coef.dim_z_;
    }

    CUDA_CALLABLE_MEMBER size_t getUpDimX()
    {
        return map_coef.up_dim_x_;
    }

    CUDA_CALLABLE_MEMBER size_t getUpDimY()
    {
        return map_coef.up_dim_y_;
    }

    CUDA_CALLABLE_MEMBER size_t getUpDimZ()
    {
        return map_coef.up_dim_z_;
    }

    CUDA_CALLABLE_MEMBER size_t getSubDim()
    {
        return map_coef.sub_dim_;
    }

    CUDA_CALLABLE_MEMBER  IndexT getMinX()
    {
        return map_coef.min_x_;
    }

    CUDA_CALLABLE_MEMBER  IndexT getMinY()
    {
        return map_coef.min_y_;
    }

    CUDA_CALLABLE_MEMBER IndexT getMinZ()
    {
        return map_coef.min_z_;
    }

    CUDA_CALLABLE_MEMBER IndexT getMaxX()
    {
        return map_coef.max_x_;
    }

    CUDA_CALLABLE_MEMBER IndexT getMaxY()
    {
        return map_coef.max_y_;
    }

    CUDA_CALLABLE_MEMBER IndexT getMaxZ()
    {
        return map_coef.max_z_;
    }

    CUDA_CALLABLE_MEMBER IndexT getResolution()
    {
        return map_coef.resolution_;
    }

    CUDA_CALLABLE_MEMBER DataT getInitValue()
    {
        return map_coef.init_value_;
    }

    CUDA_CALLABLE_MEMBER MapCoef& coef()
    {
        return map_coef;
    }

private:
    void move(CudaSubVoxelMap& other)
    {
        grid_occ_ = other.grid_occ_;
        data_ = other.data_;
        occ_reuse_ = other.occ_reuse_;

        map_coef.grid_occ_size_ = other.map_coef.grid_occ_size_;
        map_coef.data_size_ = other.map_coef.data_size_;

        map_coef.up_dim_x_ = other.map_coef.up_dim_x_;
        map_coef.up_dim_y_ = other.map_coef.up_dim_y_;
        map_coef.up_dim_z_ = other.map_coef.up_dim_z_;
        map_coef.up_dim_2_ = other.map_coef.up_dim_2_;

        map_coef.sub_dim_ = other.map_coef.sub_dim_;
        map_coef.sub_dim_2_ = other.map_coef.sub_dim_2_;

        map_coef.grid_occ_size_ = other.map_coef.grid_occ_size_;
        map_coef.data_size_ = other.map_coef.data_size_;

        other.grid_occ_ = nullptr;
        other.data_ = nullptr;
        other.occ_reuse_ = nullptr;

        other.map_coef.grid_occ_size_ = 0;
        other.map_coef.data_size_ = 0;

        map_coef.dim_x_ = other.map_coef.dim_x_;
        map_coef.dim_y_ = other.map_coef.dim_y_;
        map_coef.dim_z_ = other.map_coef.dim_z_;

        map_coef.min_x_ = other.map_coef.min_x_;
        map_coef.min_y_ = other.map_coef.min_y_;
        map_coef.min_z_ = other.map_coef.min_z_;

        map_coef.max_x_ = other.map_coef.max_x_;
        map_coef.max_y_ = other.map_coef.max_y_;
        map_coef.max_z_ = other.map_coef.max_z_;

        map_coef.resolution_ = other.map_coef.resolution_;
        map_coef.init_value_ = other.map_coef.init_value_;

        track_reuse_ = other.track_reuse_;
        new_track_ = other.new_track_;
        track_count_ = other.track_count_;
        last_track_ = other.last_track_;
        last_index_ = other.last_index_;
        invalid_count_ = other.invalid_count_;
    }

    CUDA_CALLABLE_MEMBER size_t getIndex(IndexT x, IndexT y, IndexT z);

    OCC_T* grid_occ_;
    DataT* data_;

    MapCoef map_coef;

    bool track_reuse_;
    bool new_track_;
    int track_count_;
    int last_track_;
    int last_index_;
    int invalid_count_;

    long* occ_reuse_;
};

} // namespace mcl

#include <cuda/cuda_sub_voxel_map.tcc>

#endif