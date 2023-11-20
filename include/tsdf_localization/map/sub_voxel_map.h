#ifndef SUBVOXELMAP_H
#define SUBVOXELMAP_H

#include <tsdf_localization/map/grid_map.h>
#include <tsdf_localization/map/device_map.h>
#include <tsdf_localization/cuda/cuda_util.h>

namespace tsdf_localization
{

template <typename DataT, typename IndexT, int sub_voxel_size>
class SubVoxelMap : public DeviceMap<DataT, IndexT>, public GridMap<DataT, IndexT>
{
public:
    CUDA_CALLABLE_MEMBER SubVoxelMap(IndexT min_x , IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value = 0);

    CUDA_CALLABLE_MEMBER virtual DataT getEntry(IndexT x, IndexT y, IndexT z) const override;
    CUDA_CALLABLE_MEMBER virtual void setEntry(IndexT x, IndexT y, IndexT z, DataT value) override;

    CUDA_CALLABLE_MEMBER virtual size_t size() override
    {
        return data_size_ * sizeof(DataT) + grid_occ_size_ * sizeof(long);
    }

    HOST_CALLABLE_MEMBER virtual size_t getUpDimX()
    {
        return up_dim_x_;
    }

    HOST_CALLABLE_MEMBER virtual size_t getUpDimY()
    {
        return up_dim_y_;
    }

    HOST_CALLABLE_MEMBER virtual size_t getUpDimZ()
    {
        return up_dim_z_;
    }

    HOST_CALLABLE_MEMBER virtual void setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data) override;

    CUDA_CALLABLE_MEMBER virtual ~SubVoxelMap()
    {
        #ifndef __CUDACC__
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
        #endif
    }

    #ifndef __CUDACC__
        CUDA_CALLABLE_MEMBER SubVoxelMap(const SubVoxelMap&) = delete;
        CUDA_CALLABLE_MEMBER SubVoxelMap& operator=(const SubVoxelMap&) = delete;
    #else

        CUDA_CALLABLE_MEMBER void copy(const SubVoxelMap& other)
        {
            this->grid_occ_ = other.grid_occ_;
            this->data_ = other.data_;

            this->grid_occ_size_ = other.grid_occ_size_;
            this->data_size_ = other.data_size_;

            this->up_dim_x_ = other.up_dim_x_;
            this->up_dim_y_ = other.up_dim_y_;
            this->up_dim_z_ = other.up_dim_z_;
            this->up_dim_2_ = other.up_dim_2_;

            this->sub_dim_ = other.sub_dim_;
            this->sub_dim_2_ = other.sub_dim_2_;

            this->up_shift_ = other.up_shift_;

            this->grid_occ_size_ = other.grid_occ_size_;
            this->data_size_ = other.data_size_;
            this->occupancy_ = other.occupancy;
        }

        CUDA_CALLABLE_MEMBER SubVoxelMap(const SubVoxelMap& other) : Base(other)
        {
            copy(other);
        }
        
        CUDA_CALLABLE_MEMBER SubVoxelMap& operator=(const SubVoxelMap& other)
        {
            if (this == &other)
            {
                return *this;
            }

            Base::operator=(other);
            copy(other);
        };


    #endif

    HOST_CALLABLE_MEMBER SubVoxelMap(SubVoxelMap&& other) : Base(std::move(other))
    {
        move(other);
    }
    
    HOST_CALLABLE_MEMBER SubVoxelMap& operator=(SubVoxelMap&& other)
    {
        if (this == &other)
        {
            return *this;
        }

        Base::operator=(std::move(other));
        move(other);
        return *this;
    }

    CUDA_CALLABLE_MEMBER long* rawGridOcc()
    {
        return grid_occ_;
    }

    CUDA_CALLABLE_MEMBER long** rawGridOccPtr()
    {
        return &grid_occ_;
    }

    CUDA_CALLABLE_MEMBER size_t gridOccSize() const
    {
        return grid_occ_size_;
    }

    CUDA_CALLABLE_MEMBER size_t gridOccBytes() const
    {
        return grid_occ_size_ * sizeof(long);
    }

    CUDA_CALLABLE_MEMBER DataT* rawData()
    {
        return data_;
    }

    CUDA_CALLABLE_MEMBER DataT** rawDataPtr()
    {
        return &data_;
    }

    CUDA_CALLABLE_MEMBER size_t dataSize() const
    {
        return data_size_;
    }

    CUDA_CALLABLE_MEMBER size_t dataBytes() const
    {
        return data_size_ * sizeof(DataT);
    }

    HOST_CALLABLE_MEMBER double occupancy() const 
    {
        return occupancy_;
    }

    HOST_CALLABLE_MEMBER size_t subSize() const 
    {
        return sub_dim_ * sub_dim_ * sub_dim_;
    }

    HOST_CALLABLE_MEMBER size_t subBytes() const 
    {
        return subSize() * sizeof(DataT);
    }

private:
    void move(SubVoxelMap& other)
    {
        this->grid_occ_ = other.grid_occ_;
        this->data_ = other.data_;

        this->grid_occ_size_ = other.grid_occ_size_;
        this->data_size_ = other.data_size_;

        this->up_dim_x_ = other.up_dim_x_;
        this->up_dim_y_ = other.up_dim_y_;
        this->up_dim_z_ = other.up_dim_z_;
        this->up_dim_2_ = other.up_dim_2_;

        this->sub_dim_ = other.sub_dim_;
        this->sub_dim_2_ = other.sub_dim_2_;

        this->up_shift_ = other.up_shift_;

        this->grid_occ_size_ = other.grid_occ_size_;
        this->data_size_ = other.data_size_;

        this->occupancy_ = other.occupancy_;

        other.grid_occ_ = nullptr;
        other.data_ = nullptr;

        other.grid_occ_size_ = 0;
        other.data_size_ = 0;
    }

    CUDA_CALLABLE_MEMBER size_t getIndex(IndexT x, IndexT y, IndexT z) const;

    using Base = GridMap<DataT, IndexT>;

    long* grid_occ_;
    DataT* data_;

    size_t up_dim_x_;
    size_t up_dim_y_;
    size_t up_dim_z_;
    size_t up_dim_2_;

    size_t sub_dim_;
    size_t sub_dim_2_;

    IndexT up_shift_;

    size_t grid_occ_size_;
    size_t data_size_;

    double occupancy_;
};

} // namespace tsdf_localization

#include <tsdf_localization/map/sub_voxel_map.tcc>

#endif