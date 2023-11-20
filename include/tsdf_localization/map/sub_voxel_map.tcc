#include <tsdf_localization/map/grid_map.h>
#include <type_traits>

namespace tsdf_localization
{

template <typename DataT, typename IndexT, int sub_voxel_size>
SubVoxelMap<DataT, IndexT, sub_voxel_size>::SubVoxelMap(IndexT min_x , IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value) :
Base(min_x, min_y, min_z, max_x, max_y, max_z, resolution, init_value), 
grid_occ_(nullptr),
data_(nullptr),
up_dim_x_(std::ceil(static_cast<FLOAT_T>(std::abs(max_x - min_x)) / sub_voxel_size)),
up_dim_y_(std::ceil(static_cast<FLOAT_T>(std::abs(max_y - min_y)) / sub_voxel_size)),
up_dim_z_(std::ceil(static_cast<FLOAT_T>(std::abs(max_z - min_z)) / sub_voxel_size)),
up_dim_2_(this->up_dim_x_ * this->up_dim_y_),
sub_dim_(std::ceil(static_cast<FLOAT_T>(sub_voxel_size) / resolution)),
sub_dim_2_(sub_dim_ * sub_dim_),
grid_occ_size_(up_dim_x_ * up_dim_y_ * up_dim_z_),
data_size_(0)
{
    auto log_2 = std::log2(sub_voxel_size);

    #ifndef  __CUDACC__
        if constexpr (std::is_integral_v<IndexT>) 
        {
            if ((log_2 - static_cast<int>(log_2)) != 0.0)
            {
                throw std::runtime_error("sub_voxel_size must be a pow of base 2!");
            }
        }
    #endif

    up_shift_ = log_2;

    //this->grid_occ_.resize(up_dim_x_ * up_dim_y_ * up_dim_z_, -1);

    grid_occ_ = new long[grid_occ_size_];

    for (auto index = 0; index < grid_occ_size_; ++index)
    {
        grid_occ_[index] = -1;
    }

    //data_.resize(Base::dim_x_ * Base::dim_y_ * Base::dim_z_, init_value);
}

template <typename DataT, typename IndexT, int sub_voxel_size>
size_t SubVoxelMap<DataT, IndexT, sub_voxel_size>::getIndex(IndexT x, IndexT y, IndexT z) const
{
    // size_t grid_x = (x - Base::min_x_) / Base::resolution_;
    // size_t grid_y = (y - Base::min_y_) / Base::resolution_;
    // size_t grid_z = (z - Base::min_z_) / Base::resolution_;

    size_t grid_x = x - Base::min_x_;
    size_t grid_y = y - Base::min_y_;
    size_t grid_z = z - Base::min_z_;

    #ifndef  __CUDACC__
    if constexpr (std::is_integral_v<IndexT>)
    {
        grid_x >>= MAP_SHIFT;
        grid_y >>= MAP_SHIFT;
        grid_z >>= MAP_SHIFT; 
    }
    else
    #endif
    {
        grid_x /= Base::resolution_;
        grid_y /= Base::resolution_;
        grid_z /= Base::resolution_;
    }

    if (grid_x >= Base::dim_x_ || grid_y >= Base::dim_y_ || grid_z >= Base::dim_z_)
    {
        return data_size_;

        //throw std::invalid_argument("Coordinates out of bounds!");
    }

    auto x_offset = x - Base::min_x_;
    auto y_offset = y - Base::min_y_;
    auto z_offset = z - Base::min_z_;

    // size_t up_x = (x_offset / sub_voxel_size);
    // size_t up_y = (y_offset / sub_voxel_size);
    // size_t up_z = (z_offset / sub_voxel_size);

    size_t up_x, up_y, up_z;

    #ifndef  __CUDACC__
    if constexpr (std::is_integral_v<IndexT>) 
    {
        up_x = x_offset >> up_shift_;
        up_y = y_offset >> up_shift_;
        up_z = z_offset >> up_shift_;
    }
    else
    #endif
    {
        up_x = (x_offset / sub_voxel_size);
        up_y = (y_offset / sub_voxel_size);
        up_z = (z_offset / sub_voxel_size);
    }

    // auto sub_pos_x = x_offset - up_x * sub_voxel_size;
    // auto sub_pos_y = y_offset - up_y * sub_voxel_size;
    // auto sub_pos_z = z_offset - up_z * sub_voxel_size;

    IndexT sub_pos_x, sub_pos_y, sub_pos_z;

    #ifndef  __CUDACC__
    if constexpr (std::is_integral_v<IndexT>) 
    {
        sub_pos_x = x_offset - (up_x << up_shift_);
        sub_pos_y = y_offset - (up_y << up_shift_);
        sub_pos_z = z_offset - (up_z << up_shift_);
    }
    else
    #endif
    {
        sub_pos_x = x_offset - up_x * sub_voxel_size;
        sub_pos_y = y_offset - up_y * sub_voxel_size;
        sub_pos_z = z_offset - up_z * sub_voxel_size;
    }

    size_t up_index = up_x + up_y * this->up_dim_x_ + up_z * up_dim_2_;

    if (up_index >= grid_occ_size_)
    {
        #ifndef __CUDACC__
            throw std::runtime_error("Upper voxel index overflow!");
        #else
            return data_size_;
        #endif
    }

    auto sub_index = grid_occ_[up_index];

    if (sub_index < 0)
    {
        return data_size_;
        //throw std::invalid_argument("Upper voxel entry does not exist!");
    }

    // size_t sub_x = sub_pos_x / Base::resolution_;
    // size_t sub_y = sub_pos_y / Base::resolution_;
    // size_t sub_z = sub_pos_z / Base::resolution_;

    size_t sub_x, sub_y, sub_z;

    #ifndef  __CUDACC__
    if constexpr (std::is_integral_v<IndexT>)
    {
        sub_x = sub_pos_x >> MAP_SHIFT;
        sub_y = sub_pos_y >> MAP_SHIFT;
        sub_z = sub_pos_z >> MAP_SHIFT; 
    }    
    else
    #endif
    {
        sub_x = sub_pos_x / Base::resolution_;
        sub_y = sub_pos_y / Base::resolution_;
        sub_z = sub_pos_z / Base::resolution_;
    }

    return sub_index + sub_x + sub_y * this->sub_dim_ + sub_z * this->sub_dim_2_ ;
}

template <typename DataT, typename IndexT, int sub_voxel_size>
DataT SubVoxelMap<DataT, IndexT, sub_voxel_size>::getEntry(IndexT x, IndexT y, IndexT z) const
{
    if (this->data_ == nullptr || this->grid_occ_ == nullptr)
    {
        return this->init_value_;
    }

    auto index = this->getIndex(x, y, z);

    if (index < data_size_)
    {
        return this->data_[index];
    }
    else
    {
        return this->init_value_;
    }
}

template <typename DataT, typename IndexT, int sub_voxel_size>
void SubVoxelMap<DataT, IndexT, sub_voxel_size>::setEntry(IndexT x, IndexT y, IndexT z, DataT value)
{
    if (this->data_ == nullptr || this->grid_occ_ == nullptr)
    {
        return;
    }

    this->data_[this->getIndex(x, y, z)] = value;
}

template <typename DataT, typename IndexT, int sub_voxel_size>
void SubVoxelMap<DataT, IndexT, sub_voxel_size>::setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data)
{
    if (data.size() == 0)
    {
        return;
    }

    for (const auto& cell : data)
    {
        auto x_offset = std::get<0>(cell) - Base::min_x_;
        auto y_offset = std::get<1>(cell) - Base::min_y_;
        auto z_offset = std::get<2>(cell) - Base::min_z_;

        size_t up_x = (x_offset / sub_voxel_size);
        size_t up_y = (y_offset / sub_voxel_size);
        size_t up_z = (z_offset / sub_voxel_size);

        auto up_index = up_x + up_y * this->up_dim_x_ + up_z * this->up_dim_x_ * this->up_dim_y_;

        if (up_index >= grid_occ_size_)
        {
            throw std::runtime_error("Upper voxel index overflow!");
        }

        grid_occ_[up_index] = 0;
    }

    long current_offset = 0;

    auto sub_size = this->sub_dim_ * this->sub_dim_ * this->sub_dim_;

    //for (auto& up_cell : grid_occ_)
    for (auto occ_index = 0u; occ_index < grid_occ_size_; ++occ_index)
    {
        auto& up_cell = grid_occ_[occ_index];

        if (up_cell >= 0)
        {
            up_cell = current_offset;
            current_offset += sub_size;
        }
    }

    //data_.resize(current_offset, this->init_value_);

    if (data_ != nullptr)
    {
        delete[] data_;
    }

    occupancy_ =  static_cast<double>(current_offset / sub_size) / grid_occ_size_;

    data_size_ = current_offset;
    data_ = new DataT[data_size_];

    for (auto index = 0; index < data_size_; ++index)
    {
        data_[index] = this->init_value_;
    }

    for (const auto& cell : data)
    {
        setEntry(std::get<0>(cell), std::get<1>(cell), std::get<2>(cell), std::get<3>(cell));
    }
}

} // namespace tsdf_localization