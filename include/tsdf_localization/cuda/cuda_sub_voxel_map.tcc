namespace tsdf_localization
{

template <typename DataT, typename IndexT>
CudaSubVoxelMap<DataT, IndexT>::CudaSubVoxelMap(IndexT min_x , IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value) :
grid_occ_(nullptr),
data_(nullptr),
track_reuse_(false),
new_track_(false),
track_count_(0),
last_track_(0),
last_index_(-1),
invalid_count_(0),
occ_reuse_(nullptr)
{
    map_coef.dim_x_ = std::ceil(std::abs(max_x - min_x) / resolution);
    map_coef.dim_y_ = std::ceil(std::abs(max_y - min_y) / resolution);
    map_coef.dim_z_ = std::ceil(std::abs(max_z - min_z) / resolution);
    map_coef.min_x_ = min_x;
    map_coef.min_y_ = min_y;
    map_coef.min_z_ = min_z;
    map_coef.max_x_ = max_x;
    map_coef.max_y_ = max_y;
    map_coef.max_z_ = max_z;
    map_coef.resolution_ = resolution;
    map_coef.init_value_ = init_value;
    map_coef.up_dim_x_ = std::ceil(static_cast<FLOAT_T>(std::abs(max_x - min_x)) / sub_voxel_size);
    map_coef.up_dim_y_ = std::ceil(static_cast<FLOAT_T>(std::abs(max_y - min_y)) / sub_voxel_size);
    map_coef.up_dim_z_ = std::ceil(static_cast<FLOAT_T>(std::abs(max_z - min_z)) / sub_voxel_size);
    map_coef.up_dim_2_ = map_coef.up_dim_x_ * map_coef.up_dim_y_;
    map_coef.sub_dim_ = std::ceil(static_cast<FLOAT_T>(sub_voxel_size) / resolution);
    map_coef.sub_dim_2_ = map_coef.sub_dim_ * map_coef.sub_dim_;
    map_coef.grid_occ_size_ = map_coef.up_dim_x_ * map_coef.up_dim_y_ * map_coef.up_dim_z_;
    map_coef.data_size_ = 0;

    //this->grid_occ_.resize(up_dim_x_ * up_dim_y_ * up_dim_z_, -1);

    grid_occ_ = new OCC_T[map_coef.grid_occ_size_];
    occ_reuse_ = new long[map_coef.grid_occ_size_];

    for (auto index = 0; index < map_coef.grid_occ_size_; ++index)
    {
        grid_occ_[index] = -1;
        occ_reuse_[index] = 0;
    }

    //data_.resize(dim_x_ * dim_y_ * dim_z_, init_value);
}

template <typename DataT, typename IndexT>
size_t CudaSubVoxelMap<DataT, IndexT>::getIndex(IndexT x, IndexT y, IndexT z)
{
    size_t grid_x = x - map_coef.min_x_;
    size_t grid_y = y - map_coef.min_y_;
    size_t grid_z = z - map_coef.min_z_;

    
    grid_x /= map_coef.resolution_;
    grid_y /= map_coef.resolution_;
    grid_z /= map_coef.resolution_;
    
    if (grid_x >= map_coef.dim_x_ || grid_y >= map_coef.dim_y_ || grid_z >= map_coef.dim_z_)
    {
        return map_coef.data_size_;
    }

    auto x_offset = x - map_coef.min_x_;
    auto y_offset = y - map_coef.min_y_;
    auto z_offset = z - map_coef.min_z_;

    size_t up_x, up_y, up_z;


    up_x = (x_offset / sub_voxel_size);
    up_y = (y_offset / sub_voxel_size);
    up_z = (z_offset / sub_voxel_size);
    
    IndexT sub_pos_x, sub_pos_y, sub_pos_z;

    
    
    sub_pos_x = x_offset - up_x * sub_voxel_size;
    sub_pos_y = y_offset - up_y * sub_voxel_size;
    sub_pos_z = z_offset - up_z * sub_voxel_size;
    
    size_t up_index = up_x + up_y * map_coef.up_dim_x_ + up_z * map_coef.up_dim_2_;

    if (track_reuse_)
    {
        if (last_index_ == up_index)
        {
            new_track_ = false;
            ++track_count_;
        }
        else
        {
            if (last_index_ != -1)
            {
                new_track_ = true;
                last_track_ = track_count_;
            }

            last_index_ = up_index;
            track_count_ = 1;
        }
    }

    if (up_index >= map_coef.grid_occ_size_)
    {
        return map_coef.data_size_;   
    }

    auto sub_index = grid_occ_[up_index];

    if (track_reuse_)
    {
        ++occ_reuse_[up_index];
    }

    if (sub_index < 0)
    {
        if (track_reuse_)
        {
            ++invalid_count_;
        }

        return map_coef.data_size_;
    }

    size_t sub_x, sub_y, sub_z;

    sub_x = sub_pos_x / map_coef.resolution_;
    sub_y = sub_pos_y / map_coef.resolution_;
    sub_z = sub_pos_z / map_coef.resolution_;
    
    return sub_index + sub_x + sub_y * map_coef.sub_dim_ + sub_z * map_coef.sub_dim_2_ ;
}

template <typename DataT, typename IndexT>
DataT CudaSubVoxelMap<DataT, IndexT>::getEntry(IndexT x, IndexT y, IndexT z)
{
    if (this->data_ == nullptr || this->grid_occ_ == nullptr)
    {
        return map_coef.init_value_;
    }

    auto index = this->getIndex(x, y, z);

    if (index < map_coef.data_size_)
    {
        return this->data_[index];
    }
    else
    {
        return map_coef.init_value_;
    }
}

template <typename DataT, typename IndexT>
void CudaSubVoxelMap<DataT, IndexT>::setEntry(IndexT x, IndexT y, IndexT z, DataT value)
{
    if (this->data_ == nullptr || this->grid_occ_ == nullptr)
    {
        return;
    }

    this->data_[this->getIndex(x, y, z)] = value;
}

template <typename DataT, typename IndexT>
void CudaSubVoxelMap<DataT, IndexT>::setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data)
{
    if (data.size() == 0)
    {
        return;
    }

    for (const auto& cell : data)
    {
        auto x_offset = std::get<0>(cell) - map_coef.min_x_;
        auto y_offset = std::get<1>(cell) - map_coef.min_y_;
        auto z_offset = std::get<2>(cell) - map_coef.min_z_;

        size_t up_x = (x_offset / sub_voxel_size);
        size_t up_y = (y_offset / sub_voxel_size);
        size_t up_z = (z_offset / sub_voxel_size);

        auto up_index = up_x + up_y * map_coef.up_dim_x_ + up_z * map_coef.up_dim_x_ * map_coef.up_dim_y_;

        if (up_index >= map_coef.grid_occ_size_)
        {
            throw std::runtime_error("Upper voxel index overflow!");
        }

        grid_occ_[up_index] = 0;
    }

    OCC_T current_offset = 0;

    auto sub_size = map_coef.sub_dim_ * map_coef.sub_dim_ * map_coef.sub_dim_;

    for (auto occ_index = 0u; occ_index < map_coef.grid_occ_size_; ++occ_index)
    {
        auto& up_cell = grid_occ_[occ_index];

        if (up_cell >= 0)
        {
            up_cell = current_offset;
            current_offset += sub_size;
        }
    }

    if (data_ != nullptr)
    {
        delete[] data_;
    }

    map_coef.data_size_ = current_offset;
    data_ = new DataT[map_coef.data_size_];

    for (auto index = 0; index < map_coef.data_size_; ++index)
    {
        data_[index] = map_coef.init_value_;
    }

    for (const auto& cell : data)
    {
        setEntry(std::get<0>(cell), std::get<1>(cell), std::get<2>(cell), std::get<3>(cell));
    }
}

} // namespace tsdf_localization