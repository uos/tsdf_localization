#include <tsdf_localization/map/grid_map.h>

#include <exception>

namespace tsdf_localization
{

template <typename DataT, typename IndexT>
HashGridMap<DataT, IndexT>::HashGridMap(IndexT min_x, IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value) : 
Base(min_x, min_y, min_z, max_x, max_y, max_z, resolution, init_value)
{
    //data_.resize(Base::dim_x_ * Base::dim_y_ * Base::dim_z_, init_value);
}

template <typename DataT, typename IndexT>
size_t HashGridMap<DataT, IndexT>::getIndex(IndexT x, IndexT y, IndexT z) const
{
    size_t grid_x = (x - Base::min_x_) / Base::resolution_;
    size_t grid_y = (y - Base::min_y_) / Base::resolution_;
    size_t grid_z = (z - Base::min_z_) / Base::resolution_;

    if (grid_x >= Base::dim_x_ || grid_y >= Base::dim_y_ || grid_z >= Base::dim_z_)
    {
        throw std::invalid_argument("Coordinates out of bounds!");
    }

    return grid_x + grid_y * Base::dim_x_ + grid_z * Base::dim_x_ * Base::dim_y_;
}

template <typename DataT, typename IndexT>
DataT HashGridMap<DataT, IndexT>::getEntry(IndexT x, IndexT y, IndexT z) const
{
    try
    {
        auto find = data_.find(getIndex(x, y, z));

        if (find == data_.end())
        {
            return Base::init_value_;
        }

        return (*find).second;
    }
    catch(...)
    {
        return Base::init_value_;
    }
}

template <typename DataT, typename IndexT>
void HashGridMap<DataT, IndexT>::setEntry(IndexT x, IndexT y, IndexT z, DataT value)
{
    data_[getIndex(x, y, z)] = value;
}

template <typename DataT, typename IndexT>
void HashGridMap<DataT, IndexT>::setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data)
{
    for (const auto& cell : data)
    {
        setEntry(std::get<0>(cell), std::get<1>(cell), std::get<2>(cell), std::get<3>(cell));
    }
}

}