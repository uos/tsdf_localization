#ifndef NAIVGRIDMAP_H
#define NAIVGRIDMAP_H

#include <map/grid_map.h>

#include <map>

namespace mcl 
{

template <typename DataT, typename IndexT>
class HashGridMap : public GridMap<DataT, IndexT>
{
public:
    HashGridMap(IndexT min_x , IndexT min_y, IndexT min_z, IndexT max_x , IndexT max_y, IndexT max_z, IndexT resolution, DataT init_value = 0);

    virtual DataT getEntry(IndexT x, IndexT y, IndexT z) const override;
    virtual void setEntry(IndexT x, IndexT y, IndexT z, DataT value) override;

    virtual size_t size() override
    {
        return data_.size();
    }

    virtual void setData(const std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>>& data) override;

    std::unordered_map<size_t, DataT>& data()
    {
        return data_;
    }

private:
    size_t getIndex(IndexT x, IndexT y, IndexT z) const;

    using Base = GridMap<DataT, IndexT>;

    std::unordered_map<size_t, DataT> data_;
};

} // namespace mcl

#include <map/hash_grid_map.tcc>

#endif