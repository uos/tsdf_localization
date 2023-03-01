#ifndef MAP_UTIL_H
#define MAP_UTIL_H

#include <highfive/H5File.hpp>
#include <map/grid_map.h>
#include <util/tsdf.h>
#include <int_mcl/int_constant.h>

#include <util/util.h> 
#include <evaluation/model/likelihood_evaluation.h>

#include <cuda/cuda_evaluator.h>

namespace mcl
{

template <typename MapT, typename IndexT, typename DataT>
std::shared_ptr<MapT> createTSDFMap(std::string h5_filename, std::vector<CudaPoint>& free_map, FLOAT_T sigma = 0.1)
{
    HighFive::File f(h5_filename, HighFive::File::ReadOnly); // TODO: Path and name as command line input
    HighFive::Group g = f.getGroup("/map");

    std::vector<FLOAT_T> min(3, 0); 
    std::vector<FLOAT_T> max(3, 0);

    // Determine the boundingbox of the complete map
    for (auto tag : g.listObjectNames())
    {
        std::vector<int> chunk_pos;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;

        while ((pos = tag.find(delimiter)) != std::string::npos)
        {
            token = tag.substr(0, pos);
            chunk_pos.push_back(std::stoi(token));
            tag.erase(0, pos + delimiter.length());
        }

        chunk_pos.push_back(std::stoi(tag));

        int index = 0;

        for (const auto& coord : chunk_pos)
        {   
            if (coord < min[index])
            {
                min[index] = coord;
            }

            if (coord > max[index])
            {
                max[index] = coord;
            }

            ++index;
        }

    }

    for (auto index = 0u; index < min.size(); ++index)
    {
        min[index] = min[index] * CHUNK_SIZE * MAP_RESOLUTION * 0.001;
        max[index] = (max[index] * CHUNK_SIZE + CHUNK_SIZE) * MAP_RESOLUTION * 0.001;
    }

    auto sigma_quad = sigma * sigma;

    auto init = expf(-(10.0 * 10.0) / sigma_quad / 2) / (sqrtf(2 * sigma_quad * M_PI));
    init = init * init * init;

    auto map = std::make_shared<MapT>(MapT(min[0], min[1], min[2], max[0], max[1], max[2], MAP_RESOLUTION * 0.001, init));

    std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>> data;
    data.reserve(10000);

    free_map.reserve(10000);

    // Fill the grid with the valid TSDF values of the map
    for (auto tag : g.listObjectNames())
    {
        // Get the chunk data
        HighFive::DataSet d = g.getDataSet(tag);
        std::vector<TSDFValue::RawType> chunk_data;
        d.read(chunk_data);
        // Get the chunk position
        std::vector<int> chunk_pos;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;
        while ((pos = tag.find(delimiter)) != std::string::npos)
        {
            token = tag.substr(0, pos);
            chunk_pos.push_back(std::stoi(token));
            tag.erase(0, pos + delimiter.length());
        }
        chunk_pos.push_back(std::stoi(tag));

        for (int i = 0; i < CHUNK_SIZE; i++)
        {
            for (int j = 0; j < CHUNK_SIZE; j++)
            {
                for (int k = 0; k < CHUNK_SIZE; k++)
                {
                    auto entry = TSDFValue(chunk_data[CHUNK_SIZE * CHUNK_SIZE * i + CHUNK_SIZE * j + k]);

                    auto tsdf_value = (float)(entry.value());
                    auto weight = entry.weight();

                    int x = CHUNK_SIZE * chunk_pos[0] + i;
                    int y = CHUNK_SIZE * chunk_pos[1] + j;
                    int z = CHUNK_SIZE * chunk_pos[2] + k;

                    // Only touched cells are considered
                    if (weight != 0 && std::abs(tsdf_value) < truncation)
                    {
                        auto pos_x = static_cast<FLOAT_T>(x) * MAP_RESOLUTION * 0.001;
                        auto pos_y = static_cast<FLOAT_T>(y) * MAP_RESOLUTION * 0.001;
                        auto pos_z = static_cast<FLOAT_T>(z) * MAP_RESOLUTION * 0.001;

                        //map->setEntry(pos_x, pos_y, pos_z, tsdf_value * 0.001);

                        auto value = tsdf_value * 0.001;
                        value = expf(-(value * value) / sigma_quad / 2) / (sqrtf(2 * sigma_quad * M_PI));
                        value = value * value * value;

                        data.push_back(std::make_tuple(pos_x, pos_y, pos_z, value));
                    }

                    if (weight != 0 && std::abs(tsdf_value) >= truncation)
                    {
                        auto pos_x = static_cast<FLOAT_T>(x) * MAP_RESOLUTION * 0.001;
                        auto pos_y = static_cast<FLOAT_T>(y) * MAP_RESOLUTION * 0.001;
                        auto pos_z = static_cast<FLOAT_T>(z) * MAP_RESOLUTION * 0.001;

                        //map->setEntry(pos_x, pos_y, pos_z, tsdf_value * 0.001);
                        
                        CudaPoint free_point;
                        free_point.x = pos_x;
                        free_point.y = pos_y;
                        free_point.z = pos_z;

                        free_map.push_back(free_point);
                    }
                }
            }
        }
    }

    map->setData(data);

    return map;
}

template <typename MapT, typename IndexT, typename DataT>
std::shared_ptr<MapT> createIntTSDFMap(std::string h5_filename)
{
    HighFive::File f(h5_filename, HighFive::File::ReadOnly); // TODO: Path and name as command line input
    HighFive::Group g = f.getGroup("/map");

    std::vector<int_mcl::FIXED_T> min(3, 0); 
    std::vector<int_mcl::FIXED_T> max(3, 0);

    // Determine the boundingbox of the complete map
    for (auto tag : g.listObjectNames())
    {
        std::vector<int> chunk_pos;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;

        while ((pos = tag.find(delimiter)) != std::string::npos)
        {
            token = tag.substr(0, pos);
            chunk_pos.push_back(std::stoi(token));
            tag.erase(0, pos + delimiter.length());
        }

        chunk_pos.push_back(std::stoi(tag));

        int index = 0;

        for (const auto& coord : chunk_pos)
        {   
            if (coord < min[index])
            {
                min[index] = coord;
            }

            if (coord > max[index])
            {
                max[index] = coord;
            }

            ++index;
        }

    }

    for (auto index = 0u; index < min.size(); ++index)
    {
        min[index] = min[index] * CHUNK_SIZE * MAP_RESOLUTION * (int_mcl::INT_RESOLUTION / 1000);
        max[index] = (max[index] * CHUNK_SIZE + CHUNK_SIZE) * MAP_RESOLUTION * (int_mcl::INT_RESOLUTION / 1000);
    }

    auto map = std::make_shared<MapT>(MapT(min[0], min[1], min[2], max[0], max[1], max[2], MAP_RESOLUTION * (int_mcl::INT_RESOLUTION / 1000), -1000));

    std::vector<std::tuple<IndexT, IndexT, IndexT, DataT>> data;
    data.reserve(10000);

    auto count = 0;

    // Fill the grid with the valid TSDF values of the map
    for (auto tag : g.listObjectNames())
    {
        // Get the chunk data
        HighFive::DataSet d = g.getDataSet(tag);
        std::vector<TSDFValue::RawType> chunk_data;
        d.read(chunk_data);
        // Get the chunk position
        std::vector<int> chunk_pos;
        std::string delimiter = "_";
        size_t pos = 0;
        std::string token;
        while ((pos = tag.find(delimiter)) != std::string::npos)
        {
            token = tag.substr(0, pos);
            chunk_pos.push_back(std::stoi(token));
            tag.erase(0, pos + delimiter.length());
        }
        chunk_pos.push_back(std::stoi(tag));

        for (int i = 0; i < CHUNK_SIZE; i++)
        {
            for (int j = 0; j < CHUNK_SIZE; j++)
            {
                for (int k = 0; k < CHUNK_SIZE; k++)
                {
                    auto entry = TSDFValue(chunk_data[CHUNK_SIZE * CHUNK_SIZE * i + CHUNK_SIZE * j + k]);

                    auto tsdf_value = (int_mcl::FIXED_T)(entry.value());
                    auto weight = entry.weight();

                    int x = CHUNK_SIZE * chunk_pos[0] + i;
                    int y = CHUNK_SIZE * chunk_pos[1] + j;
                    int z = CHUNK_SIZE * chunk_pos[2] + k;

                    // Only touched cells are considered
                    if (weight != 0 && std::abs(tsdf_value) < truncation)
                    {
                        auto pos_x = static_cast<int_mcl::FIXED_T>(x) * MAP_RESOLUTION * (int_mcl::INT_RESOLUTION / 1000);
                        auto pos_y = static_cast<int_mcl::FIXED_T>(y) * MAP_RESOLUTION * (int_mcl::INT_RESOLUTION / 1000);
                        auto pos_z = static_cast<int_mcl::FIXED_T>(z) * MAP_RESOLUTION * (int_mcl::INT_RESOLUTION / 1000);

                        //map->setEntry(pos_x, pos_y, pos_z, tsdf_value * 0.001);

                        data.push_back(std::make_tuple(pos_x, pos_y, pos_z, tsdf_value * (int_mcl::INT_RESOLUTION / 1000)));
                    
                        ++count;
                    }
                }
            }
        }
    }

    map->setData(data);

    return map;
}

} // namespace mcl

#endif