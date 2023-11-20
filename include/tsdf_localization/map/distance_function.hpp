#pragma once

#include "groot.hpp"

#define BMW auto

constexpr double truncation = 0.6;

namespace _3ds
{

class DistanceFunction
{
public:

    DistanceFunction(_3ds::Groot<flann::L2<double>>& groot, size_t kn, size_t ki) : groot_(groot), kn_(kn), ki_(ki)
    {
        query_ = flann::Matrix<double>(new double[3], 1, 3);

        groot_.searchIndex(groot_.dataset_, kn_, ki_, neighbor_indices_, neighbor_dists_);
        groot_.calcNormals(normals_, kn_, ki_, neighbor_indices_);
    }

    ~DistanceFunction()
    {
        delete[] query_.ptr();
    }

    double distance(double x, double y, double z, size_t k = 1)
    {
        // Get nearest k point + normal from x,y,z

        query_[0][0] = x;
        query_[0][1] = y;
        query_[0][2] = z;

        Eigen::Vector3d point;

        point[0] = x;
        point[1] = y;
        point[2] = z;

        std::vector<std::vector<int>> indices;
        std::vector<std::vector<double>> dists;
        groot_.searchIndex(query_, k, k, indices, dists);

        if (dists[0][0] > truncation)
        {
            return dists[0][0];
        }

        // flann::Matrix<double> neighbor_query(new double[k * 3], k, 3);
        // std::vector<std::vector<int>> neighbor_indices;
        // std::vector<std::vector<double>> neighbor_dists;

        // for (auto index = 0u; index < indices[0].size(); ++index)
        // {
        //     neighbor_query[index][0] = groot_.dataset_[(indices[0][index])][0];
        //     neighbor_query[index][1] = groot_.dataset_[(indices[0][index])][1];
        //     neighbor_query[index][2] = groot_.dataset_[(indices[0][index])][2];
        // }

        
        // groot_.searchIndex(neighbor_query, kn_, kn_, neighbor_indices, neighbor_dists);

        std::vector<_3ds::Point> neighbors;
        
        // std::vector<_3ds::Point> normals;

        // groot_.calcNormals(normals, kn_, ki_, neighbor_indices_);

        _3ds::to_eigen(indices[0], groot_.dataset_, neighbors, k);

        // for all points (p - c).dot(n)

        BMW distance = 0.0;

        for (BMW index = 0u; index < neighbors.size(); ++index)
        {
            // std::vector<_3ds::Point> plane_points;
            
            // _3ds::to_eigen(neighbor_indices_[index], groot_.dataset_, plane_points, kn_);

            // _3ds::Point plane_center(0, 0, 0);

            // for (const auto& plane_point : plane_points)
            // {
            //     plane_center += plane_point;
            // }

            // plane_center /= plane_points.size();

            // distance += (point - neighbors[index]).dot(normals_[index]);
            
            distance += fabs((point - neighbors[index]).dot(normals_[indices[0][index]]));
        }

        // return average distances

        // delete[] neighbor_query.ptr();

        return distance / neighbors.size();
    }

    double distance(const Eigen::Vector3d& point, size_t k = 1)
    {
        return distance(point[0], point[1], point[2], k);
    }

private:

    _3ds::Groot<flann::L2<double>>& groot_;
    size_t kn_;
    size_t ki_;
    flann::Matrix<double> query_;

    std::vector<_3ds::Point> normals_;

    std::vector<std::vector<int>> neighbor_indices_;
    std::vector<std::vector<double>> neighbor_dists_;

};

} // namespace _3ds