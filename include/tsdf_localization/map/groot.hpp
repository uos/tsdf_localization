#pragma once

//#include "parse.hpp"

#include <sys/types.h>
#include <flann/flann.hpp>
#include <boost/filesystem.hpp>

#include <sensor_msgs/PointCloud.h>

// Eigen deps
#include <Eigen/Dense>

#include <string>
#include <fstream>

namespace fs = boost::filesystem;

namespace _3ds 
{

using CoordT = double;

using Point = Eigen::Matrix<CoordT, 3, 1>;

void print_points_by_indices(const std::vector<std::vector<int>>& indices, const flann::Matrix<CoordT>& dataset)
{
    for (const auto& index_list : indices)
    {
        for (const auto& index : index_list)
        {
            std::cout << dataset[index][0] << std::endl;
            std::cout << dataset[index][1] << std::endl;
            std::cout << dataset[index][2] << std::endl << std::endl;
        }
    }
}

void to_eigen(const std::vector<int>& indices, const flann::Matrix<double>& dataset, std::vector<Point>& neighbors, size_t number_neighbors)
{
    neighbors.resize(indices.size());

    auto bound = (number_neighbors <= indices.size() ? number_neighbors : indices.size());

    for (auto step = 0u; step < bound; ++step)
    {
        neighbors[step][0] = dataset[indices[step]][0];
        neighbors[step][1] = dataset[indices[step]][1];
        neighbors[step][2] = dataset[indices[step]][2];
    }

}

int parse_points(const std::string& filename, std::vector<geometry_msgs::Point32>& points)
{
    std::ifstream stream(filename);
    std::string line;

    do
    {
        std::getline(stream, line);
    } while (line.find("element vertex") == std::string::npos);

    auto last_of = line.find_last_of(' ') + 1;
    auto number = std::stoi(line.substr(last_of, line.size() - last_of));

    do
    {
        std::getline(stream, line);
    } while (line.find("end") == std::string::npos);

    float x, y, z;

    int count = 0;

    points.resize(number);

    while (!stream.eof() && count < number)
    {
        if (!stream.read((char*)&x, sizeof(x)) || !stream.read((char*)&y, sizeof(y)) || !stream.read((char*)&z, sizeof(z)))
        {
            throw std::ifstream::failure("Error: Cannot read point from file");
        }

        points[count].x = x;
        points[count].y = y;
        points[count].z = z;

        ++count;
    }

    return number;
}

int parse_points(const std::string& filename, sensor_msgs::PointCloud& cloud)
{
    return parse_points(filename, cloud.points);
}

template <typename T>
int parse_points(const std::string& filename, flann::Matrix<T>& points)
{
    std::vector<geometry_msgs::Point32> ros_points;
    auto number = parse_points(filename, ros_points);

    points = flann::Matrix<double>(new double[number * 3], number, 3);

    for (auto index = 0u; index < number; ++index)
    {   
        points[index][0] = ros_points[index].x;
        points[index][1] = ros_points[index].y;
        points[index][2] = ros_points[index].z;
    }

    return number;
}

template <typename PointT>
struct Groot
{
    explicit Groot(flann::Matrix<double>& dataset, const Point& orientation_point, int checks = flann::FLANN_CHECKS_UNLIMITED);
    explicit Groot(flann::Matrix<double>& dataset, int npoints, const Point& orientation_point, int checks = flann::FLANN_CHECKS_UNLIMITED);
    Groot(const Groot& o) = delete;
    Groot(Groot&& o) = delete;
    Groot operator=(const Groot& o) = delete;
    Groot operator=(Groot&& o) = delete;
    ~Groot()
    {
        if (index_)
        {
            delete index_;
        }

        delete[] dataset_.ptr();
    }

    void initDataset(std::string filename);
    void initDataset(const std::vector<geometry_msgs::Point32>& ros_points);
    void buildIndex();
    void searchIndex(flann::Matrix<double>& dataset, size_t kn, size_t ki, std::vector<std::vector<int>>& indices, std::vector<std::vector<double>>& dists);
    void calcNormals(std::vector<Point>& normals, size_t kn, size_t ki, const std::vector<std::vector<int>>& indices);
    void save(std::string filename, const std::vector<Point>& normals);
    flann::Index<PointT>* index_;
    flann::Matrix<double>& dataset_;
    int npoints_;
    const Point& orientation_point_;
    int checks_;
};

template<typename PointT>
void Groot<PointT>::initDataset(std::string filename)
{
    npoints_ = parse_points(filename, dataset_);
    if (npoints_ == -1)
    {
        throw std::runtime_error("Couldn't parse dataset");
    }
}

template<typename PointT>
void Groot<PointT>::initDataset(const std::vector<geometry_msgs::Point32>& ros_points)
{
    npoints_ = ros_points.size();
    dataset_ = flann::Matrix<double>(new double[npoints_ * 3], npoints_, 3);

    for (auto index = 0u; index < npoints_; ++index)
    {   
        dataset_[index][0] = ros_points[index].x;
        dataset_[index][1] = ros_points[index].y;
        dataset_[index][2] = ros_points[index].z;
    }
}

template<typename PointT>
Groot<PointT>::Groot(flann::Matrix<double>& dataset, int npoints, const Point& orientation_point, int checks)
: index_(nullptr), dataset_(dataset), npoints_(npoints), orientation_point_(orientation_point), checks_(checks)
{

}


template<typename PointT>
Groot<PointT>::Groot(flann::Matrix<double>& dataset, const Point& orientation_point, int checks)
: Groot(dataset, 0, orientation_point, checks)
{

}

template<typename PointT>
void Groot<PointT>::buildIndex()
{
    if (npoints_ == 0)
    {
        throw std::runtime_error("You haven't initialized dataset yet, bitch");
    }

    index_ = new flann::Index<flann::L2<double>>(dataset_,flann::KDTreeIndexParams(8));
    index_->buildIndex();
}

template<typename PointT>
void Groot<PointT>::calcNormals(std::vector<Point>& normals, size_t kn, size_t ki, const std::vector<std::vector<int>>& indices)
{
    if (indices.empty())
    {
        throw std::runtime_error("indices not calculated! Forget the searchKNN step?");
    }
    
    normals.resize(indices.size());
    std::vector<Point> neighbors;

    for (auto step = 0u; step < indices.size(); ++step)
    {
        const auto& index_list = indices[step];
        auto& normal = normals[step];

        to_eigen(index_list, dataset_, neighbors, kn);

        Point center = Point::Zero();

        for (const auto& neighbor : neighbors)
        {
            center += neighbor;
        }

        center /= neighbors.size();

        double xx = 0;
        double yy = 0;
        double zz = 0;

        double xy = 0;
        double xz = 0;
        double yz = 0;

        for (auto& neighbor : neighbors)
        {
            neighbor -= center;

            xx += neighbor[0] * neighbor[0];
            yy += neighbor[1] * neighbor[1];
            zz += neighbor[2] * neighbor[2];

            xy += neighbor[0] * neighbor[1];
            xz += neighbor[0] * neighbor[2];

            yz += neighbor[1] * neighbor[2];
        }

        Eigen::Matrix3d Q;
        Q << xx, xy, xz,
                xy, yy, yz,
                xz, yz, zz;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(Q);

        if (eigen_solver.info() != Eigen::Success)
        {
            throw std::runtime_error("The normal of one point could not be determined!");
        }

        const auto& eigenvectors = eigen_solver.eigenvectors();

        normal << eigenvectors(0, 0), eigenvectors(1, 0), eigenvectors(2, 0);
        normal.normalize();

        if (normal.dot(orientation_point_) < 0)
        {
            normal = -normal;
        }
    }

    // for (auto step = 0u; step < indices.size(); ++step)
    // {
    //     const auto& index_list = indices[step];
    //     Point normal_average = Point::Zero();

    //     for (auto step = 0u; step < ki; ++step)
    //     {
    //         normal_average += normals[index_list[step]];
    //     }

    //     normals[step] = (normal_average / ki).normalized();

    //     if (normals[step].dot(orientation_point_) < 0)
    //     {
    //         normals[step] = -normals[step];
    //     }
    // }
}

template<typename PointT>
void Groot<PointT>::searchIndex(flann::Matrix<double> &dataset, size_t kn, size_t ki,
                                std::vector<std::vector<int>> &indices, std::vector<std::vector<double>> &dists)
{
    if (!index_)
    {
        throw std::runtime_error("Index not initialized");
    }

    if (ki > kn)
    {
        index_->knnSearch(dataset, indices, dists, ki, flann::SearchParams(checks_));
    
        // if (indices[0].size() == 0)
        // {
        //     std::cout << dataset[0][0] << " " << dataset[0][1] << " " << dataset[0][2] << std::endl;
        // }
    }
    else
    {
        index_->knnSearch(dataset, indices, dists, kn, flann::SearchParams(checks_));
    
        // if (indices[0].size() == 0)
        // {
        //     std::cout << dataset[0][0] << " " << dataset[0][1] << " " << dataset[0][2] << std::endl;
        // }
    }
}

}