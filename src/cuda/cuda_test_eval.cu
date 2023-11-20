#include <iostream>

//#include <boost/filesystem.hpp>

#include <tsdf_localization/cuda/cuda_evaluator.h>
#include <tsdf_localization/util/mcl_file.h>
#include <tsdf_localization/map/map_util.h>

#include <tsdf_localization/cuda/cuda_sub_voxel_map.h>

using namespace tsdf_localization;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "usage: " << argv[0] << " <mcl-file> <map-file>" << std::endl;
        return 0;
    }

    std::string mcl_name(argv[1]);
    std::string map_name(argv[2]);

    std::cout << "Read mcl data from file..." << std::endl;

    MCLFile file(mcl_name);

    std::vector<CudaPoint> points;
    std::vector<int> rings;
    std::vector<Particle> particles;
    std::array<FLOAT_T, 16> tf_matrix;
   
    FLOAT_T x, y, z, q_1, q_2, q_3, q_4;

    file.read(points, rings, particles, tf_matrix, x, y, z, q_1, q_2, q_3, q_4);

    std::unordered_set<SortClass, hash> point_set;

    for (auto index = 0u; index < points.size(); ++index)
    {
        CudaPoint center = {static_cast<float>(std::floor(points[index].x / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(points[index].y / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(points[index].z / 0.064) * 0.064 + 0.032)};
        
        point_set.insert(SortClass(rings[index], index, center));
    }

    std::vector<SortClass> reduced_points;
    reduced_points.resize(point_set.size());
    std::copy(point_set.begin(), point_set.end(), reduced_points.begin());

    std::vector<CudaPoint> ordered_points;
    ordered_points.reserve(reduced_points.size());

    std::vector<std::vector<std::pair<int, CudaPoint>>> sort_points(16);

    struct
    {
        bool operator()(std::pair<int, CudaPoint>& a, std::pair<int, CudaPoint>& b)
        {
            return a.first < b.first;
        }
    } customComp;


    for (auto& curr_ring : sort_points)
    {
        curr_ring.reserve(1500);
    }

    for (const auto& point : reduced_points)
    {
        sort_points[point.ring_].push_back(std::make_pair(point.index_, point.point_));
    }

    for (auto& curr_ring : sort_points)
    {
        std::sort(curr_ring.begin(), curr_ring.end(), customComp);
    }

    for (auto index = 0u; index < sort_points.size(); ++index)
    {
        auto& curr_ring = sort_points[index];

        for (auto& point : curr_ring)
        {
            ordered_points.push_back(point.second);
        }
    }

    std::cout << "Original cloud size: " << ordered_points.size() << std::endl;
    std::cout << "Reduced cloud size: " << reduced_points.size() << std::endl;
    std::cout << "Reduction ratio: " << static_cast<double>(reduced_points.size()) / ordered_points.size() * 100 << "%" << std::endl;

    std::cout << "Create map from file..." << std::endl;

    std::vector<CudaPoint> free_map;
    auto map = createTSDFMap<CudaSubVoxelMap<FLOAT_T, FLOAT_T>, FLOAT_T, FLOAT_T>(map_name, free_map);

    std::cout << "\nStart evaluation of the cuda kernel..." << std::endl;

    std::cout << "Generate device context..." << std::endl;
    
    CudaEvaluator evaluator(*map, false);

    std::cout << "Execute kernel..." << std::endl;

    geometry_msgs::Pose center_pose;
    center_pose.position.x = x;
    center_pose.position.y = y;
    center_pose.position.z = z;

    center_pose.orientation.w = q_1;
    center_pose.orientation.x = q_2;
    center_pose.orientation.y = q_3;
    center_pose.orientation.z = q_4;

    evaluator.evaluate(particles, ordered_points, tf_matrix.data());

    std::cout << "\nEvaluation finished!" << std::endl;

    return 0;
}