/**
 * @file num_particles_eval.cpp
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Evaluate the runtime of the implemented evaluation model based an sensor snapshot depending on the number of particles
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#include <iostream>
#include <fstream>


#include <tsdf_localization/evaluation/tsdf_evaluator.h>
#include <tsdf_localization/util/mcl_file.h>
#include <tsdf_localization/map/map_util.h>

#include <tsdf_localization/cuda/cuda_sub_voxel_map.h>

#include <tsdf_localization/util/runtime_evaluator.h>

using namespace tsdf_localization;

std::shared_ptr<ros::NodeHandle> nh_p_;

bool use_cuda = true;
bool per_point = false;

FLOAT_T sigma_trans = 0.5;
FLOAT_T sigma_rot = 1.5;

int repeat = 10;
int inc = 1000;
int num_particles = 100000;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "num_particles_eval");

    if (argc != 3)
    {
        std::cout << "usage: " << argv[0] << " <mcl-file> <map-file>" << std::endl;
        return 0;
    }

    static auto& eval = RuntimeEvaluator::get_instance();

    ros::NodeHandle n;
    nh_p_.reset(new ros::NodeHandle("~"));

    nh_p_->getParam("num_particles", num_particles);
    nh_p_->getParam("inc", inc);
    nh_p_->getParam("repeat", repeat);
    nh_p_->getParam("sigma_trans", sigma_trans);
    nh_p_->getParam("sigma_rot", sigma_rot);
    nh_p_->getParam("use_cuda", use_cuda);
    nh_p_->getParam("per_point", per_point);


    std::stringstream param_ss;

    param_ss << "Used parameters:\n"
             << "\tnum_particles: " << num_particles << std::endl
             << "\tinc: " << inc << std::endl
             << "\trepeat: " << repeat << std::endl
             << "\tsigma_trans: " << sigma_trans << std::endl
             << "\tsigma_rot: " << sigma_rot << std::endl
             << "\tuse_cuda: " << use_cuda << std::endl;

    std::cout << param_ss.str() << std::endl;

    std::string mcl_name(argv[1]);
    std::string map_name(argv[2]);
    
    // if (!boost::filesystem::exists(mcl_name))
    // {
    //     std::cout << "mcl file \"" << mcl_name << "\" does not exist" << std::endl;
    //     return 1;
    // }

    // if (!boost::filesystem::exists(map_name))
    // {
    //     std::cout << "map file \"" << map_name << "\" does not exist" << std::endl;
    //     return 1;
    // }

    std::cout << "Read mcl data from file..." << std::endl;

    MCLFile file(mcl_name);

    std::vector<CudaPoint> points;
    std::vector<int> rings;
    std::vector<Particle> particles;
    std::array<FLOAT_T, 16> tf_matrix;
   
    FLOAT_T x, y, z, q_1, q_2, q_3, q_4;

    file.read(points, rings, particles, tf_matrix, x, y, z, q_1, q_2, q_3, q_4);

   /* std::multimap<int, CudaPoint> point_map;

    for (auto index = 0u; index < points.size(); ++index)
    {
        point_map.insert(std::pair<int, CudaPoint>(rings[index], {points[index].x, points[index].y, points[index].z}));
    }

    std::vector<CudaPoint> ordered_points;
    ordered_points.reserve(points.size());


    for (const auto& entry : point_map)
    {
        ordered_points.push_back(entry.second);
    }

    std::unordered_set<CudaPoint, hash> point_set;

    for (const auto& point : ordered_points)
    {
        CudaPoint center = {static_cast<float>(std::floor(point.x / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(point.y / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(point.z / 0.064) * 0.064 + 0.032)};
        
        point_set.insert(center);
    }

    std::vector<CudaPoint> reduced_points;
    reduced_points.resize(point_set.size());
    std::copy(point_set.begin(), point_set.end(), reduced_points.begin());*/

    std::unordered_set<SortClass, hash> point_set;

    //int index = 0;

    for (auto index = 0u; index < points.size(); ++index)
    {
        CudaPoint center = {static_cast<float>(std::floor(points[index].x / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(points[index].y / 0.064) * 0.064 + 0.032), 
                            static_cast<float>(std::floor(points[index].z / 0.064) * 0.064 + 0.032)};
        
        //point_set.insert(std::make_pair(iter_ring[0], center));
        point_set.insert(SortClass(rings[index], index, center));

        //++index;
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
    
    TSDFEvaluator evaluator(map, per_point);

    std::cout << "Execute kernel..." << std::endl;

    ParticleCloud cloud;

    geometry_msgs::Pose center_pose;
    center_pose.position.x = x;
    center_pose.position.y = y;
    center_pose.position.z = z;

    center_pose.orientation.w = q_1;
    center_pose.orientation.x = q_2;
    center_pose.orientation.y = q_3;
    center_pose.orientation.z = q_4;

    auto start = std::chrono::high_resolution_clock::now();
    auto stop = start;

    long measure = 0;

    std::vector<long> numbers_of_particles;
    std::vector<long> measurements;

    std::cout << "Warmup..." << std::endl;

    cloud.initialize(center_pose, num_particles, sigma_trans, sigma_trans, 0.1, sigma_rot, sigma_rot, sigma_rot);
    evaluator.evaluate(cloud.particles(), ordered_points, tf_matrix.data(), use_cuda);

    std::cout << "Evaluation..." << std::endl;

    for (auto step = inc; step <= num_particles; step += inc)
    {
        std::cout << "\r\tnum particles: " << step << " / " << num_particles;
        std::cout.flush();

        cloud.initialize(center_pose, step, sigma_trans, sigma_trans, 0.1, sigma_rot, sigma_rot, sigma_rot);
        
        measure = 0;

        for (auto index = 0; index < repeat; ++index)
        {
            start = std::chrono::high_resolution_clock::now();
            evaluator.evaluate(cloud.particles(), ordered_points, tf_matrix.data(), use_cuda);
            stop = std::chrono::high_resolution_clock::now();
            
            measure += std::chrono::duration_cast<measurement_unit>(stop - start).count();
        }

        numbers_of_particles.push_back(step);
        measurements.push_back(measure / repeat / 1000);
    }

    std::cout << std::endl;

    std::ostringstream filename;
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);  

    filename << "mcl_num_particles_times_" << std::put_time(std::localtime(&t), "%Y-%m-%d-%H-%M-%S") << ".log";

    std::ofstream time_output(filename.str());
    
    std::stringstream eval_string;

    //time_output << RuntimeEvaluator::get_instance().to_string(true);
    
    eval_string << "| num. particles | runtime [ms] |\n";

    for (auto index = 0; index < measurements.size(); ++index)
    {
        eval_string << numbers_of_particles[index] << " " << measurements[index] << "\n";
    }

    std::cout << eval_string.str();

    time_output << param_ss.str() << "\n" << eval_string.str();

    time_output.close();

    std::cout << "\nEvaluation finished!" << std::endl;
    return 0;
}