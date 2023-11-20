#ifndef MCL_FILE_H
#define MCL_FILE_H

#include <string>
#include <vector>
#include <array>

#include <tsdf_localization/util/util.h>
#include <tsdf_localization/cuda/cuda_evaluator.h>
#include <tsdf_localization/particle_cloud.h>

#ifndef __CUDACC__

#include <tsdf_localization/int_mcl/int_constant.h>

#endif

namespace tsdf_localization
{

class MCLFile
{
public:
    MCLFile(const std::string& file_name);

    void write(const std::vector<CudaPoint>& points, const std::vector<int>& rings, const std::vector<Particle>& particles, const std::array<FLOAT_T, 16>& tf, FLOAT_T x, FLOAT_T y, FLOAT_T z, FLOAT_T q_1, FLOAT_T q_2, FLOAT_T q_3, FLOAT_T q_4) const;

    void read(std::vector<CudaPoint>& points, std::vector<int>& rings, std::vector<Particle>& particles, std::array<FLOAT_T, 16>& tf, FLOAT_T& x, FLOAT_T& y, FLOAT_T& z, FLOAT_T& q_1, FLOAT_T& q_2, FLOAT_T& q_3, FLOAT_T& q_4) const;

private:
    std::string name_;
};

} // namespace tsdf_localization

#endif