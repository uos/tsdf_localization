#ifndef CUDA_DATA_H
#define CUDA_DATA_H

namespace mcl
{

#ifdef __CUDACC__

constexpr FLOAT_T SIGMAR = 0.1f;
constexpr FLOAT_T SIGMAR_QUAD = SIGMAR * SIGMAR;

//using OCC_T = int;

__constant__ FLOAT_T const_tf_matrix[16];
//FLOAT_T const_tf_matrix[16];

CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef* g_map_coef_ = nullptr;
__constant__ CudaSubVoxelMap<FLOAT_T, FLOAT_T>::MapCoef const_map_coef_;

cudaTextureObject_t grid_occ_tex_;

cudaTextureObject_t* d_tex_grid_ = nullptr;
std::vector<cudaArray*> d_arrays_;

OCC_T* g_grid_occ_ = nullptr;
FLOAT_T* g_data_ = nullptr;

__constant__ FLOAT_T const_a_hit_;
__constant__ FLOAT_T const_a_range_;
__constant__ FLOAT_T const_a_max_;

__constant__ FLOAT_T const_max_range_;
__constant__ FLOAT_T const_max_range_squared_;
__constant__ FLOAT_T const_inv_max_range_;


#endif

} // namespace mcl

#endif