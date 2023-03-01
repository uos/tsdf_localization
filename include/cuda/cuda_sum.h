#ifndef CUDA_SUM_H
#define CUDA_SUM_H

#include <util/util.h>
#include <cuda/cuda_util.h>

namespace mcl
{

#ifdef __CUDACC__

__device__ void warpReduce(volatile FLOAT_T* sdata, unsigned int tid);

__global__ void chunk_sums_kernel(const FLOAT_T* data, unsigned int data_size, unsigned int chunkSize, FLOAT_T* res);

void chunk_sums(const FLOAT_T* data_d, unsigned int data_size, FLOAT_T* res_d, unsigned int Nchunks, unsigned int chunkSize);

FLOAT_T* sumBatched(const FLOAT_T* data, size_t data_size, size_t batchSize);

FLOAT_T* sumBatchedRecursive(const FLOAT_T* data, size_t data_size);

FLOAT_T weightSum(const FLOAT_T* data, size_t data_size);

}   // namespace mcl

#endif

#endif