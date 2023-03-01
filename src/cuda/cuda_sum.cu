#include <cuda/cuda_sum.h>

#include <cmath>

namespace mcl
{

__device__ void warpReduce(volatile FLOAT_T* sdata, unsigned int tid)
{
    if(blockDim.x >= 64) sdata[tid] += sdata[tid + 32];
    if(blockDim.x >= 32) sdata[tid] += sdata[tid + 16];
    if(blockDim.x >= 16) sdata[tid] += sdata[tid + 8];
    if(blockDim.x >=  8) sdata[tid] += sdata[tid + 4];
    if(blockDim.x >=  4) sdata[tid] += sdata[tid + 2];
    if(blockDim.x >=  2) sdata[tid] += sdata[tid + 1];
}

__global__ void chunk_sums_kernel(const FLOAT_T* data, unsigned int data_size, unsigned int chunkSize, FLOAT_T* res)
{

    __shared__ FLOAT_T sdata[1024];
    
    const unsigned int tid = threadIdx.x;
    const unsigned int globId = chunkSize * blockIdx.x + threadIdx.x;
    const unsigned int rows = (chunkSize + blockDim.x - 1) / blockDim.x;


    sdata[tid] = 0.0;
    for(unsigned int i=0; i<rows; i++)
    {
        if(tid + blockDim.x * i < chunkSize)
        {
            auto calc = globId + blockDim.x * i;

            if (calc < data_size)
            {
                sdata[threadIdx.x] += data[calc];
            }
        }
    }
    __syncthreads();

    for(unsigned int s= blockDim.x / 2; s > 32; s >>= 1)
    {
        if(tid < s)
        {
            sdata[tid] += sdata[tid + s];
        }
        __syncthreads();
    }

    if(tid < blockDim.x / 2 && tid < 32)
    {
        warpReduce(sdata, tid);
    }

    if(tid == 0)
    {
        res[blockIdx.x] = sdata[0];
    }
}

void chunk_sums(const FLOAT_T* data_d, unsigned int data_size, FLOAT_T* res_d, unsigned int Nchunks, unsigned int chunkSize)
{
    if(chunkSize >= 1024) 
    {
        chunk_sums_kernel<<<Nchunks, 1024>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 512) 
    {
        chunk_sums_kernel<<<Nchunks, 512>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 256) 
    {
        chunk_sums_kernel<<<Nchunks, 256>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 128) 
    {
        chunk_sums_kernel<<<Nchunks, 128>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 64) 
    {
        chunk_sums_kernel<<<Nchunks, 64>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 32) 
    {
        chunk_sums_kernel<<<Nchunks, 32>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 16) 
    {
        chunk_sums_kernel<<<Nchunks, 16>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 8) 
    {
        chunk_sums_kernel<<<Nchunks, 8>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 4) 
    {
        chunk_sums_kernel<<<Nchunks, 4>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 2) 
    {
        chunk_sums_kernel<<<Nchunks, 2>>>(data_d, data_size, chunkSize, res_d);
    } 
    else if(chunkSize >= 1) 
    {
        chunk_sums_kernel<<<Nchunks, 1>>>(data_d, data_size, chunkSize, res_d);
    }
}

FLOAT_T* sumBatched(const FLOAT_T* data, size_t data_size, size_t batchSize)
{
    size_t Nchunks = std::ceil(data_size / static_cast<double>(batchSize));

    FLOAT_T* sums;
    cudaMalloc((FLOAT_T**) &sums, sizeof(FLOAT_T) * Nchunks);
    chunk_sums(data, data_size, sums, Nchunks, batchSize);

    return sums;
}

double logN(double base, double x) 
{
    return log(x) / log(base);
}

FLOAT_T* sumBatchedRecursive(const FLOAT_T* data, size_t data_size)
{
    if (data_size <= 1024)
    {
        return sumBatched(data, data_size, data_size);
    }

    size_t Nchunks = std::ceil(data_size / static_cast<double>(1024));

    FLOAT_T* result = sumBatched(data, data_size, 1024);
    cudaDeviceSynchronize();
    size_t remain = std::ceil(data_size / static_cast<double>(1024));

    FLOAT_T* tmp_data;
    cudaMalloc((FLOAT_T**) &tmp_data, remain * sizeof(FLOAT_T));

    while (remain > 1)
    {
        Nchunks = std::ceil(remain / static_cast<double>(1024));

        chunk_sums(result, remain, tmp_data, Nchunks, 1024);
        cudaDeviceSynchronize();
        remain = std::ceil(remain / static_cast<double>(1024));

        auto swap = tmp_data;
        tmp_data = result;
        result = swap;
    }

    cudaFree(tmp_data);

    return result;
}

FLOAT_T weightSum(const FLOAT_T* data, size_t data_size)
{
    FLOAT_T weight_sum = 0;

    auto d_sums = sumBatchedRecursive(data, data_size);
    cudaCheck();

    cudaMemcpy(&weight_sum, d_sums, sizeof(FLOAT_T), cudaMemcpyDeviceToHost);

    cudaFree(d_sums);

    return weight_sum;
}


}   // namespace mcl
