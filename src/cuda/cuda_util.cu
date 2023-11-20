#include <tsdf_localization/cuda/cuda_util.h>

#ifdef __CUDACC__

#include <iostream>
#include <stdexcept>

void cudaCheck()
{
    auto code = cudaPeekAtLastError();

    if (code != cudaSuccess)
    {
        std::cout << "CUDA error: " << cudaGetErrorString(code) << std::endl;
        throw std::runtime_error("CUDA error occured!");
    }
}

#endif