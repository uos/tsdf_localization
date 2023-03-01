#ifndef CUDA_UTIL_H
#define CUDA_UTIL_H

#ifdef __CUDACC__

#define CUDA_CALLABLE_MEMBER __host__ __device__
#define HOST_CALLABLE_MEMBER __host__

#include <iostream>
#include <stdexcept>

void cudaCheck();

#else

#define CUDA_CALLABLE_MEMBER
#define HOST_CALLABLE_MEMBER

#endif 

#endif