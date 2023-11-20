/**
 * @file particle.h
 * @author Marc Eisoldt (meisoldt@uni-osnabrueck.de)
 * 
 * @brief Definition of a particle that can be used on the CPU and the GPU
 * 
 * @version 0.1
 * @date 2022-06-18
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef PARTICLE_H
#define PARTICLE_H

#include <tsdf_localization/cuda/cuda_util.h>
#include <tsdf_localization/util/util.h>

namespace tsdf_localization
{

/**
 * @brief Data structure to use a particle on CPU and GPU
 * 
 */
struct Particle
{
  CUDA_CALLABLE_MEMBER Particle()
  {
    
  }

  CUDA_CALLABLE_MEMBER Particle(const Particle& other)
  {
    *this = other;
  }

  CUDA_CALLABLE_MEMBER Particle& operator=(const Particle& other)
  {
    first[0] = other.first[0];
    first[1] = other.first[1];
    first[2] = other.first[2];
    first[3] = other.first[3];
    first[4] = other.first[4];
    first[5] = other.first[5];
    second   = other.second;

    return *this;
  }

  // Coordinates (x, y, z, roll, pitch, yaw)
  FLOAT_T first[6];
  // Weight of the particle used the Monte Carlo Localization
  FLOAT_T second;
};

} // namespace tsdf_localization

#endif