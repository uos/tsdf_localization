#ifndef RESAMPLER_H
#define RESAMPLER_H

#include <random>
#include <utility>

#include <particle_cloud.h>

#include <util/util.h>

namespace mcl
{

class ParticleCloud;

class Resampler
{
public:

    Resampler()
    {
        std::random_device device{};
        m_generator_ptr.reset(new std::mt19937(device()));
    }

    virtual void resample(ParticleCloud& particle_cloud) = 0;

protected:
    std::unique_ptr<std::mt19937> m_generator_ptr;

};

} // namespace mcl

#endif