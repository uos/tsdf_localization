#ifndef WHEELRESAMPLER_H
#define WHEELRESAMPLER_H

#include <resampling/resampler.h>

namespace mcl
{

class WheelResampler : public Resampler
{
public:
    WheelResampler() {}

    void resample(ParticleCloud& particle_cloud) override;
};

} // namespace mcl

#endif