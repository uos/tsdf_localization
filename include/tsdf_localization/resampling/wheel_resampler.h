#ifndef WHEELRESAMPLER_H
#define WHEELRESAMPLER_H

#include <tsdf_localization/resampling/resampler.h>

namespace tsdf_localization
{

class WheelResampler : public Resampler
{
public:
    WheelResampler() {}

    void resample(ParticleCloud& particle_cloud) override;
};

} // namespace tsdf_localization

#endif