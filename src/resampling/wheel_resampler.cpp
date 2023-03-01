#include <resampling/wheel_resampler.h>

namespace mcl
{

void WheelResampler::resample(ParticleCloud& particle_cloud)
{
    // Initializing an uniform distribution for choosing a new particle
    static std::uniform_real_distribution<> uniform_distribution(0.0, 1.0);

    auto tmp_particle_cloud = particle_cloud;

    for (auto particle_index = 0u; particle_index < particle_cloud.size(); ++particle_index)
    {
        FLOAT_T random_value = uniform_distribution(*m_generator_ptr);
        FLOAT_T weight_sum = 0.0;
        FLOAT_T current_weight;

        for(auto index = 0u; index < particle_cloud.size(); index++)
        { 
            current_weight = tmp_particle_cloud[index].second;
            weight_sum += current_weight;

            if(random_value <= weight_sum)
            {
                particle_cloud[particle_index] = tmp_particle_cloud[index];
                break;
            }
        }

        //particle_cloud[particle_index] = tmp_particle_cloud[particle_cloud.size() - 1];
    }

}

} // namespace mcl