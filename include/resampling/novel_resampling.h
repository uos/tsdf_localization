#ifndef NOVEL_RESAMPLING_H
#define NOVEL_RESAMPLING_H

#include <resampling/resampler.h>

namespace mcl
{

class ResidualResampler : public Resampler
{
public:
    void resample(ParticleCloud& particle_cloud) override
    {
        std::uniform_int_distribution<size_t> uniform_distribution(0, particle_cloud.size() - 1);

        std::vector<Particle> new_particles;
        new_particles.reserve(particle_cloud.size());

        while (new_particles.size() < particle_cloud.size())
        {
            auto random_index = uniform_distribution(*m_generator_ptr);
            auto& particle = particle_cloud[random_index];
            auto expected_insertions = particle.second * particle_cloud.size();
            auto insertions_left = particle_cloud.size() - new_particles.size();
            auto insertions = expected_insertions <= insertions_left ? expected_insertions : insertions_left;

            for (size_t index = 0; index < insertions; ++index)
            {
                new_particles.push_back(particle);
            }
        }

        particle_cloud.particles() = std::move(new_particles);
    }

};

class SystematicResampler : public Resampler
{
public:
    void resample(ParticleCloud& particle_cloud) override
    {
        auto inverse_M = 1.0 / particle_cloud.size();
        std::uniform_real_distribution<FLOAT_T> uniform_distribution(0.0, inverse_M);

        std::vector<Particle> new_particles;
        new_particles.reserve(particle_cloud.size());

        auto U = uniform_distribution(*m_generator_ptr);
        auto s = 0.0;

        for (size_t m = 0; m < particle_cloud.size(); ++m)
        {
            auto& particle = particle_cloud[m];
            size_t k = 0;

            s += particle.second;

            while (s > U)
            {
                ++k;
                U += inverse_M;
            }

            for (size_t index = 0; index < k; ++index)
            {
                new_particles.push_back(particle);
            }
        }

        particle_cloud.particles() = std::move(new_particles);
    }

};

class ResidualSystematicResampler : public Resampler
{
public:
    void resample(ParticleCloud& particle_cloud) override
    {
        std::uniform_real_distribution<FLOAT_T> uniform_distribution(0.0, 1.0);

        std::vector<Particle> new_particles;
        new_particles.reserve(particle_cloud.size());

        auto u = uniform_distribution(*m_generator_ptr);

        for (size_t m = 0; m < particle_cloud.size(); ++m)
        {
            auto& particle = particle_cloud[m];
            auto temp = particle_cloud.size() * particle.second - u;
            size_t o = static_cast<size_t>(temp + 1.0);
            u = o - temp;

            for (size_t index = 0; index < o; ++index)
            {
                new_particles.push_back(particle);
            }
        }

        particle_cloud.particles() = std::move(new_particles);
    }

};

class MetropolisResampler : public Resampler
{
    size_t sampling_steps_;

public:
    MetropolisResampler(size_t sampling_steps) : sampling_steps_(sampling_steps) {}

    void resample(ParticleCloud& particle_cloud) override
    {
        std::uniform_real_distribution<FLOAT_T> uniform_real_distribution(0.0, 1.0);
        std::uniform_int_distribution<size_t> uniform_int_distribution(0, particle_cloud.size() - 1);

        std::vector<Particle> new_particles;
        new_particles.reserve(particle_cloud.size());

        for (size_t i = 0; i < particle_cloud.size(); ++i)
        {
            auto k = 0;
            
            auto& particle_k = particle_cloud[k];

            for (auto n = 0u; n < sampling_steps_; ++n)
            {
                auto u = uniform_real_distribution(*m_generator_ptr);
                auto j = uniform_int_distribution(*m_generator_ptr);
                auto& particle_j = particle_cloud[j];

                if (u <= particle_j.second / particle_k.second)
                {
                    k = j;
                }
            }

            new_particles.push_back(particle_cloud[k]);
        }

        particle_cloud.particles() = std::move(new_particles);
    }
};

class RejectionResampler : public Resampler
{
public:
    void resample(ParticleCloud& particle_cloud) override
    {
        std::uniform_real_distribution<FLOAT_T> uniform_real_distribution(0.0, 1.0);
        std::uniform_int_distribution<size_t> uniform_int_distribution(0, particle_cloud.size() - 1);

        std::vector<Particle> new_particles;
        new_particles.reserve(particle_cloud.size());

        auto sup_w = 0.0;

        for (size_t index = 0; index < particle_cloud.size(); ++index)
        {
            auto w = particle_cloud[index].second;

            if (sup_w < w)
            {
                sup_w = w;
            }
        }

        for (size_t i = 0; i < particle_cloud.size(); ++i)
        {
            auto j = i;
            auto u = uniform_real_distribution(*m_generator_ptr);

            while (u > (particle_cloud[j].second / sup_w))
            {
                j = uniform_int_distribution(*m_generator_ptr);
                u = uniform_real_distribution(*m_generator_ptr);
            }

            new_particles.push_back(particle_cloud[j]);
        }

        particle_cloud.particles() = std::move(new_particles);
    }

};

} // namespace mcl

#endif