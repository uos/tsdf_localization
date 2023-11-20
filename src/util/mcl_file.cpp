#include <tsdf_localization/util/mcl_file.h>
#include <utility>
#include <fstream> 
#include <sstream>

namespace tsdf_localization
{

MCLFile::MCLFile(const std::string& file_name) : name_(file_name)
{

}

void MCLFile::write(const std::vector<CudaPoint>& points, const std::vector<int>& rings, const std::vector<Particle>& particles, const std::array<FLOAT_T, 16>& tf, FLOAT_T x, FLOAT_T y, FLOAT_T z, FLOAT_T q_1, FLOAT_T q_2, FLOAT_T q_3, FLOAT_T q_4) const
{
    std::ofstream file(name_);

    if (!file)
    {
        throw std::ifstream::failure("Error while opening file for writing");
    }

    std::stringstream data;

    data << points.size() << "\n\n";

    for (const auto& point : points)
    {
        data << point.x << ' ' << point.y << ' ' << point.z << '\n';
    }

    for (const auto& ring : rings)
    {
        data << ring << '\n';
    }

    data << particles.size() << '\n';

    for (const auto& particle : particles)
    {
        data << particle.first[0] << ' ' << particle.first[1] << ' ' << particle.first[2] << ' ' << particle.first[3] << ' ' << particle.first[4] << ' ' << particle.first[5] << "  " << particle.second << '\n';
    }

    data << '\n';

    for (auto index = 0; index < 16; ++index)
    {
        data << tf[index] << ' ';
    }

    data << '\n';

    data << x << " " << y << " " << z << " " << q_1 << " " << q_2 << " " << q_3 << " " << q_4;

    file << data.str();
    file.flush();

    if (!file)
    {
        throw std::ifstream::failure("Error while writing into file");
    }

    file.close();
}

void MCLFile::read(std::vector<CudaPoint>& points, std::vector<int>& rings, std::vector<Particle>& particles, std::array<FLOAT_T, 16>& tf, FLOAT_T& x, FLOAT_T& y, FLOAT_T& z, FLOAT_T& q_1, FLOAT_T& q_2, FLOAT_T& q_3, FLOAT_T& q_4) const
{
    std::ifstream file(name_);

    if(!file)
    {
        throw std::ifstream::failure("Error while opening file for reading");
    }

    size_t points_size = 0;
    size_t particles_size = 0;

    file >> points_size;

    points.resize(points_size);
    rings.resize(points_size);

    for (auto index = 0u; index < points_size; ++index)
    {
        file >> points[index].x >> points[index].y >> points[index].z;
    }

    for (auto index = 0u; index < points_size; ++index)
    {
        file >> rings[index];
    }

    file >> particles_size;

    particles.resize(particles_size);

    for (auto index = 0u; index < particles_size; ++index)
    {
        file >> particles[index].first[0] >> particles[index].first[1] >> particles[index].first[2] >> particles[index].first[3] >> particles[index].first[4] >> particles[index].first[5] >> particles[index].second;
    }

    for (auto index = 0; index < 16; ++index)
    {
        file >> tf[index];
    }

    file >> x >> y >> z >> q_1 >> q_2 >> q_3 >> q_4;

    if (!file)
    {
        throw std::ifstream::failure("Error: Could not read mcl data from file");
    }
}

} // namespace tsdf_localization