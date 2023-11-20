/**
 * @author Steffen Hinderink
 * @author Juri Vana
 * @author Malte Hillmann
 */

#include <sstream>

#include <tsdf_localization/util/global_map.h>

GlobalMap::GlobalMap(std::string name, TSDFValue::ValueType initial_tsdf_value, TSDFValue::WeightType initial_weight)
    : file_{name, HighFive::File::OpenOrCreate | HighFive::File::Truncate}, // Truncate clears already existing file
      initial_tsdf_value_{initial_tsdf_value, initial_weight},
      active_chunks_{},
      num_poses_{0}
{
    if (!file_.exist("/map"))
    {
        file_.createGroup("/map");
    }
    if (!file_.exist("/poses"))
    {
        file_.createGroup("/poses");
    }
}

std::string GlobalMap::tag_from_chunk_pos(const Vector3i& pos)
{
    std::stringstream ss;
    ss << pos.x() << "_" << pos.y() << "_" << pos.z();
    return ss.str();
}

int GlobalMap::index_from_pos(Vector3i pos, const Vector3i& chunkPos)
{
    pos -= chunkPos * CHUNK_SIZE;
    return (pos.x() * CHUNK_SIZE * CHUNK_SIZE + pos.y() * CHUNK_SIZE + pos.z());
}

std::vector<TSDFValue::RawType>& GlobalMap::activate_chunk(const Vector3i& chunkPos)
{
    int index = -1;
    int n = active_chunks_.size();
    for (int i = 0; i < n; i++)
    {
        if (active_chunks_[i].pos == chunkPos)
        {
            // chunk is already active
            index = i;
        }
    }
    if (index == -1)
    {
        // chunk is not already active
        ActiveChunk newChunk;
        newChunk.pos = chunkPos;
        newChunk.age = 0;

        HighFive::Group g = file_.getGroup("/map");
        auto tag = tag_from_chunk_pos(chunkPos);
        if (g.exist(tag))
        {
            // read chunk from file
            HighFive::DataSet d = g.getDataSet(tag);
            d.read(newChunk.data);
        }
        else
        {
            // create new chunk
            newChunk.data = std::vector<TSDFValue::RawType>(CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE, initial_tsdf_value_.raw()); //std::vector<int>(TOTAL_SIZE);
        }
        // put new chunk into active_chunks_
        if (n < NUM_CHUNKS)
        {
            // there is still room for active chunks
            index = n;
            newChunk.age = n; // temporarily assign oldest age so that all other ages get incremented
            active_chunks_.push_back(newChunk);
        }
        else
        {
            // write oldest chunk into file
            int max = -1;
            for (int i = 0; i < n; i++)
            {
                if (active_chunks_[i].age > max)
                {
                    max = active_chunks_[i].age;
                    index = i;
                }
            }
            auto tag = tag_from_chunk_pos(active_chunks_[index].pos);

            if (g.exist(tag))
            {
                auto d = g.getDataSet(tag);
                d.write(active_chunks_[index].data);
            }
            else
            {
                g.createDataSet(tag, active_chunks_[index].data);
            }
            // overwrite with new chunk
            active_chunks_[index] = newChunk;
        }
    }
    // update ages
    int age = active_chunks_[index].age;
    for (auto& chunk : active_chunks_)
    {
        if (chunk.age < age)
        {
            chunk.age++;
        }
    }
    active_chunks_[index].age = 0;
    return active_chunks_[index].data;
}

static inline Vector3i floor_shift(const Vector3i& a, int shift)
{
    return Vector3i(
        a[0] >> shift,
        a[1] >> shift,
        a[2] >> shift
    );
}

TSDFValue GlobalMap::get_value(const Vector3i& pos)
{
    Vector3i chunkPos = floor_shift(pos, CHUNK_SHIFT);
    const auto& chunk = activate_chunk(chunkPos);
    int index = index_from_pos(pos, chunkPos);
    return TSDFValue(chunk[index]);
}

void GlobalMap::set_value(const Vector3i& pos, const TSDFValue& value)
{
    Vector3i chunkPos = floor_shift(pos, CHUNK_SHIFT);
    auto& chunk = activate_chunk(chunkPos);
    int index = index_from_pos(pos, chunkPos);
    chunk[index] = value.raw();
}

void GlobalMap::savePose(float x, float y, float z, float roll, float pitch, float yaw)
{
    HighFive::Group g = file_.getGroup("/poses");
    std::vector<float> pose {x, y, z, roll, pitch, yaw};
    std::stringstream ss;
    ss << num_poses_;
    g.createDataSet(ss.str(), pose);
    num_poses_++;
}

void GlobalMap::write_back()
{
    HighFive::Group g = file_.getGroup("/map");
    for (auto& chunk : active_chunks_)
    {
        auto tag = tag_from_chunk_pos(chunk.pos);

        if (g.exist(tag))
        {
            auto d = g.getDataSet(tag);
            d.write(chunk.data);
        }
        else
        {
            g.createDataSet(tag, chunk.data);
        }
    }
    file_.flush();
}
