/**
 * @file ring_buffer.cpp
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#include <stdlib.h> // for abs
#include <stdexcept>

template<typename T>
RingBuffer<T>::RingBuffer(unsigned int sX, unsigned int sY, unsigned int sZ, std::shared_ptr<GlobalMap> map)
    : posX(0),
      posY(0),
      posZ(0),
      sizeX(sX % 2 == 1 ? sX : sX + 1),
      sizeY(sY % 2 == 1 ? sY : sY + 1),
      sizeZ(sZ % 2 == 1 ? sZ : sZ + 1),
      map(map)
{
    offsetX = sizeX / 2;
    offsetY = sizeY / 2;
    offsetZ = sizeZ / 2;
    data = new T[sizeX * sizeY * sizeZ];
    auto default_value = map->get_value(Vector3i(0, 0, 0));
    for (int i = 0; i < sizeX * sizeY * sizeZ; i++)
    {
        data[i] = default_value;
    }
}

template<typename T>
RingBuffer<T>::RingBuffer(const RingBuffer& other) : data(nullptr)
{
    *this = other;
}

template<typename T>
RingBuffer<T>& RingBuffer<T>::operator=(const RingBuffer& other)
{

    if (data == other.data)
    {
        return *this;
    }

    if (data != nullptr)
    {
        delete [] data;
        data = nullptr;
    }

    posX = other.posX;
    posY = other.posY;
    posZ = other.posZ;

    sizeX = other.sizeX;
    sizeY = other.sizeY;
    sizeZ = other.sizeZ;

    offsetX = other.offsetX;
    offsetY = other.offsetY;
    offsetZ = other.offsetZ;

    map = other.map;

    auto total_size = sizeX * sizeY * sizeZ;

    data = new T[total_size];

    std::copy(other.data, other.data + total_size, data);

    return *this;
}

template<typename T>
RingBuffer<T>::~RingBuffer()
{
    if (data != nullptr)
    {
        delete [] data;
        data = nullptr;
    }
}

template<typename T>
T& RingBuffer<T>::value(int x, int y, int z)
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }

    return data[getIndex(x, y, z)];
}

template<typename T>
const T& RingBuffer<T>::value(int x, int y, int z) const
{
    if (!inBounds(x, y, z))
    {
        throw std::out_of_range("Index out of bounds");
    }
    return data[getIndex(x, y, z)];
}

template<typename T>
void RingBuffer<T>::getSize(int* size) const
{
    size[0] = sizeX;
    size[1] = sizeY;
    size[2] = sizeZ;
}

template<typename T>
void RingBuffer<T>::getPos(int* pos) const
{
    pos[0] = posX;
    pos[1] = posY;
    pos[2] = posZ;
}

template<typename T>
bool RingBuffer<T>::inBounds(int x, int y, int z) const
{
    return abs(x - posX) <= sizeX / 2 && abs(y - posY) <= sizeY / 2 && abs(z - posZ) <= sizeZ / 2;
}

template<typename T>
void RingBuffer<T>::shift(int x, int y, int z)
{
    // x
    int diffX = x - posX;
    for (int i = 0; i < abs(diffX); i++)
    {
        // step x
        for (int j = posY - sizeY / 2; j <= posY + sizeY / 2; j++)
        {
            for (int k = posZ - sizeZ / 2; k <= posZ + sizeZ / 2; k++)
            {
                if (diffX > 0)
                {
                    // step forward
                    T& inout = value(posX - sizeX / 2, j, k);
                    if (inout.second != 0)
                    {
                        map->set_value(Vector3i(posX - sizeX / 2, j, k), inout);
                    }
                    inout = map->get_value(Vector3i(posX + sizeX / 2 + 1, j, k));
                }
                else
                {
                    // step backwards
                    T& inout = value(posX + sizeX / 2, j, k);
                    if (inout.second != 0)
                    {
                        map->set_value(Vector3i(posX + sizeX / 2, j, k), inout);
                    }
                    inout = map->get_value(Vector3i(posX - sizeX / 2 - 1, j, k));
                }
            }
        }
        posX = diffX > 0 ? posX + 1 : posX - 1;
        offsetX = diffX > 0 ? (offsetX + 1) % sizeX : (offsetX - 1 + sizeX) % sizeX;
    }

    // y
    int diffY = y - posY;
    for (int i = 0; i < abs(diffY); i++)
    {
        // step y
        for (int j = posX - sizeX / 2; j <= posX + sizeX / 2; j++)
        {
            for (int k = posZ - sizeZ / 2; k <= posZ + sizeZ / 2; k++)
            {
                if (diffY > 0)
                {
                    // step forward
                    T& inout = value(j, posY - sizeY / 2, k);
                    if (inout.second != 0)
                    {
                        map->set_value(Vector3i(j, posY - sizeY / 2, k), inout);
                    }
                    inout = map->get_value(Vector3i(j, posY + sizeY / 2 + 1, k));
                }
                else
                {
                    // step backwards
                    T& inout = value(j, posY + sizeY / 2, k);
                    if (inout.second != 0)
                    {
                        map->set_value(Vector3i(j, posY + sizeY / 2, k), inout);
                    }
                    inout = map->get_value(Vector3i(j, posY - sizeY / 2 - 1, k));
                }
            }
        }
        posY = diffY > 0 ? posY + 1 : posY - 1;
        offsetY = diffY > 0 ? (offsetY + 1) % sizeY : (offsetY - 1 + sizeY) % sizeY;
    }

    // z
    int diffZ = z - posZ;
    for (int i = 0; i < abs(diffZ); i++)
    {
        // step z
        for (int j = posX - sizeX / 2; j <= posX + sizeX / 2; j++)
        {
            for (int k = posY - sizeY / 2; k <= posY + sizeY / 2; k++)
            {
                if (diffZ > 0)
                {
                    // step forward
                    T& inout = value(j, k, posZ - sizeZ / 2);
                    if (inout.second != 0)
                    {
                        map->set_value(Vector3i(j, k, posZ - sizeZ / 2), inout);
                    }
                    inout = map->get_value(Vector3i(j, k, posZ + sizeZ / 2 + 1));
                }
                else
                {
                    // step backwards
                    T& inout = value(j, k, posZ + sizeZ / 2);
                    if (inout.second != 0)
                    {
                        map->set_value(Vector3i(j, k, posZ + sizeZ / 2), inout);
                    }
                    inout = map->get_value(Vector3i(j, k, posZ - sizeZ / 2 - 1));
                }
            }
        }
        posZ = diffZ > 0 ? posZ + 1 : posZ - 1;
        offsetZ = diffZ > 0 ? (offsetZ + 1) % sizeZ : (offsetZ - 1 + sizeZ) % sizeZ;
    }
}
