/**
 * @file ring_buffer.h
 * @author Steffen Hinderink
 * @author Juri Vana
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <tsdf_localization/util/global_map.h>

/**
 * Three dimensional array that can be shifted without needing to copy every entry.
 * This is done by implementing it as a ring in every dimension.
 * The ring buffer can be used as a local map containing truncated signed distance function (tsdf) values.
 * @tparam T type of the values stored in the ring buffer
 *
 * In order to use the global map with the ring buffer, T has to be std::pair<float, float>.
 * Otherwise the function shift will not work.
 */
template<typename T>
class RingBuffer
{

private:

    /**
     * Side lengths of the ring buffer. They are always odd, so that there is a central cell.
     * The ring buffer contains sizeX * sizeY * sizeZ values.
     */
    int sizeX;
    int sizeY;
    int sizeZ;

    /// Actual data of the ring buffer.
    T* data;

    /// Position of the center of the cuboid in global coordinates.
    int posX;
    int posY;
    int posZ;

    /**
     * Offset of the data in the ring.
     * Each variable is the index of the center of the cuboid in the data array in its dimension.
     *
     * If size = 5, pos = 1 (-> indices range from -1 to 3) and offset = 3 (in one dimension),
     * the indices of the global map in the data array are for example:
     * 3 -1  0  1  2
     *          ^
     *       offset
     */
    int offsetX;
    int offsetY;
    int offsetZ;

    /// Pointer to the global map in which the values outside of the buffer are stored
    std::shared_ptr<GlobalMap> map;

public:

    /**
     * Constructor of the ring buffer.
     * The position is initialized to (0, 0, 0).
     * The sizes are given as parameters. If they are even, they are initialized as s + 1.
     * The offsets are initialized to size / 2, so that the ring boundarys match the array bounds.
     * @param sX Side length of the ring buffer in the x direction
     * @param sY Side length of the ring buffer in the y direction
     * @param sZ Side length of the ring buffer in the z direction
     * @param map Pointer to the global map
     */
    RingBuffer(unsigned int sX, unsigned int sY, unsigned int sZ, std::shared_ptr<GlobalMap> map);

    /**
     * Copy constructor of the ring buffer. Creates a deep copy.
     * @param other Ring buffer to be copied
     */
    RingBuffer(const RingBuffer<T>& other);

    /**
     * Assignment operator of the ring buffer. Creates a deep copy.
     * @param other Ring buffer to be copied
     */
    RingBuffer<T>& operator=(const RingBuffer<T>& other);

    /**
     * Destructor of the ring buffer.
     * Deletes the array in particular.
     */
    virtual ~RingBuffer();

    /**
     * Returns a value from the ring buffer per reference.
     * Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * @param x x-coordinate of the index in global coordinates
     * @param y y-coordinate of the index in global coordinates
     * @param z z-coordinate of the index in global coordinates
     * @return Value of the ring buffer
     */
    T& value(int x, int y, int z);

    /**
     * Returns a value from the ring buffer per reference.
     * Throws an exception if the index is out of bounds i.e. if it is more than size / 2 away from the position.
     * @param x x-coordinate of the index in global coordinates
     * @param y y-coordinate of the index in global coordinates
     * @param z z-coordinate of the index in global coordinates
     * @return Value of the ring buffer
     */
    const T& value(int x, int y, int z) const;

    /**
     * Returns the size of the ring buffer per reference in an array of length 3.
     * @param size Array in which the size is stored
     */
    void getSize(int* size) const;

    /**
     * Returns the position of the center of the cuboid in global coordinates per reference in an array of length 3.
     * @param pos Array in which the position is stored
     */
    void getPos(int* pos) const;

    /**
     * Shifts the ring buffer, so that a new position is the center of the cuboid.
     * Entries, that stay in the buffer, stay in place.
     * Values outside of the buffer are loaded from and stored in the global map.
     * @param x x-coordinate of the new position
     * @param y y-coordinate of the new position
     * @param z z-coordinate of the new position
     */
    void shift(int x, int y, int z);

    /**
     * Checks if x, y and z are within the current range
     * 
     * @param x x-coordinate to check
     * @param y y-coordinate to check
     * @param z z-coordinate to check
     * @return true if (x, y, z) is within the area of the buffer
     */
    bool inBounds(int x, int y, int z) const;

    T* getData() const
    {
        return data;
    }

    /**
     * Alternative method to handle map overflow (should replace modulo)
     * 
     * @param val Index which should be proved
     * @param max Upper bound for the index (exclusiv)
     * @return int The reduced value so that it is in the interval [0, max)
     */
    int overflow(int val, int max) const
    {
        if (val >= 2 * max)
        {
            return val - 2 * max;
        }
        else if (val >= max)
        {
            return val - max;
        }
        else
        {
            return val;
        }
    }

    /**
     * Get correct index in the one dimensional data array based on the three dimensional indexing 
     * 
     * @param x Index for the x axis
     * @param y Index for the y axis
     * @param z Index for the z axis
     * @return int The equivalent index in the encapsulated one dimensional data array
     */
    int getIndex(int x, int y, int z) const
    {
        int x_offset = overflow(x - posX + offsetX + sizeX, sizeX) * sizeY * sizeZ;
        int y_offset = overflow(y - posY + offsetY + sizeY, sizeY) * sizeZ;
        int z_offset = overflow(z - posZ + offsetZ + sizeZ, sizeZ);
        int index = x_offset  + y_offset + z_offset;
        return index;
    }

};

#include <ring_buffer/ring_buffer.tcc>

#endif
