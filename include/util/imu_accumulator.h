#ifndef IMU_ACC_H
#define IMU_ACC_H

#include <sensor_msgs/Imu.h>

#include <tuple>
#include <mutex>

#include <util/util.h>

namespace mcl
{

void bound_angle(FLOAT_T& angle);

class ImuAccumulator
{
public:

    struct Data
    {
        FLOAT_T linear_vel = 0.0;
        FLOAT_T delta_x = 0.0;

        FLOAT_T angular_roll = 0.0;
        FLOAT_T angular_pitch = 0.0;
        FLOAT_T angular_yaw = 0.0;

        FLOAT_T delta_roll = 0.0;
        FLOAT_T delta_pitch = 0.0;
        FLOAT_T delta_yaw = 0.0;
    };

    ImuAccumulator() : first_(true), data_{0.0, 0.0, 0.0, 0.0, 0.0}
    {

    }

    void update(const sensor_msgs::Imu& imu)
    {
        mutex_.lock();

        if (first_)
        {
            last_ = ros::Time::now();
            first_ = false;

            mutex_.unlock();

            return;
        }

        ros::Time current = ros::Time::now();

        auto dt = (current - last_).toSec();

        data_.delta_x += 0.5 * -imu.linear_acceleration.x * dt * dt + data_.linear_vel * dt;
        data_.linear_vel += -imu.linear_acceleration.x * dt;

        data_.delta_roll += imu.angular_velocity.x * dt;
        bound_angle(data_.delta_roll);

        data_.delta_pitch += imu.angular_velocity.y * dt;
        bound_angle(data_.delta_pitch);

        data_.delta_yaw += imu.angular_velocity.z * dt;
        bound_angle(data_.delta_yaw);

        if (std::fabs(data_.angular_roll) < std::fabs(imu.angular_velocity.x))
        {
            data_.angular_roll = imu.angular_velocity.x;
        }

        if (std::fabs(data_.angular_pitch) < std::fabs(imu.angular_velocity.y))
        {
            data_.angular_pitch = imu.angular_velocity.y;
        }

        if (std::fabs(data_.angular_yaw) < std::fabs(imu.angular_velocity.z))
        {
            data_.angular_yaw = imu.angular_velocity.z;
        }
        
        last_ = current;

        mutex_.unlock();
    }

    void getData(Data& data)
    {
        // mutex_.lock();
        data = data_;
        // mutex_.unlock();
    }

    void reset()
    {
        // mutex_.lock();

        data_.delta_x = 0.0;

        data_.delta_roll = 0.0;
        data_.delta_pitch = 0.0;
        data_.delta_yaw = 0.0;

        data_.angular_roll = 0.0;
        data_.angular_pitch = 0.0;
        data_.angular_yaw = 0.0;

        // mutex_.unlock();
    }

    void getAndResetData(Data& data)
    {
        mutex_.lock();

        getData(data);
        reset();

        mutex_.unlock();
    }

private:
    bool first_;
    ros::Time last_;

    Data data_;

    std::mutex mutex_;
};

} // namespace mcl

#endif