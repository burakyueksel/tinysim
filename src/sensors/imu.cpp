 /**
 * @file imu.cpp
 * @brief IMU sensor function bodies.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-10
 */
#include "sensors.h"
#include "parameters.h"

Sensor::IMU::IMU()
{
    droneParameters& params_drone = droneParameters::getInstance();
    accel_noise_stddev_ = params_drone.imuAccStdev;
    gyro_noise_stddev_  = params_drone.imuGyrStdev;
    // Seed the random number generator
    random_generator_.seed(std::random_device()());
}

IMUStates Sensor::IMU::measurementModel(Eigen::Vector3f acceleration, Eigen::Vector3f angular_velocity)
{
    IMUStates imustates;
    // Simulate accelerometer measurements
    for (int i = 0; i < 3; ++i)
    {
        // zero bias
        imustates.acc_mps2[i] = acceleration[i] + generateNormalDistribution(0.0, accel_noise_stddev_);
    }

    // Simulate gyroscope measurements
    for (int i = 0; i < 3; ++i)
    {
        // zero bias
        imustates.rotVel_rps[i] = angular_velocity[i] + generateNormalDistribution(0.0, gyro_noise_stddev_);
    }
    return imustates;
}