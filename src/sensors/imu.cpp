 /**
 * @file sensors.h
 * @brief Sensors library declerations
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

IMUStates Sensor::IMU::measurementModel(Eigen::Vector3d acceleration, Eigen::Vector3d angular_velocity)
{
    IMUStates imustates;
    // Simulate accelerometer measurements
    for (int i = 0; i < 3; ++i) {
        imustates.acc[i] = generateNormalDistribution(acceleration[i], accel_noise_stddev_);
    }

    // Simulate gyroscope measurements
    for (int i = 0; i < 3; ++i) {
        imustates.rotVel[i] = generateNormalDistribution(angular_velocity[i], gyro_noise_stddev_);
    }
    return imustates;
}