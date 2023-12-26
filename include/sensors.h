 /**
 * @file sensors.h
 * @brief Sensors library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-10
 */
#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations
#include <random> // For gaussian (normal) distributions

struct IMUStates
{
    Eigen::Vector3f acc_mps2;
    Eigen::Vector3f rotVel_rps;
};

struct BaroStates
{
  float temperature_c;  // temperature (in C)
  float pressure_pa;     // pressure (in Pa)
  float altitude_m;     // altitude (in m)
};

class Sensor
{
public:
    /**
     * @brief Sensor class
     */
    // IMU
    class IMU
    {
    public:
        /**
         * @brief IMU class
         */
        IMU(); // init
        IMUStates measurementModel(Eigen::Vector3f acceleration, Eigen::Vector3f angular_velocity);
    private:
        IMUStates imustates;
        // Standard deviations for sensor noise
        float accel_noise_stddev_;
        float gyro_noise_stddev_;
        // Generate random numbers from a normal distribution
        float generateNormalDistribution(float mean, float stddev)
        {
            std::normal_distribution<float> distribution(mean, stddev);
            return distribution(random_generator_);
        }
        // Random number generator
        std::default_random_engine random_generator_;
    };
    // BARO
    class Baro
    {
    public:
        /**
         * @brief IMU class
         */
        Baro(); // init
        BaroStates measurementModel(float temperature, float trueAltitude);
    private:
        BaroStates barostates;
        // Standard deviations for sensor noise
        float pressure_noise_stddev_;
        // Generate random numbers from a normal distribution
        float generateNormalDistribution(float mean, float stddev)
        {
            std::normal_distribution<float> distribution(mean, stddev);
            return distribution(random_generator_);
        }
        // Random number generator
        std::default_random_engine random_generator_;
    };
private:

};