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
    Eigen::Vector3d acc_mps2;
    Eigen::Vector3d rotVel_rps;
};

struct BaroStates
{
  double temperature_c;  // temperature (in C)
  double pressure_pa;     // pressure (in Pa)
  double altitude_m;     // altitude (in m)
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
        IMUStates measurementModel(Eigen::Vector3d acceleration, Eigen::Vector3d angular_velocity);
    private:
        IMUStates imustates;
        // Standard deviations for sensor noise
        double accel_noise_stddev_;
        double gyro_noise_stddev_;
        // Generate random numbers from a normal distribution
        double generateNormalDistribution(double mean, double stddev)
        {
            std::normal_distribution<double> distribution(mean, stddev);
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
        BaroStates measurementModel(double temperature, double trueAltitude);
    private:
        BaroStates barostates;
        // Standard deviations for sensor noise
        double pressure_noise_stddev_;
        // Generate random numbers from a normal distribution
        double generateNormalDistribution(double mean, double stddev)
        {
            std::normal_distribution<double> distribution(mean, stddev);
            return distribution(random_generator_);
        }
        // Random number generator
        std::default_random_engine random_generator_;
    };
private:

};