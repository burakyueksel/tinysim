 /**
 * @file fusion.h
 * @brief (Sensor) Fusion library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-28
 */
#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

class KalmanFilter 
{
public:
    KalmanFilter(int n, double dt);
    void predict();
    void update(const Eigen::VectorXd& z);
    Eigen::VectorXd getState() const;

private:
    int n;      // Dimension of the state vector
    double dt;  // Time step
    Eigen::MatrixXd A; // State transition matrix
    Eigen::MatrixXd H; // Measurement matrix
    Eigen::MatrixXd Q; // Process noise covariance matrix
    Eigen::MatrixXd R; // Measurement noise covariance matrix
    Eigen::VectorXd x_hat; // State estimate
    Eigen::MatrixXd P; // Estimate covariance matrix
};
