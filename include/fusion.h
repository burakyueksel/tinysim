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
    KalmanFilter(const Eigen::MatrixXf& A, const Eigen::MatrixXf& H, const Eigen::MatrixXf& Q,
                 const Eigen::MatrixXf& R, const Eigen::VectorXf& initial_state,
                 const Eigen::MatrixXf& initial_covariance);
    void predict();
    void update(const Eigen::VectorXf& z);
    Eigen::VectorXf getState() const;

private:
    int n;      // Dimension of the state vector
    Eigen::MatrixXf A; // State transition matrix
    Eigen::MatrixXf H; // Measurement matrix
    Eigen::MatrixXf Q; // Process noise covariance matrix
    Eigen::MatrixXf R; // Measurement noise covariance matrix
    Eigen::VectorXf x_hat; // State estimate
    Eigen::MatrixXf P; // Estimate covariance matrix
};
