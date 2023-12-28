 /**
 * @file kalman.cpp
 * @brief Kalman Filters.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-28
 */
#include "fusion.h"

KalmanFilter::KalmanFilter(int n, double dt) : n(n), dt(dt)
{
    // Initialize matrices and vectors based on user-defined dimension (n)
    // TODO: Allow initialization from outside. Right now it is hard codded here, which is not nice
    // because for each KF instance you probably gonna use different matrices and parameters.
    A = Eigen::MatrixXd::Identity(n, n);
    H = Eigen::MatrixXd::Identity(n, n);
    Q = Eigen::MatrixXd::Identity(n, n);
    R = Eigen::MatrixXd::Identity(n, n);
    x_hat = Eigen::VectorXd::Zero(n);
    P = Eigen::MatrixXd::Identity(n, n);
}

void KalmanFilter::predict()
{
    // Prediction step
    x_hat = A * x_hat;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(const Eigen::VectorXd& z)
{
    // Update step
    Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x_hat = x_hat + K * (z - H * x_hat);
    P = (Eigen::MatrixXd::Identity(n, n) - K * H) * P;
}

Eigen::VectorXd KalmanFilter::getState() const
{
    return x_hat;
}
