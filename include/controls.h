 /**
 * @file controls.h
 * @brief Control library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

class Control
{
public:
    /**
     * @brief Control functions
     */
    // tilt priorizing quaternion based attitude controller
    Eigen::Vector3d attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps);

    /**
     * @brief Utility functions
     */
    // Signum function template
    template <typename T>
    int mySignum(T value) {
        return (value > T(0)) - (value < T(0));
    }
private:

};