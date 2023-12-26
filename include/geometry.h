 /**
 * @file geometry.h
 * @brief geometry library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-06
 */

#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

class Geometry
{
public:
    /**
     * @brief Geometry class
     */
    double quat2R33(const Eigen::Quaterniond& q);
    Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& q);
    Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q);
    Eigen::Vector3d quat2Re3(const Eigen::Quaterniond& q);
    Eigen::Vector3d quat2RTe3(const Eigen::Quaterniond& q);
    Eigen::Quaterniond angleAxisToQuaternion (const double& angle_rad, const Eigen::Vector3d vector);
    Eigen::Quaterniond eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg);



private:

    // Add other relevant private member variables

    // Define other relevant private member variables
};