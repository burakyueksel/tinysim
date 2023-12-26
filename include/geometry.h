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
    float quat2R33(const Eigen::Quaternionf& q);
    Eigen::Vector3f quaternionToEulerAngles(const Eigen::Quaternionf& q);
    Eigen::Matrix3f quaternionToRotationMatrix(const Eigen::Quaternionf& q);
    Eigen::Vector3f quat2Re3(const Eigen::Quaternionf& q);
    Eigen::Vector3f quat2RTe3(const Eigen::Quaternionf& q);
    Eigen::Quaternionf angleAxisToQuaternion (const float& angle_rad, const Eigen::Vector3f vector);
    Eigen::Quaternionf eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg);



private:

    // Add other relevant private member variables

    // Define other relevant private member variables
};