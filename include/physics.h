 /**
 * @file physics.h
 * @brief Physics library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-03
 */
#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

class RigidPhysics
{
public:
    /**
     * @brief Rigid body dynamics
     */
    RigidPhysics(); // Constructor to initialize the object
    /**
     * Updates the state of the rigid body based on the given time step.
     * This function computes the new position, orientation, and velocity of the rigidy body.
     *
     * @param timeStep The time step for the state update.
     */
    void updateState(float timeStep);
    void setExternalForceBody(const Eigen::Vector3f& force); // Set the value of externalForceBody
    void setExternalTorqueBody(const Eigen::Vector3f& torque); // Set the value of externalTorqueBody
    /**
     * Gets the ID of the drone.
     *
     * @return The ID of the drone.
     */
    Eigen::Vector3f getPosition() const;
    Eigen::Vector3f getVelocity() const;
    Eigen::Vector3f getAcceleration() const;
    Eigen::Quaternionf getQuaternion() const;
    Eigen::Vector3f getBodyRates() const;
    Eigen::Quaternionf eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg);

    float quat2R33(const Eigen::Quaternionf& q);


private:
    // States of the rigid body
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f acceleration;
    Eigen::Vector3f angularVelocity; // Angular velocity as a member variable
    Eigen::Quaternionf orientation; // Orientation as a member variable
    Eigen::Vector3f externalTorqueBody; // External torque in body frame as a member variable
    Eigen::Vector3f externalForceBody; // External force in body frame as a member variable
    // Add other relevant private member variables

    // Define other relevant private member variables
};