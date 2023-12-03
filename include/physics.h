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
    void updateState(double timeStep);
    void setExternalForceBody(const Eigen::Vector3d& force); // Set the value of externalForceBody
    void setExternalTorqueBody(const Eigen::Vector3d& torque); // Set the value of externalTorqueBody
    /**
     * Gets the ID of the drone.
     *
     * @return The ID of the drone.
     */
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getVelocity() const;
    Eigen::Quaterniond getQuaternion() const;
    Eigen::Vector3d getBodyRates() const;
    Eigen::Quaterniond eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg);

private:
    // States of the rigid body
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d angularVelocity; // Angular velocity as a member variable
    Eigen::Quaterniond orientation; // Orientation as a member variable
    Eigen::Vector3d externalTorqueBody; // External torque in body frame as a member variable
    Eigen::Vector3d externalForceBody; // External force in body frame as a member variable
    // Add other relevant private member variables

    // Define other relevant private member variables
};