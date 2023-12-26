 /**
 * @file rigid.cpp
 * @brief Rigid body physics evolving in 3D space, i.e. SE(3)
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#include "parameters.h"
#include "parameters.h"
#include "physics.h"
#include "geometry.h"

RigidPhysics::RigidPhysics()
{
    droneParameters& params_drone = droneParameters::getInstance();
    // Initialize the member variables
    velocity.setZero();
    position = params_drone.initPos;
    angularVelocity.setZero();
    orientation.setIdentity();
    externalForceBody.setZero();
    externalTorqueBody.setZero();
}

void RigidPhysics::updateState(float timeStep) {
    Geometry geometry;
    droneParameters& params_drone = droneParameters::getInstance();
    physicsParameters& params_phy = physicsParameters::getInstance();
    // Update the drone's state based on dynamics
    // Update translational dynamics:
    // Translational dynamics are evolving in inertial frame
    // Compute translational acceleration
    // Compute gravity force (NED reference frame)
    Eigen::Vector3f gravityForce(0.0, 0.0, params_phy.gravity * params_drone.mass_kg);

    // Get the orientation as rotation matrix
    Eigen::Matrix3f rotMat = geometry.quaternionToRotationMatrix(orientation);

    // Compute net force in inertial frame
    Eigen::Vector3f netForce = rotMat * externalForceBody + gravityForce;

    // Compute acceleration
    acceleration = netForce / params_drone.mass_kg;
    // Update velocity and position
    position = position + velocity * timeStep + 0.5 * acceleration * pow(timeStep,2) ;
    velocity += acceleration * timeStep;

    // Update rotational dynamics:
    // Rotational dynamics are evolving in body frame
    // Compute angular momentum
    Eigen::Vector3f angularMomentum = angularVelocity.cross(params_drone.inertiaMatrix * angularVelocity);
    // Calculate the angular acceleration
    Eigen::Vector3f angularAcceleration  = params_drone.inertiaMatrix.inverse() * (externalTorqueBody-angularMomentum);
    // Integrate the angular acceleration to update the angular velocity
    angularVelocity += angularAcceleration * timeStep;
    // Convert angular velocity to the time derivative of quaternion
    // source: https://ahrs.readthedocs.io/en/latest/filters/angular.html
    // source: https://github.com/burakyueksel/physics/blob/eeba843fe20e5fd4e2d5d2d3d9608ed038bfb069/src/physics.c#L93
    Eigen::Quaternionf orientationDot;
    orientationDot.w() = -0.5  * (angularVelocity.x() * orientation.x() + angularVelocity.y() * orientation.y() + angularVelocity.z() * orientation.z());
    orientationDot.x() =  0.5  * (angularVelocity.x() * orientation.w() + angularVelocity.z() * orientation.y() - angularVelocity.y() * orientation.z());
    orientationDot.y() =  0.5  * (angularVelocity.y() * orientation.w() - angularVelocity.z() * orientation.x() + angularVelocity.x() * orientation.z());
    orientationDot.z() =  0.5  * (angularVelocity.z() * orientation.w() + angularVelocity.y() * orientation.x() - angularVelocity.x() * orientation.y());
    // Integrate orientationDot with time step
    orientation.w() += orientationDot.w() * timeStep;
    orientation.x() += orientationDot.x() * timeStep;
    orientation.y() += orientationDot.y() * timeStep;
    orientation.z() += orientationDot.z() * timeStep;
    orientation.normalize();  // Normalize the quaternion
}

// external torques: control torques, disturbance torques, etc
void RigidPhysics::setExternalTorqueBody(const Eigen::Vector3f& torque) {
    externalTorqueBody = torque;
}

// external forces: control forces, disturbance forces, etc
void RigidPhysics::setExternalForceBody(const Eigen::Vector3f& force) {
    externalForceBody = force;
}

// get 3d positions
Eigen::Vector3f RigidPhysics::getPosition() const {
    return position;
}

// get 3d velocities
Eigen::Vector3f RigidPhysics::getVelocity() const {
    return velocity;
}

// get 3d accelerations
Eigen::Vector3f RigidPhysics::getAcceleration() const {
    return acceleration;
}

// get quaternion that defines the rotation of the body w.r.t. inertial frame
Eigen::Quaternionf RigidPhysics::getQuaternion() const {
    return orientation;
}

// get body frame rotational velocities
Eigen::Vector3f RigidPhysics::getBodyRates() const {
    return angularVelocity;
}