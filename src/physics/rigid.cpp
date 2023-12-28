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
    velocity_mps.setZero();
    position_m = params_drone.initPos_m;
    angularVelocity_rps.setZero();
    orientation.setIdentity();
    externalForceBody_N.setZero();
    externalTorqueBody_Nm.setZero();
}

void RigidPhysics::updateState(float timeStep_s) {
    Geometry geometry;
    droneParameters& params_drone = droneParameters::getInstance();
    physicsParameters& params_phy = physicsParameters::getInstance();
    // Update the drone's state based on dynamics
    // Update translational dynamics:
    // Translational dynamics are evolving in inertial frame
    // Compute translational acceleration
    // Compute gravity force (NED reference frame)
    Eigen::Vector3f gravityForce(0.0, 0.0, params_phy.gravity_mps2 * params_drone.mass_kg);

    // Get the orientation as rotation matrix
    Eigen::Matrix3f rotMat = geometry.quaternionToRotationMatrix(orientation);

    // Compute net force in inertial frame
    Eigen::Vector3f netForce = rotMat * externalForceBody_N + gravityForce;

    // Compute acceleration
    acceleration_mps2 = netForce / params_drone.mass_kg;
    // Update velocity and position
    position_m = position_m + velocity_mps * timeStep_s + 0.5 * acceleration_mps2 * pow(timeStep_s,2) ;
    velocity_mps += acceleration_mps2 * timeStep_s;

    // Update rotational dynamics:
    // Rotational dynamics are evolving in body frame
    // Compute angular momentum
    Eigen::Vector3f angularMomentum = angularVelocity_rps.cross(params_drone.inertiaMatrix_kgm2 * angularVelocity_rps);
    // Calculate the angular acceleration
    Eigen::Vector3f angularAcceleration  = params_drone.inertiaMatrix_kgm2.inverse() * (externalTorqueBody_Nm-angularMomentum);
    // Integrate the angular acceleration to update the angular velocity
    angularVelocity_rps += angularAcceleration * timeStep_s;
    // Convert angular velocity to the time derivative of quaternion
    // source: https://ahrs.readthedocs.io/en/latest/filters/angular.html
    // source: https://github.com/burakyueksel/physics/blob/eeba843fe20e5fd4e2d5d2d3d9608ed038bfb069/src/physics.c#L93
    Eigen::Quaternionf orientationDot;
    orientationDot.w() = -0.5  * (angularVelocity_rps.x() * orientation.x() + angularVelocity_rps.y() * orientation.y() + angularVelocity_rps.z() * orientation.z());
    orientationDot.x() =  0.5  * (angularVelocity_rps.x() * orientation.w() + angularVelocity_rps.z() * orientation.y() - angularVelocity_rps.y() * orientation.z());
    orientationDot.y() =  0.5  * (angularVelocity_rps.y() * orientation.w() - angularVelocity_rps.z() * orientation.x() + angularVelocity_rps.x() * orientation.z());
    orientationDot.z() =  0.5  * (angularVelocity_rps.z() * orientation.w() + angularVelocity_rps.y() * orientation.x() - angularVelocity_rps.x() * orientation.y());
    // Integrate orientationDot with time step
    orientation.w() += orientationDot.w() * timeStep_s;
    orientation.x() += orientationDot.x() * timeStep_s;
    orientation.y() += orientationDot.y() * timeStep_s;
    orientation.z() += orientationDot.z() * timeStep_s;
    orientation.normalize();  // Normalize the quaternion
}

// external torques: control torques, disturbance torques, etc
void RigidPhysics::setExternalTorqueBody(const Eigen::Vector3f& torque) {
    externalTorqueBody_Nm = torque;
}

// external forces: control forces, disturbance forces, etc
void RigidPhysics::setExternalForceBody(const Eigen::Vector3f& force) {
    externalForceBody_N = force;
}

// get 3d positions
Eigen::Vector3f RigidPhysics::getPosition() const {
    return position_m;
}

// get 3d velocities
Eigen::Vector3f RigidPhysics::getVelocity() const {
    return velocity_mps;
}

// get 3d accelerations
Eigen::Vector3f RigidPhysics::getAcceleration() const {
    return acceleration_mps2;
}

// get quaternion that defines the rotation of the body w.r.t. inertial frame
Eigen::Quaternionf RigidPhysics::getQuaternion() const {
    return orientation;
}

// get body frame rotational velocities
Eigen::Vector3f RigidPhysics::getBodyRates() const {
    return angularVelocity_rps;
}