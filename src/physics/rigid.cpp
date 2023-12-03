#include "parameters.h"
#include "physics.h"
#include <math.h>

RigidPhysics::RigidPhysics()
{
    Parameters& params = Parameters::getInstance();
    // Initialize the member variables
    velocity.setZero();
    position = params.initPos;
    angularVelocity.setZero();
    orientation.setIdentity();
    externalForceBody.setZero();
    externalTorqueBody.setZero();
}

Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& q)
{
    // Convert quaternion to rotation matrix
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();
    // Define Euler angle vector
    Eigen::Vector3d eulerAngles;

    // Extract roll, pitch, and yaw angles from rotation matrix
    eulerAngles.x() = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    eulerAngles.y() = asin(-rotationMatrix(2, 0));
    eulerAngles.z() = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

    return eulerAngles;
}

Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& q)
{
    // Convert quaternion to rotation matrix
    // This rotation represents the orientation of the body frame w.r.t. an inertial frame.
    // Meaning, it can left multiply a vector represented in body frame, to bring it to the inertial frame
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;
}

Eigen::Vector3d quat2Re3(const Eigen::Quaterniond& q)
{
    // Compute Re3, that is the third column of the rotation matrix (from body to inertial frame)
    Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    Eigen::Vector3d Re3 = R.col(2);

    return Re3;
}

Eigen::Vector3d quat2RTe3(const Eigen::Quaterniond& q)
{
    // Compute R^Te3, that is the third column of the transpose of the rotation matrix (from inertial to body frame)
    Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    Eigen::Matrix3d RT = R.transpose();
    Eigen::Vector3d RTe3 = RT.col(2);

    return RTe3;
}

double quat2R33(const Eigen::Quaterniond& q)
{
    // compute the (3,3)th element of the rotation matrix from quaternion
    Eigen::Matrix3d R = quaternionToRotationMatrix(q);
    double minThreshold = 1e-4;
    // protect it for very small numbers (we tend to use this value for division)
    double R33 = std::abs(R.coeff(2, 2)) < minThreshold ? minThreshold : R.coeff(2, 2);

    return R33;
}


/*
Implements eq 1 of https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
*/
Eigen::Quaterniond angleAxisToQuaternion (const double& angle, const Eigen::Vector3d vector)
{
    // define unit quaternion
    Eigen::Quaterniond quat;
    // trigonometric constants
    double ca2 = cos(angle/2);
    double sa2 = sin(angle/2);
    quat.w() = ca2;
    quat.x() = vector.x()*sa2;
    quat.y() = vector.y()*sa2;
    quat.z() = vector.z()*sa2;

    return quat;
}

Eigen::Quaterniond eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
    // Convert roll, pitch, and yaw angles to radians
    double rollRad = roll_deg * M_PI / 180.0;
    double pitchRad = pitch_deg * M_PI / 180.0;
    double yawRad = yaw_deg * M_PI / 180.0;

    // Calculate half angles
    double cosRollHalf = cos(rollRad / 2.0);
    double sinRollHalf = sin(rollRad / 2.0);
    double cosPitchHalf = cos(pitchRad / 2.0);
    double sinPitchHalf = sin(pitchRad / 2.0);
    double cosYawHalf = cos(yawRad / 2.0);
    double sinYawHalf = sin(yawRad / 2.0);

    // Compute quaternion components
    double w = cosRollHalf * cosPitchHalf * cosYawHalf + sinRollHalf * sinPitchHalf * sinYawHalf;
    double x = sinRollHalf * cosPitchHalf * cosYawHalf - cosRollHalf * sinPitchHalf * sinYawHalf;
    double y = cosRollHalf * sinPitchHalf * cosYawHalf + sinRollHalf * cosPitchHalf * sinYawHalf;
    double z = cosRollHalf * cosPitchHalf * sinYawHalf - sinRollHalf * sinPitchHalf * cosYawHalf;

    // Return the quaternion
    return Eigen::Quaterniond(w, x, y, z);
};

void RigidPhysics::updateState(double timeStep) {
    // Update the drone's state based on dynamics
    Parameters& params = Parameters::getInstance();

    // Update translational dynamics:
    // Translational dynamics are evolving in inertial frame
    // Compute translational acceleration
    // Compute gravity force (NED reference frame)
    Eigen::Vector3d gravityForce(0.0, 0.0, 9.81 * params.mass);

    // Get the orientation as rotation matrix
    Eigen::Matrix3d rotMat = quaternionToRotationMatrix(orientation);

    // Compute net force in inertial frame
    Eigen::Vector3d netForce = rotMat * externalForceBody + gravityForce;

    // Compute acceleration
    Eigen::Vector3d acceleration = netForce / params.mass;
    // Update velocity and position
    position = position + velocity * timeStep + 0.5 * acceleration * pow(timeStep,2) ;
    velocity += acceleration * timeStep;

    // Update rotational dynamics:
    // Rotational dynamics are evolving in body frame
    // Compute angular momentum
    Eigen::Vector3d angularMomentum = angularVelocity.cross(params.inertiaMatrix * angularVelocity);
    // Calculate the angular acceleration
    Eigen::Vector3d angularAcceleration  = params.inertiaMatrix.inverse() * (externalTorqueBody-angularMomentum);
    // Integrate the angular acceleration to update the angular velocity
    angularVelocity += angularAcceleration * timeStep;
    // Convert angular velocity to the time derivative of quaternion
    // source: https://ahrs.readthedocs.io/en/latest/filters/angular.html
    // source: https://github.com/burakyueksel/physics/blob/eeba843fe20e5fd4e2d5d2d3d9608ed038bfb069/src/physics.c#L93
    Eigen::Quaterniond orientationDot;
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
void RigidPhysics::setExternalTorqueBody(const Eigen::Vector3d& torque) {
    externalTorqueBody = torque;
}

// external forces: control forces, disturbance forces, etc
void RigidPhysics::setExternalForceBody(const Eigen::Vector3d& force) {
    externalForceBody = force;
}

// get 3d positions
Eigen::Vector3d RigidPhysics::getPosition() const {
    return position;
}

// get 3d velocities
Eigen::Vector3d RigidPhysics::getVelocity() const {
    return velocity;
}

// get quaternion that defines the rotation of the body w.r.t. inertial frame
Eigen::Quaterniond RigidPhysics::getQuaternion() const {
    return orientation;
}

// get body frame rotational velocities
Eigen::Vector3d RigidPhysics::getBodyRates() const {
    return angularVelocity;
}