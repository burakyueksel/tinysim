 /**
 * @file so3.cpp
 * @brief 3D rigid body rotation functions. Evolving in SO3 and its tangent space so3.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-06
 */

#include "geometry.h"
#include "parameters.h"

Eigen::Vector3f Geometry::quaternionToEulerAngles(const Eigen::Quaternionf& q)
{
    // Convert quaternion to rotation matrix
    Eigen::Matrix3f rotationMatrix = q.normalized().toRotationMatrix();
    // Define Euler angle vector
    Eigen::Vector3f eulerAngles;

    // Extract roll, pitch, and yaw angles from rotation matrix
    eulerAngles.x() = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    eulerAngles.y() = asin(-rotationMatrix(2, 0));
    eulerAngles.z() = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));

    return eulerAngles;
}

Eigen::Matrix3f Geometry::quaternionToRotationMatrix(const Eigen::Quaternionf& q)
{
    // Convert quaternion to rotation matrix
    // This rotation represents the orientation of the body frame w.r.t. an inertial frame.
    // Meaning, it can left multiply a vector represented in body frame, to bring it to the inertial frame
    Eigen::Matrix3f rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;
}

Eigen::Vector3f Geometry::quat2Re3(const Eigen::Quaternionf& q)
{
    Geometry geometry;
    // Compute Re3, that is the third column of the rotation matrix (from body to inertial frame)
    Eigen::Matrix3f R = geometry.quaternionToRotationMatrix(q);
    Eigen::Vector3f Re3 = R.col(2);

    return Re3;
}

Eigen::Vector3f Geometry::quat2RTe3(const Eigen::Quaternionf& q)
{
    Geometry geometry;
    // Compute R^Te3, that is the third column of the transpose of the rotation matrix (from inertial to body frame)
    Eigen::Matrix3f R = geometry.quaternionToRotationMatrix(q);
    Eigen::Matrix3f RT = R.transpose();
    Eigen::Vector3f RTe3 = RT.col(2);

    return RTe3;
}

float Geometry::quat2R33(const Eigen::Quaternionf& q)
{
    Geometry geometry;
    // compute the (3,3)th element of the rotation matrix from quaternion
    Eigen::Matrix3f R = geometry.quaternionToRotationMatrix(q);
    float minThreshold = 1e-4;
    // protect it for very small numbers (we tend to use this value for division)
    float R33 = std::abs(R.coeff(2, 2)) < minThreshold ? minThreshold : R.coeff(2, 2);

    return R33;
}


/*
Implements eq 1 of https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
*/
Eigen::Quaternionf Geometry::angleAxisToQuaternion (const float& angle_rad, const Eigen::Vector3f vector)
{
    // define unit quaternion
    Eigen::Quaternionf quat;
    // trigonometric constants
    float ca2 = cos(angle_rad/2);
    float sa2 = sin(angle_rad/2);
    quat.w() = ca2;
    quat.x() = vector.x()*sa2;
    quat.y() = vector.y()*sa2;
    quat.z() = vector.z()*sa2;

    return quat;
}

Eigen::Quaternionf Geometry::eulerToQuaternion(float roll_deg, float pitch_deg, float yaw_deg)
{
    geometricParameters& params_geometric = geometricParameters::getInstance();
    // Convert roll, pitch, and yaw angles to radians
    float rollRad = roll_deg * params_geometric.PI / 180.0;
    float pitchRad = pitch_deg * params_geometric.PI / 180.0;
    float yawRad = yaw_deg * params_geometric.PI / 180.0;

    // Calculate half angles
    float cosRollHalf = cos(rollRad / 2.0);
    float sinRollHalf = sin(rollRad / 2.0);
    float cosPitchHalf = cos(pitchRad / 2.0);
    float sinPitchHalf = sin(pitchRad / 2.0);
    float cosYawHalf = cos(yawRad / 2.0);
    float sinYawHalf = sin(yawRad / 2.0);

    // Compute quaternion components
    float w = cosRollHalf * cosPitchHalf * cosYawHalf + sinRollHalf * sinPitchHalf * sinYawHalf;
    float x = sinRollHalf * cosPitchHalf * cosYawHalf - cosRollHalf * sinPitchHalf * sinYawHalf;
    float y = cosRollHalf * sinPitchHalf * cosYawHalf + sinRollHalf * cosPitchHalf * sinYawHalf;
    float z = cosRollHalf * cosPitchHalf * sinYawHalf - sinRollHalf * sinPitchHalf * cosYawHalf;

    // Return the quaternion
    return Eigen::Quaternionf(w, x, y, z);
};