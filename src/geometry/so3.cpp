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

Eigen::Vector3d Geometry::quaternionToEulerAngles(const Eigen::Quaterniond& q)
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

Eigen::Matrix3d Geometry::quaternionToRotationMatrix(const Eigen::Quaterniond& q)
{
    // Convert quaternion to rotation matrix
    // This rotation represents the orientation of the body frame w.r.t. an inertial frame.
    // Meaning, it can left multiply a vector represented in body frame, to bring it to the inertial frame
    Eigen::Matrix3d rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;
}

Eigen::Vector3d Geometry::quat2Re3(const Eigen::Quaterniond& q)
{
    Geometry geometry;
    // Compute Re3, that is the third column of the rotation matrix (from body to inertial frame)
    Eigen::Matrix3d R = geometry.quaternionToRotationMatrix(q);
    Eigen::Vector3d Re3 = R.col(2);

    return Re3;
}

Eigen::Vector3d Geometry::quat2RTe3(const Eigen::Quaterniond& q)
{
    Geometry geometry;
    // Compute R^Te3, that is the third column of the transpose of the rotation matrix (from inertial to body frame)
    Eigen::Matrix3d R = geometry.quaternionToRotationMatrix(q);
    Eigen::Matrix3d RT = R.transpose();
    Eigen::Vector3d RTe3 = RT.col(2);

    return RTe3;
}

double Geometry::quat2R33(const Eigen::Quaterniond& q)
{
    Geometry geometry;
    // compute the (3,3)th element of the rotation matrix from quaternion
    Eigen::Matrix3d R = geometry.quaternionToRotationMatrix(q);
    double minThreshold = 1e-4;
    // protect it for very small numbers (we tend to use this value for division)
    double R33 = std::abs(R.coeff(2, 2)) < minThreshold ? minThreshold : R.coeff(2, 2);

    return R33;
}


/*
Implements eq 1 of https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
*/
Eigen::Quaterniond Geometry::angleAxisToQuaternion (const double& angle, const Eigen::Vector3d vector)
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

Eigen::Quaterniond Geometry::eulerToQuaternion(double roll_deg, double pitch_deg, double yaw_deg)
{
    geometricParameters& params_geometric = geometricParameters::getInstance();
    // Convert roll, pitch, and yaw angles to radians
    double rollRad = roll_deg * params_geometric.PI / 180.0;
    double pitchRad = pitch_deg * params_geometric.PI / 180.0;
    double yawRad = yaw_deg * params_geometric.PI / 180.0;

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