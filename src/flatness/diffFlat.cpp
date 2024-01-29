 /**
 * @file diffFlat.cpp
 * @brief Differential flatness functions.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-01-28
 */
#include "flatness.h"
#include "parameters.h"

// Thrust vector in inertial frame.
Eigen::Vector3f get_t_I(Eigen::Vector3f acc_I)
{
    physicsParameters& params_phy = physicsParameters::getInstance(); // get physics/environment parameters

    Eigen::Vector3f thrustVect_I (acc_I.x, acc_I.y, acc_I.z - params_phy.gravity_mps2);
    return thrustVect_I;
}

Eigen::Vector3f get_x_I_C(float psi_rad)
{
    Eigen::Vector3f x_I_C (cos(psi_rad), sin(psi_rad), 0.0f);
    return x_I_C;
}

Eigen::Vector3f get_y_I_C(float psi_rad)
{
    Eigen::Vector3f y_I_C (-sin(psi_rad), cos(psi_rad), 0.0f);
    return y_I_C;
}

Eigen::Vector3f get_x_I_B(Eigen::Vector3f acc_I, float psi_rad)
{
    Eigen::Vector3f x_I_C = get_x_I_C(psi_rad);
    Eigen::Vector3f y_I_C = get_y_I_C(psi_rad);
    Eigen::Vector3f t_I   = get_t_I(acc_I, psi_rad);

    Eigen::Vector3f txy   = t_I.cross(y_I_C);   // x_B is defined by the cross product of t and y_C
    float norm_txy = txy.norm();    // compute its norm
    norm_txy = (norm_txy < 1e-6) ? 1e-6 : norm_txy; // protect division by small number
    Eigen::Vector3f x_I_B = txy/norm_txy;

    return x_I_B;
}

Eigen::Vector3f get_y_I_B(Eigen::Vector3f acc_I, float psi_rad)
{
    Eigen::Vector3f x_I_B   = get_x_I_B(acc_I, psi_rad);
    Eigen::Vector3f t_I   = get_t_I(acc_I, psi_rad);

    Eigen::Vector3f xxt   = x_I_B.cross(t_I);   // y_B is defined by the cross product of x_B and t
    float norm_xxt = xxt.norm();    // compute its norm
    norm_xxt = (norm_xxt < 1e-6) ? 1e-6 : norm_xxt; // protect division by small number
    Eigen::Vector3f y_I_B = xxt/norm_xxt;

    return y_I_B;
}

Eigen::Vector3f get_z_I_B(Eigen::Vector3f acc_I, float psi_rad)
{
    Eigen::Vector3f x_I_B   = get_x_I_B(acc_I, psi_rad);
    Eigen::Vector3f y_I_B   = get_y_I_B(acc_I, psi_rad);

    Eigen::Vector3f z_I_B   = x_I_B.cross(y_I_B);   // x is defined by the cross product of x_B and y_B

    return z_I_B;
}

Eigen::Matrix3f getR_I_B (Eigen::Vector3f acc_I, float psi_rad)
{
    Eigen::Matrix3f R_I_B;
    Eigen::Vector3f x_I_B   = get_x_I_B(acc_I, psi_rad);
    Eigen::Vector3f y_I_B   = get_y_I_B(acc_I, psi_rad);
    Eigen::Vector3f z_I_B   = get_z_I_B(acc_I, psi_rad);

    R_I_B << x_I_B, y_I_B, z_I_B;

    return R_I_B;
}

float getThrust_B (Eigen::Vector3f acc_I, float psi_rad)
{
    droneParameters& params_drone = droneParameters::getInstance(); // get drone parameters
    Eigen::Vector3f z_I_B   = get_z_I_B(acc_I, psi_rad);
    Eigen::Vector3f t_I   = get_t_I(acc_I, psi_rad);

    float thurst_B = -(params_drone.mass_kg * z_I_B).transpose() * t_I;

    return thurst_B;
}

float getcollectiveThrust_B (Eigen::Vector3f acc_I, float psi_rad)
{
    droneParameters& params_drone = droneParameters::getInstance(); // get drone parameters
    float collectiveThrust_B = getThrust_B (Eigen::Vector3f acc_I, float psi_rad) / params_drone.mass_kg;

    return collectiveThrust_B;
}

Eigen::Vector3d getOmega_B (Eigen::Vector3f acc_I, Eigen::Vector3f jerk_I, float psi_rad, float psi_dot_rps)
{
    Eigen::Vector3f x_I_B   = get_x_I_B(acc_I, psi_rad);
    Eigen::Vector3f y_I_B   = get_y_I_B(acc_I, psi_rad);
    Eigen::Vector3f z_I_B   = get_z_I_B(acc_I, psi_rad);

    float c = getcollectiveThrust_B;

    Eigen::Vector3f x_I_C = get_x_I_C(float psi_rad);
    Eigen::Vector3f y_I_C = get_y_I_C(float psi_rad);
    Eigen::Vector3f yxz   = y_I_C.cross(z_I_B);   // y_I_C x z_I_B
    float norm_yxz = yxz.norm();    // compute its norm
    norm_yxz = (norm_yxz < 1e-6) ? 1e-6 : norm_yxz; // protect division by small number

    float omega_x = y_I_B.transpose() * jerk_I / c;
    float omega_y = -x_I_B.transpose() * jerk_I / c;
    float omega_z = (psi_dot_rps*x_I_C.transpose()*x_I_B + omega_y*y_I_C.transpose()*z_I_B)/norm_yxz;

    Eigen::Vector3f omega_B (mega_x, omega_y, omega_z);
    return omega_B;
}