 /**
 * @file controls.h
 * @brief Control library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

struct horizontalStates
{
    double x;
    double y;
};

struct posCtrlRefStates
{
    horizontalStates posRef;
    horizontalStates velRef;
    horizontalStates accRef;
};

struct altCtrlRefStates
{
    double zRef;
    double dzRef;
    double ddzRef;
};

struct altCtrlErrOutputs
{
    double controlThrust_N;
    double accCmd_mps2;
};

class Control
{
public:
    /**
     * @brief Control functions
     */
    altCtrlRefStates altControlRefDyn(double zCmd, double timeStep_s);
    altCtrlErrOutputs altPidErrControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, Eigen::Quaterniond quaternion, double timeStep_s);
    altCtrlErrOutputs altPidControl(double zCmd, double z, double dz, Eigen::Quaterniond quaternion, double timeStep_s);
    Eigen::Vector3d attRateIndiCtrl(Eigen::Vector3d omega_rps, Eigen::Vector3d domega_des_rps, Eigen::Vector3d mu_Nm, double timeStep_s);

    // tilt priorizing quaternion based attitude controller
    Eigen::Vector3d attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps);

    /**
     * @brief Utility functions
     */
    // Signum function template
    template <typename T>
    int mySignum(T value) {
        return (value > T(0)) - (value < T(0));
    }
private:
    double g_altPIDCtrlIntegral;
    horizontalStates g_horizontalPosIntegral;
    posCtrlRefStates g_posCtrlRefDynStates;
    altCtrlRefStates g_altCtrlRefDynStates;
};