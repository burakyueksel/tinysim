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
    float x;
    float y;
};

struct posCtrlRefStates
{
    horizontalStates posRef_m;
    horizontalStates velRef_mps;
    horizontalStates accRef_mps2;
};

struct altCtrlRefStates
{
    float zRef_m;
    float dzRef_mps;
    float ddzRef_mps2;
};

struct altCtrlErrOutputs
{
    float controlThrust_N;
    float accCmd_mps2;
};

struct altCtrl
{
    altCtrlRefStates altRef;
    altCtrlErrOutputs altOut;
};

class Control
{
public:
    /**
     * @brief Control functions
     */
    Control();  // Constructor to initialize the object
    // alt ctrl
    altCtrlRefStates altControlRefDyn(float zCmd_m, float timeStep_s);
    altCtrlErrOutputs altPidErrControl(float zDes_m, float z_m, float dzDes_mps, float dz_mps, Eigen::Quaternionf quaternion, float timeStep_s);
    altCtrl altPidControl(float zCmd_m, float z_m, float dz_mps, Eigen::Quaternionf quaternion, float timeStep_s);
    // att ctrl
    Eigen::Vector3f attRateIndiCtrl(Eigen::Vector3f omega_rps, Eigen::Vector3f domega_des_rps, Eigen::Vector3f mu_Nm, float timeStep_s);
    Eigen::Vector3f attTiltPrioControl(Eigen::Quaternionf quatDes, Eigen::Quaternionf quaternion, Eigen::Vector3f angVelDes_rps, Eigen::Vector3f angVel_rps, Eigen::Vector3f angVelDotEst_rps2);
    // pos ctrl
    posCtrlRefStates posControlRefDyn(horizontalStates posCmd_m, float timeStep_s);
    horizontalStates posCtrlErr(posCtrlRefStates posRefStates, Eigen::Vector3f position_m, Eigen::Vector3f velocity_mps, float timeStep_s);
    horizontalStates posPidControl(horizontalStates posCmd_m, Eigen::Vector3f position_m, Eigen::Vector3f velocity_mps, float timeStep_s);
    /**
     * @brief Utility functions
     */
    // Signum function template
    template <typename T>
    int mySignum(T value) {
        return (value > T(0)) - (value < T(0));
    }
private:
    float g_altPIDCtrlIntegral;
    horizontalStates g_horizontalPosIntegral;
    posCtrlRefStates g_posCtrlRefDynStates;
    altCtrlRefStates g_altCtrlRefDynStates;
};