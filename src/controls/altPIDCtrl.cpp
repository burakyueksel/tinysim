 /**
 * @file altPIDCtrl.h
 * @brief Simple and basic PID Controller for the altitude channel.
 *        2nd order reference dynamics and error controller for PID.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-03
 */
#include "controls.h"
#include "parameters.h"
#include "physics.h"
#include "geometry.h"

// Altitude Ref Dynamics
// Note: both ref and err ctrl are defined under Control class, so we can reach the private globals.
altCtrlRefStates Control::altControlRefDyn(float zCmd_m, float timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance();
    // Compute the control input (acceleration)
    float error = zCmd_m - g_altCtrlRefDynStates.zRef_m;
    float Kp = params_drone.altCtrlRefDyn.Kp;
    float Kd = params_drone.altCtrlRefDyn.Kd;
    // reference velocity to step position command is not given, hence 0.
    float accNow = Kp * error  - Kd * g_altCtrlRefDynStates.dzRef_mps;
    // TODO: add acc limits
    g_altCtrlRefDynStates.ddzRef_mps2 = accNow;
    // integrate to velocity now
    g_altCtrlRefDynStates.dzRef_mps = g_altCtrlRefDynStates.dzRef_mps + g_altCtrlRefDynStates.ddzRef_mps2 * timeStep_s;
    //TODO: add vel limits
    // integrate to position now
    g_altCtrlRefDynStates.zRef_m = g_altCtrlRefDynStates.zRef_m + g_altCtrlRefDynStates.dzRef_mps*timeStep_s;

    return g_altCtrlRefDynStates;
}


//  Altitude PID control
altCtrlErrOutputs Control::altPidErrControl(float zDes_m, float z_m, float dzDes_mps, float dz_mps, Eigen::Quaternionf quaternion, float timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance(); // get drone parameters
    physicsParameters& params_phy = physicsParameters::getInstance(); // get physics/environment parameters
    Geometry geometry;

    altCtrlErrOutputs outputs;
    // error
    float error = zDes_m - z_m;

    // d_error
    float d_error = dzDes_mps - dz_mps;

    // Proportional term
    float proportional = params_drone.altCtrlPID.Kp * error;

    // Integral term
    g_altPIDCtrlIntegral += params_drone.altCtrlPID.Ki * error * timeStep_s;

    // todo: add proper anti-windup

    // Derivative term
    float derivative = params_drone.altCtrlPID.Kd * d_error;

    // Calculate the thrust for height control
    // Following lines will implement the correct thrust computation (assumption: thrust aligned with the body z axis)
    float R33 = geometry.quat2R33(quaternion);
    outputs.accCmd_mps2     =   (params_phy.gravity_mps2 + proportional + g_altPIDCtrlIntegral + derivative) / R33;
    outputs.controlThrust_N =   params_drone.mass_kg * outputs.accCmd_mps2;

    // Following lines implements generic PID, which will perform very well if you stick to the parametrization I gave in parameter.cpp.
    //float controlThrust_N =   proportional + g_altIntegral + derivative;

    return outputs;
}

altCtrlErrOutputs Control::altPidControl(float zCmd_m, float z_m, float dz_mps, Eigen::Quaternionf quaternion, float timeStep_s)
{
    Control ctrl;
    // step command in height. Pass it through the 2nd order reference dynamics for smooth trajectories
    altCtrlRefStates altRefStates = altControlRefDyn(zCmd_m, timeStep_s);
    // follow the trajectories with a PID
    altCtrlErrOutputs altCtrlOutputs = altPidErrControl(altRefStates.zRef_m, z_m, altRefStates.dzRef_mps, dz_mps, quaternion, timeStep_s);

    return altCtrlOutputs;
}