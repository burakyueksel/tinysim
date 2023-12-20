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
altCtrlRefStates Control::altControlRefDyn(double zCmd, double timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance();
    // Compute the control input (acceleration)
    double error = zCmd - g_altCtrlRefDynStates.zRef;
    double Kp = params_drone.altCtrlRefDyn.Kp;
    double Kd = params_drone.altCtrlRefDyn.Kd;
    // reference velocity to step position command is not given, hence 0.
    double accNow = Kp * error  - Kd * g_altCtrlRefDynStates.dzRef;
    // TODO: add acc limits
    g_altCtrlRefDynStates.ddzRef = accNow;
    // integrate to velocity now
    g_altCtrlRefDynStates.dzRef = g_altCtrlRefDynStates.dzRef + g_altCtrlRefDynStates.ddzRef * timeStep_s;
    //TODO: add vel limits
    // integrate to position now
    g_altCtrlRefDynStates.zRef = g_altCtrlRefDynStates.zRef + g_altCtrlRefDynStates.dzRef*timeStep_s;

    return g_altCtrlRefDynStates;
}


//  Altitude PID control
altCtrlErrOutputs Control::altPidErrControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, Eigen::Quaterniond quaternion, double timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance(); // get drone parameters
    physicsParameters& params_phy = physicsParameters::getInstance(); // get physics/environment parameters
    Geometry geometry;

    altCtrlErrOutputs outputs;
    // error
    double error = zDes_m - z_m;

    // d_error
    double d_error = dzDes_mps - dz_mps;

    // Proportional term
    double proportional = params_drone.altCtrlPID.Kp * error;

    // Integral term
    g_altPIDCtrlIntegral += params_drone.altCtrlPID.Ki * error * timeStep_s;

    // todo: add proper anti-windup

    // Derivative term
    double derivative = params_drone.altCtrlPID.Kd * d_error;

    // Calculate the thrust for height control
    // Following lines will implement the correct thrust computation (assumption: thrust aligned with the body z axis)
    double R33 = geometry.quat2R33(quaternion);
    outputs.accCmd_mps2     =   (params_phy.gravity + proportional + g_altPIDCtrlIntegral + derivative) / R33;
    outputs.controlThrust_N =   params_drone.mass * outputs.accCmd_mps2;

    // Following lines implements generic PID, which will perform very well if you stick to the parametrization I gave in parameter.cpp.
    //double controlThrust_N =   proportional + g_altIntegral + derivative;

    return outputs;
}

altCtrlErrOutputs Control::altPidControl(double zCmd, double z, double dz, Eigen::Quaterniond quaternion, double timeStep_s)
{
    Control ctrl;
    // step command in height. Pass it through the 2nd order reference dynamics for smooth trajectories
    altCtrlRefStates altRefStates = altControlRefDyn(zCmd, timeStep_s);
    // follow the trajectories with a PID
    altCtrlErrOutputs altCtrlOutputs = altPidErrControl(altRefStates.zRef, z, altRefStates.dzRef, dz, quaternion, timeStep_s);

    return altCtrlOutputs;
}