 /**
 * @file posPIDCtrl.h
 * @brief Simple and basic PID Controller for the horizontal position and velocity channel.
 *        2nd order reference dynamics and error controller for PID.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-20
 */
#include "controls.h"
#include "parameters.h"

// Position Ref Dynamics
// Note: both ref and err ctrl are defined under Control class, so we can reach the private globals.
posCtrlRefStates Control::posControlRefDyn(horizontalStates posCmd, double timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance();
    double Kp = params_drone.posCtrlRefDyn.Kp;
    double Kd = params_drone.posCtrlRefDyn.Kd;

    // Compute the control input (acceleration)
    double posErrorX = posCmd.x - g_posCtrlRefDynStates.posRef_m.x;
    double posErrorY = posCmd.y - g_posCtrlRefDynStates.posRef_m.y;

    // reference velocity to step position command is not given, hence 0.
    g_posCtrlRefDynStates.accRef_mps2.x = posErrorX *  Kp - Kd * g_posCtrlRefDynStates.velRef_mps.x;
    g_posCtrlRefDynStates.accRef_mps2.y = posErrorY *  Kp - Kd * g_posCtrlRefDynStates.velRef_mps.y;
    // TODO: add acc limits
    // integrate to velocity now
    g_posCtrlRefDynStates.velRef_mps.x = g_posCtrlRefDynStates.velRef_mps.x + g_posCtrlRefDynStates.accRef_mps2.x * timeStep_s;
    g_posCtrlRefDynStates.velRef_mps.y = g_posCtrlRefDynStates.velRef_mps.y + g_posCtrlRefDynStates.accRef_mps2.y * timeStep_s;
    // TODO: add vel limits
    // integrate to position now
    g_posCtrlRefDynStates.posRef_m.x = g_posCtrlRefDynStates.posRef_m.x + g_posCtrlRefDynStates.velRef_mps.x * timeStep_s;
    g_posCtrlRefDynStates.posRef_m.y = g_posCtrlRefDynStates.posRef_m.y + g_posCtrlRefDynStates.velRef_mps.y * timeStep_s;

    return g_posCtrlRefDynStates;
}


// Position Error Dynamics
horizontalStates Control::posCtrlErr(posCtrlRefStates posRefStates, Eigen::Vector3d position, Eigen::Vector3d velocity, double timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance();

    // error
    double errorX = posRefStates.posRef_m.x - position[0];
    double errorY = posRefStates.posRef_m.x - position[1];

    // d_error
    double derrorX = posRefStates.velRef_mps.x - velocity[0];
    double derrorY = posRefStates.velRef_mps.y - velocity[1];

    // Proportional term
    double proportionalX = params_drone.posCtrlPID.Kp * errorX;
    double proportionalY = params_drone.posCtrlPID.Kp * errorY;

    // Integral term
    g_horizontalPosIntegral.x += params_drone.posCtrlPID.Ki * errorX * timeStep_s;
    g_horizontalPosIntegral.y += params_drone.posCtrlPID.Ki * errorY * timeStep_s;

    // todo: add proper anti-windup

    // Derivative term
    double derivativeX = params_drone.posCtrlPID.Kd * derrorX;
    double derivativeY = params_drone.posCtrlPID.Kd * derrorY;

    // Calculate the desired accelerations in x and y axis
    horizontalStates accXYRef;
    accXYRef.x = posRefStates.accRef_mps2.x + proportionalX + g_horizontalPosIntegral.x + derivativeX;
    accXYRef.y = posRefStates.accRef_mps2.y + proportionalY + g_horizontalPosIntegral.y + derivativeY;

    return accXYRef;
}

horizontalStates Control::posPidControl(horizontalStates posCmd, Eigen::Vector3d position, Eigen::Vector3d velocity, double timeStep_s)
{
    Control ctrl;
    // step command in height. Pass it through the 2nd order reference dynamics for smooth trajectories
    posCtrlRefStates posRefStates = posControlRefDyn(posCmd, timeStep_s);
    // follow the trajectories with a PID
    horizontalStates accXYRef = posCtrlErr(posRefStates, position, velocity, timeStep_s);

    return accXYRef;
}