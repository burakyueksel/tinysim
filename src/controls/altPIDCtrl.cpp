#include "controls.h"
#include "parameters.h"
#include "physics.h"

// Altitude Ref Dynamics

altCtrlRefStates Control::altControlRefDyn(double zCmd, double timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance();
    // Compute the control input (acceleration)
    double error = zCmd - g_altCtrlRefDynStates.posRef;
    double timeConst = params_drone.altCtrlRefDyn.timeConst;
    double damping = params_drone.altCtrlRefDyn.damping;
    double accNow = error *  timeConst * timeConst - 2.0 * damping * timeConst * g_altCtrlRefDynStates.velRef;
    // TODO: add acc limits
    g_altCtrlRefDynStates.accRef = accNow;
    // integrate to velocity now
    g_altCtrlRefDynStates.velRef = g_altCtrlRefDynStates.velRef + g_altCtrlRefDynStates.accRef * timeStep_s;
    //TODO: add vel limits
    // integrate to position now
    g_altCtrlRefDynStates.posRef = g_altCtrlRefDynStates.posRef + g_altCtrlRefDynStates.velRef*timeStep_s;

    return g_altCtrlRefDynStates;
}


//  Altitude PID control
altCtrlErrOutputs Control::altPidControl(double zDes_m, double z_m, double dzDes_mps, double dz_mps, Eigen::Quaterniond quaternion, double timeStep_s)
{
    droneParameters& params_drone = droneParameters::getInstance(); // get drone parameters
    physicsParameters& params_phy = physicsParameters::getInstance(); // get physics/environment parameters
    RigidPhysics physics; // be able to use the functions defined under physics. TODO: Move trigonometric fcns to math or geometry folder.

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
    double R33 = physics.quat2R33(quaternion);
    outputs.accCmd_mps2     =   (params_phy.gravity + proportional + g_altPIDCtrlIntegral + derivative) / R33;
    outputs.controlThrust_N =   params_drone.mass * outputs.accCmd_mps2;

    // Following lines implements generic PID, which will perform very well if you stick to the parametrization I gave in parameter.cpp.
    //double controlThrust_N =   proportional + g_altIntegral + derivative;

    return outputs;
}