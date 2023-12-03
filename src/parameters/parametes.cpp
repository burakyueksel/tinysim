
#include "parameters.h"

// Define control parameters
// pos error ctrl
double posOmega = 1.0; // make sure there is enough margin wrt inner loop
double posXi    = 1.0; // critically damped
// alt error ctrl
double altOmega = 3.0; // make sure there is enough margin wrt inner loop
double altXi    = 1.0; // critically damped
// attitude error ctrl
double timeConst_X  = 0.3;
double timeConst_Y  = 0.3;
double timeConst_Z  = 0.3;
double damping_X    = 1.0;
double damping_Y    = 1.0;
double damping_Z    = 1.0;

Parameters parameters
{
    // Load the drone parameters from a file or any other source
    // type
    .droneType = DroneTypes::MC_QUAD,
    // physical properties
    .mass = 1.0, // Set the mass for drone 1
    /*.inertiaMatrix = Eigen::Matrix3d(
                        1.0, 0.0, 0.0,
                        0.0, 2.0, 0.0,
                        0.0, 0.0, 3.0),
    // init states
    .initPos = Eigen::Vector3d ( 0.0, 0.0, 0.0),
    // pos ctrl ref dyn
    .posCtrlRefDyn =
    {
        .timeConst = 0.6, // make sure there is enough margin wrt inner loop
        .damping   = 1.0  // critically damped
    },
    // pos ctrl error
    .posCtrlPID =
    {
        .Kp = pow(posOmega,2),
        .Kd = 2*posOmega*posXi,
        .Ki = 0.1
    },
    // alt ctrl ref dyn
    .altCtrlRefDyn = 
    {
        .timeConst = 0.8,
        .damping   = 1.0
    },
    // alt pid ctrl
    .altCtrlPID =
    {
        .Kp = pow(altOmega,2),
        .Kd = 2*altOmega*altXi,
        .Ki = 0.1
    },
    // att tilt prio ctrl
    .attCtrlTiltPrio =
    {
        .KP = Eigen::Matrix3d(
                              2*parameters.inertiaMatrix.coeff(0,0)/(timeConst_X*timeConst_X), 0.0, 0.0,
                              0.0, 2*parameters.inertiaMatrix.coeff(1,1)/(timeConst_Y*timeConst_Y), 0.0,
                              0.0, 0.0, 2*parameters.inertiaMatrix.coeff(2,2)/(timeConst_Z*timeConst_Z)),
        .KD = Eigen::Matrix3d(
                              2*parameters.inertiaMatrix.coeff(0,0)*damping_X/timeConst_X, 0.0, 0.0,
                              0.0, 2*parameters.inertiaMatrix.coeff(1,1)*damping_Y/timeConst_Y, 0.0,
                              0.0, 0.0, 2*parameters.inertiaMatrix.coeff(2,2)*damping_Z/timeConst_Z)
    }
    */
};