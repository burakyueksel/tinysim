// parameters.cpp
// parameters.cpp

#include "parameters.h"

// Drone aux parameters
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

droneParameters::droneParameters(): 
      droneType(DroneTypes::MC_QUAD),  // Initialize your parameters here
      mass(1.0),
      // example: inertiaMatrix(Eigen::Matrix3d::Identity()),
      inertiaMatrix((Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                          0.0, 2.0, 0.0,
                                          0.0, 0.0, 3.0).finished()),
      cogOffset((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished()),
      initPos(Eigen::Vector3d::Zero()),
      posCtrlRefDyn{0.6, 1.0},  // time const, damping
      posCtrlPID{pow(posOmega,2), 2*posOmega*posXi, 0.01},  // Kp, Kd, Ki
      altCtrlRefDyn{0.6, 1.0}, // time const, damping
      altCtrlPID{pow(altOmega,2), 2*altOmega*altXi, 0.01},  // Kp, Kd, Ki
      attCtrlTiltPrio{
        (Eigen::Matrix3d() << 2*inertiaMatrix.coeff(0,0)/(timeConst_X*timeConst_X), 0.0, 0.0,
                              0.0, 2*inertiaMatrix.coeff(1,1)/(timeConst_Y*timeConst_Y), 0.0,
                              0.0, 0.0, 2*inertiaMatrix.coeff(2,2)/(timeConst_Z*timeConst_Z)).finished(),
        (Eigen::Matrix3d() << 2*inertiaMatrix.coeff(0,0)*damping_X/timeConst_X, 0.0, 0.0,
                              0.0, 2*inertiaMatrix.coeff(1,1)*damping_Y/timeConst_Y, 0.0,
                              0.0, 0.0, 2*inertiaMatrix.coeff(2,2)*damping_Z/timeConst_Z).finished()
      }
{
    // Additional initialization if needed
}

droneParameters& droneParameters::getInstance()
{
    static droneParameters instance;
    return instance;
}