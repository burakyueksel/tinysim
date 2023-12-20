 /**
 * @file parameters_drone.cpp
 * @brief Parameters of a drone
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */

#include "parameters.h"

// helper functions:
// compute bandwidth of the controller with a margin from actuators
double computeOmega(double actTime, double margin)
{
    return 1/(actTime*margin);
}

// compute p gain from the bandwdith of a 2nd order controller
double computeKp(double omega)
{
    return pow(omega,2);
}

// cpmpute d gain from the bandwidth and damping of a 2nd order controller
double computeKd(double omega, double xi)
{
    return 2*omega*xi;
}

// Drone aux parameters
/*
Following parameters are relevant for flight control stability.
We assume that our drone has certain number of actuators, whose
dynamics can be approximated with a first order linear system,
which can be uniquely described with a single parameter: its bandwidth.
This means we assume that the motor controller does its best to control
its motion within this bandwidth, no matter what the disturbances are.
TODO: consider adding electric motor physical model and a speed
controller for it with torque limitations.
*/
double actuatorBandWidth = 3.0;
/*
We assume then that the actuator is the fastest moving element of the drone,
compared to its translational and rotational motion.
This means higher we go in the control loop design, more margins we will leave
for stability and robustness of that loop.
Let us start with attitude and altitude loops, their reference dynamics if exists,
and go up to position control and mission planner step by step.
*/
// Attitude (rpy)
double rpErrMargin   = 1.2; // tilt error margin makes it 20% slower than actuators can achieve
double yawErrMargin  = 1.4; // make yaw slower than roll and pitch
double rpRefMargin   = rpErrMargin*1.2;    // tilt reference is slower than roll error dymamics
double yawRefMargin  = yawErrMargin*1.2;   // yaw reference is slower than yaw error dynamics
// Altitude (Z)
double altErrMargin  = 1.2; // altitude error margin makes it 20% slower than actuators can achieve
double altRefMargin  = altErrMargin * 1.2; // make reference slower than yaw 
// Position (XY)
double posErrMargin = rpRefMargin * 2.0; // make it half slow as the attitude reference dynamics.
double posRefMargin = posErrMargin * 2.0;// make it half slow as the position error dynamics.
/*
Now let us compute the control gains
*/
// alt error ctrl
double altOmega = computeOmega(actuatorBandWidth, altErrMargin); // make sure there is enough margin wrt inner loop
double altXi    = 1.0; // critically damped
// alt ref dyn
double altRefOmega = computeOmega(actuatorBandWidth, altRefMargin); // make sure there is enough margin wrt inner loop
double altRefXi    = 1.0; // critically damped
// attitude error ctrl (gains computed inline when setting the paramters according to the paper)
double omegaX = computeOmega(actuatorBandWidth,rpErrMargin);
double omegaY = computeOmega(actuatorBandWidth,rpErrMargin);
double omegaZ = computeOmega(actuatorBandWidth,yawErrMargin);
double xiX    = 1.0;
double xiY    = 1.0;
double xiZ    = 1.0;
// pos error ctrl
double posOmega = computeOmega(actuatorBandWidth, posErrMargin); // make sure there is enough margin wrt inner loop
double posXi    = 1.0; // critically damped
// pos ref dyn
double posRefOmega = computeOmega(actuatorBandWidth, posRefMargin); // make sure there is enough margin wrt inner loop
double posRefXi    = 1.0; // critically damped


droneParameters::droneParameters(): 
      droneType(DroneTypes::MC_QUAD),  // Initialize your parameters here
      mass(1.0),    // kg
      actBW(3.0),   // rad/s
      indiOmegaBW(3.0), // rad/s
      indiMuBW(3.0),    //rad/s
      imuAccStdev (0.01),
      imuGyrStdev(0.001),
      baroPressureStdev(3.0), // standard deviation of noise (in Pa)
      // example: inertiaMatrix(Eigen::Matrix3d::Identity()),
      inertiaMatrix((Eigen::Matrix3d() << 1.0, 0.0, 0.0,
                                          0.0, 2.0, 0.0,
                                          0.0, 0.0, 3.0).finished()),
      cogOffset((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished()),
      initPos(Eigen::Vector3d::Zero()),
      posCtrlRefDyn{computeKp(posRefOmega), computeKd(posRefOmega, posRefXi)}, // Kp, Kd
      posCtrlPID{pow(posOmega,2), 2*posOmega*posXi, 0.01},  // Kp, Kd, Ki
      altCtrlRefDyn{computeKp(altRefOmega), computeKd(altRefOmega, altRefXi)}, // Kp, Kd
      altCtrlPID{computeKp(altOmega), computeKd(altOmega, altXi), 0.01},  // Kp, Kd, Ki
      attCtrlTiltPrio{
        (Eigen::Matrix3d() << 2*inertiaMatrix.coeff(0,0)/(omegaX*omegaX), 0.0, 0.0,
                              0.0, 2*inertiaMatrix.coeff(1,1)/(omegaY*omegaY), 0.0,
                              0.0, 0.0, 2*inertiaMatrix.coeff(2,2)/(omegaZ*omegaZ)).finished(),
        (Eigen::Matrix3d() << 2*inertiaMatrix.coeff(0,0)*xiX/omegaX, 0.0, 0.0,
                              0.0, 2*inertiaMatrix.coeff(1,1)*xiY/omegaY, 0.0,
                              0.0, 0.0, 2*inertiaMatrix.coeff(2,2)*xiZ/omegaZ).finished()
      }
{
    // Additional initialization if needed
}

droneParameters& droneParameters::getInstance()
{
    static droneParameters instance;
    return instance;
}