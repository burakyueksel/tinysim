 /**
 * @file parameters_drone.cpp
 * @brief Parameters of a drone (control, actuator, weight and balance, sensor data)
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */

#include "parameters.h"

// helper functions:
// compute bandwidth of the controller with a margin from actuators
float computeOmega(float actTime, float margin)
{
    return 1/(actTime*margin);
}

// compute p gain from the bandwdith of a 2nd order controller
float computeKp(float omega)
{
    return pow(omega,2);
}

// cpmpute d gain from the bandwidth and damping of a 2nd order controller
float computeKd(float omega, float xi)
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
float actuatorBandWidth = 3.0;
/*
We assume then that the actuator is the fastest moving element of the drone,
compared to its translational and rotational motion.
This means higher we go in the control loop design, more margins we will leave
for stability and robustness of that loop.
Let us start with attitude and altitude loops, their reference dynamics if exists,
and go up to position control and mission planner step by step.
*/
// Attitude (rpy)
float rpErrMargin   = 1.2; // tilt error margin makes it 20% slower than actuators can achieve
float yawErrMargin  = 1.4; // make yaw slower than roll and pitch
float rpRefMargin   = rpErrMargin*1.2;    // tilt reference is slower than roll error dymamics
float yawRefMargin  = yawErrMargin*1.2;   // yaw reference is slower than yaw error dynamics
// Altitude (Z)
float altErrMargin  = 1.2; // altitude error margin makes it 20% slower than actuators can achieve
float altRefMargin  = altErrMargin * 1.2; // make reference slower than yaw 
// Position (XY)
float posErrMargin = rpRefMargin * 2.0; // make it half slow as the attitude reference dynamics.
float posRefMargin = posErrMargin * 2.0;// make it half slow as the position error dynamics.
/*
Now let us compute the control gains
*/
// alt error ctrl
float altOmega = computeOmega(actuatorBandWidth, altErrMargin); // make sure there is enough margin wrt inner loop
float altXi    = 1.0; // critically damped
// alt ref dyn
float altRefOmega = computeOmega(actuatorBandWidth, altRefMargin); // make sure there is enough margin wrt inner loop
float altRefXi    = 1.0; // critically damped
// attitude error ctrl (gains computed inline when setting the paramters according to the paper)
float omegaX = computeOmega(actuatorBandWidth,rpErrMargin);
float omegaY = computeOmega(actuatorBandWidth,rpErrMargin);
float omegaZ = computeOmega(actuatorBandWidth,yawErrMargin);
float xiX    = 1.0;
float xiY    = 1.0;
float xiZ    = 1.0;
// pos error ctrl
float posOmega = computeOmega(actuatorBandWidth, posErrMargin); // make sure there is enough margin wrt inner loop
float posXi    = 1.0; // critically damped
// pos ref dyn
float posRefOmega = computeOmega(actuatorBandWidth, posRefMargin); // make sure there is enough margin wrt inner loop
float posRefXi    = 1.0; // critically damped


droneParameters::droneParameters(): 
      droneType(DroneTypes::MC_QUAD),  // Initialize your parameters here
      mass_kg(1.0),    // kg
      actBW_rps(3.0),   // rad/s
      indiOmegaBW_rps(3.0), // rad/s
      indiMuBW_rps(3.0),    //rad/s
      imuAccStdev (0.01),
      imuGyrStdev(0.001),
      baroPressureStdev(3.0), // standard deviation of noise (in Pa)
      // example: inertiaMatrix_kgm2(Eigen::Matrix3f::Identity()),
      inertiaMatrix_kgm2((Eigen::Matrix3f() << 1.0, 0.0, 0.0,
                                          0.0, 2.0, 0.0,
                                          0.0, 0.0, 3.0).finished()),
      cogOffset((Eigen::Vector3f() << 0.0, 0.0, 0.0).finished()),
      initPos(Eigen::Vector3f::Zero()),
      posCtrlRefDyn{computeKp(posRefOmega), computeKd(posRefOmega, posRefXi)}, // Kp, Kd
      posCtrlPID{computeKp(posOmega), computeKd(posOmega, posXi), 0.01},  // Kp, Kd, Ki
      altCtrlRefDyn{computeKp(altRefOmega), computeKd(altRefOmega, altRefXi)}, // Kp, Kd
      altCtrlPID{computeKp(altOmega), computeKd(altOmega, altXi), 0.01},  // Kp, Kd, Ki
      attCtrlTiltPrio{
        (Eigen::Matrix3f() << 2*inertiaMatrix_kgm2.coeff(0,0)/(omegaX*omegaX), 0.0, 0.0,
                              0.0, 2*inertiaMatrix_kgm2.coeff(1,1)/(omegaY*omegaY), 0.0,
                              0.0, 0.0, 2*inertiaMatrix_kgm2.coeff(2,2)/(omegaZ*omegaZ)).finished(),
        (Eigen::Matrix3f() << 2*inertiaMatrix_kgm2.coeff(0,0)*xiX/omegaX, 0.0, 0.0,
                              0.0, 2*inertiaMatrix_kgm2.coeff(1,1)*xiY/omegaY, 0.0,
                              0.0, 0.0, 2*inertiaMatrix_kgm2.coeff(2,2)*xiZ/omegaZ).finished()
      }
{
    // Additional initialization if needed
}

droneParameters& droneParameters::getInstance()
{
    static droneParameters instance;
    return instance;
}