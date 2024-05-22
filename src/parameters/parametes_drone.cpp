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
which can be uniquely described with a single parameter: its "3dB" bandwidth.

Rise time and 3 dB bandwidth are two closely-related parameters used to describe the limit of a system's ability
to respond to abrupt changes in an input signal. Rise time and 3 dB bandwidth are inversely proportional,
with a proportionality constant of ~0.35 when the system's response resembles that of an RC low-pass filter.
See: https://www.thorlabs.com/newgrouppage9.cfm?objectgroup_id=9817

This means:

actuatorRiseTime_s = 0.35/actuatorBandwidth_Hz


So let us say that actuator BW in 3dB is actuatorBandwidth_rps, where rps is rad per sec. Then:

actuatorBandwidth_Hz = actuatorBandwidth_rps/(2*pi)

Then the rise time and time constants are:

actuatorRiseTime_s = 0.35/actuatorBandwidth_Hz
actuatorTimeConst_s= actuatorRiseTime_s/2.2

assuming it is a first order system where the initial 10 percents add very little to the total time. See: https://en.wikipedia.org/wiki/Rise_time

*/
float actuatorBandwidth_rps = 20;
float actuatorBandwidth_Hz  = actuatorBandwidth_rps/(2*3.14);
float actuatorRiseTime_s    = 0.35/actuatorBandwidth_Hz;
float actuatorTimeConst_s   = actuatorRiseTime_s/2.2;
/*
We assume then that the actuator is the fastest moving element of the drone,
compared to its translational and rotational motion.
This means higher we go in the control loop design, more margins we will leave
for stability and robustness of that loop.
Let us start with attitude and altitude loops, their reference dynamics if exists,
and go up to position control and mission planner step by step.

NOTICE: that actuatorTimeConst_s shall match to the mass/moment of inertia of the aircraft.
Otherwise cannot be imagined, because actuators need to be fast enough to counteract
the accelerations achieved by the physical object.

TODO: consider adding electric motor physical model and a speed
controller for it with torque limitations. Right now we assume that motors have perfect
first order linear system behavior (which holds well for small motors, but not always for the big ones).
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
float altOmega = computeOmega(actuatorTimeConst_s, altErrMargin); // make sure there is enough margin wrt inner loop
float altXi    = 1.0; // critically damped
// alt ref dyn
float altRefOmega = computeOmega(actuatorTimeConst_s, altRefMargin); // make sure there is enough margin wrt inner loop
float altRefXi    = 1.0; // critically damped
// attitude error ctrl (gains computed inline when setting the paramters according to the paper)
float omegaX = computeOmega(actuatorTimeConst_s,rpErrMargin);
float omegaY = computeOmega(actuatorTimeConst_s,rpErrMargin);
float omegaZ = computeOmega(actuatorTimeConst_s,yawErrMargin);
float xiX    = 1.0;
float xiY    = 1.0;
float xiZ    = 1.0;
// pos error ctrl
float posOmega = computeOmega(actuatorTimeConst_s, posErrMargin); // make sure there is enough margin wrt inner loop
float posXi    = 1.0; // critically damped
// pos ref dyn
float posRefOmega = computeOmega(actuatorTimeConst_s, posRefMargin); // make sure there is enough margin wrt inner loop
float posRefXi    = 1.0; // critically damped


droneParameters::droneParameters(): 
      droneType(DroneTypes::MC_QUAD),  // Initialize your parameters here
      mass_kg(1.0),    // kg
      actTime_s(actuatorTimeConst_s),   // s
      indiOmegaBW_rps(3.0), // rad/s
      indiMuBW_rps(3.0),    //rad/s
      imuAccStdev (0.01),
      imuGyrStdev(0.001),
      baroPressureStdev(3.0), // standard deviation of noise (in Pa)
      // example: inertiaMatrix_kgm2(Eigen::Matrix3f::Identity()),
      inertiaMatrix_kgm2((Eigen::Matrix3f() << 1.0, 0.0, 0.0,
                                          0.0, 2.0, 0.0,
                                          0.0, 0.0, 3.0).finished()),
      cogOffset_m((Eigen::Vector3f() << 0.0, 0.0, 0.0).finished()),
      initPos_m(Eigen::Vector3f::Zero()),
      posCtrlRefDyn{computeKp(posRefOmega), computeKd(posRefOmega, posRefXi)}, // Kp, Kd
      posCtrlPID{computeKp(posOmega), computeKd(posOmega, posXi), 0.01},  // Kp, Kd, Ki
      altCtrlRefDyn{computeKp(altRefOmega), computeKd(altRefOmega, altRefXi)}, // Kp, Kd
      altCtrlPID{computeKp(altOmega), computeKd(altOmega, altXi), 1},  // Kp, Kd, Ki
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