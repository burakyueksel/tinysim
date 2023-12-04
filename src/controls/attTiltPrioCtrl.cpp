 /**
 * @file attTiltPrioCtrl.cpp
 * @brief Quaternion based tilt prioritizing attitude error controller
 *        source:https://www.flyingmachinearena.ethz.ch/wp-content/publications/2018/breTCST18.pdf
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#include "controls.h"
#include "parameters.h"

// Attitude, tilt prioritizing quaternion based control
Eigen::Vector3d Control::attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps)
{
    droneParameters& params_drone = droneParameters::getInstance();
    // eq.13
    Eigen::Quaterniond quatError = quatDes * quat.inverse(); // assumption: eigen does the correct multiplication
    // eq. 14
    Eigen::Vector3d angVelErr_rps = angVelDes_rps - angVel_rps;
    // compute 1/sqrt(quat.w² + quat.z²)
    double qNorm = quatError.w()*quatError.w() + quatError.z()*quatError.z();
    double oneOverQuatErrRedNorm;
    // in case quat isn not well defined
    // this happens in the following case as an example:
    // quat is the quaternion between a desired frame and the current frame
    // z axis of the desired frame is aligned exactly at the opposite direction of the z axis of the current frame
    if (qNorm<1e-4)
    {
        oneOverQuatErrRedNorm = 1e-4; // a small number.
    }
    else
    {
        oneOverQuatErrRedNorm = 1/sqrtf(qNorm);
    }
    // eq. 18
    Eigen::Quaterniond quatErrRed;
    quatErrRed.w() = oneOverQuatErrRedNorm * (quatError.w()*quatError.w() + quatError.z()*quatError.z());
    quatErrRed.x() = oneOverQuatErrRedNorm * (quatError.w()*quatError.x() - quatError.y()*quatError.z());
    quatErrRed.y() = oneOverQuatErrRedNorm * (quatError.w()*quatError.y() + quatError.x()*quatError.z());
    quatErrRed.z() = 0.0;
    // eq. 20
    Eigen::Quaterniond quatErrYaw;
    quatErrYaw.w() = oneOverQuatErrRedNorm * quatError.w();
    quatErrYaw.x() = 0.0;
    quatErrYaw.y() = 0.0;
    quatErrYaw.z() = oneOverQuatErrRedNorm * quatError.z();
    // eq. 23
    Eigen::Vector3d tauFF = params_drone.inertiaMatrix*angVelDotEst_rps - (params_drone.inertiaMatrix*angVel_rps).cross(angVel_rps);
    // eq. 21
    Eigen::Vector3d tauCtrl_Nm = params_drone.attCtrlTiltPrio.KP * quatErrRed.vec() +
                                 params_drone.attCtrlTiltPrio.KP(2,2) * mySignum(quatError.w()) * quatErrYaw.vec() +
                                 params_drone.attCtrlTiltPrio.KD * angVelErr_rps +
                                 tauFF;
    return tauCtrl_Nm;
}