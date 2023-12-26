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
Eigen::Vector3f Control::attTiltPrioControl(Eigen::Quaternionf quatDes, Eigen::Quaternionf quat, Eigen::Vector3f angVelDes_rps, Eigen::Vector3f angVel_rps, Eigen::Vector3f angVelDotEst_rps)
{
    droneParameters& params_drone = droneParameters::getInstance();
    // eq.13
    Eigen::Quaternionf quatError = quatDes * quat.inverse(); // assumption: eigen does the correct multiplication
    // eq. 14
    Eigen::Vector3f angVelErr_rps = angVelDes_rps - angVel_rps;
    // compute 1/sqrt(quat.w² + quat.z²)
    float qzNorm_sqrt = sqrtf(quatError.w()*quatError.w() + quatError.z()*quatError.z());
    float oneOverQuatErrRedNorm;
    // in case quat isn not well defined
    // this happens in the following case as an example:
    // quat is the quaternion between a desired frame and the current frame
    // z axis of the desired frame is aligned exactly at the opposite direction of the z axis of the current frame
    if (qzNorm_sqrt<1e-6)
    {
        oneOverQuatErrRedNorm = 1e-6; // a small number.
    }
    else
    {
        oneOverQuatErrRedNorm = 1/qzNorm_sqrt;
    }
    // eq. 18
    Eigen::Quaternionf quatErrRed;
    quatErrRed.w() = oneOverQuatErrRedNorm * (quatError.w()*quatError.w() + quatError.z()*quatError.z());
    quatErrRed.x() = oneOverQuatErrRedNorm * (quatError.w()*quatError.x() - quatError.y()*quatError.z());
    quatErrRed.y() = oneOverQuatErrRedNorm * (quatError.w()*quatError.y() + quatError.x()*quatError.z());
    quatErrRed.z() = 0.0;
    // eq. 20
    Eigen::Quaternionf quatErrYaw;
    quatErrYaw.w() = oneOverQuatErrRedNorm * quatError.w();
    quatErrYaw.x() = 0.0;
    quatErrYaw.y() = 0.0;
    quatErrYaw.z() = oneOverQuatErrRedNorm * quatError.z();
    // eq. 23
    Eigen::Vector3f tauFF = params_drone.inertiaMatrix*angVelDotEst_rps - (params_drone.inertiaMatrix*angVel_rps).cross(angVel_rps);
    // eq. 21
    Eigen::Vector3f tauCtrl_Nm = params_drone.attCtrlTiltPrio.KP * quatErrRed.vec() +
                                 params_drone.attCtrlTiltPrio.KP(2,2) * mySignum(quatError.w()) * quatErrYaw.vec() +
                                 params_drone.attCtrlTiltPrio.KD * angVelErr_rps +
                                 tauFF;
    return tauCtrl_Nm;
}