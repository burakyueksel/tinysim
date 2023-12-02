/*
 * File: parameters.h
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations
#include <vector>

struct DroneTypes
{
    enum Type
    {
        MC_QUAD,
        MC_HEXA,
        MC_OCTO,
        FW_MONO,
    };
};

struct altCtrlErrOutputs
{
    double controlThrust_N;
    double accCmd_mps2;
};

struct pidParameters
{
    double Kp;
    double Kd;
    double Ki;
};

struct attCtrlTiltPrioParameters
{
    Eigen::Matrix3d KP;
    Eigen::Matrix3d KD;

};

struct linSysParameters
{
    double timeConst;
    double damping;
};

struct horizontalStates
{
    double x;
    double y;
};

struct posCtrlRefStates
{
    horizontalStates posRef;
    horizontalStates velRef;
    horizontalStates accRef;
};

struct altCtrlRefStates
{
    double posRef;
    double velRef;
    double accRef;
};

struct Parameters
{
    DroneTypes::Type droneType;
    double mass;
    Eigen::Matrix3d inertiaMatrix;
    Eigen::Vector3d cogOffset;
    Eigen::Vector3d initPos;
    linSysParameters posCtrlRefDyn;
    pidParameters posCtrlPID;
    linSysParameters altCtrlRefDyn;
    pidParameters altCtrlPID;
    attCtrlTiltPrioParameters attCtrlTiltPrio;
};