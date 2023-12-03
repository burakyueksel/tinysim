// parameters.h

#pragma once

#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

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

class Parameters
{
public:
    static Parameters& getInstance();

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

private:
    Parameters(); // Private constructor to enforce Singleton pattern
    Parameters(const Parameters&) = delete;
    Parameters& operator=(const Parameters&) = delete;
};
