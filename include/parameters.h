 /**
 * @file parameters.h
 * @brief Parameter library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
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

// DRONE PARAMETERS
class droneParameters
{
public:
    static droneParameters& getInstance();

    DroneTypes::Type droneType;
    double mass;
    double actBW;
    double indiOmegaBW;
    double indiMuBW;
    Eigen::Matrix3d inertiaMatrix;
    Eigen::Vector3d cogOffset;
    Eigen::Vector3d initPos;
    linSysParameters posCtrlRefDyn;
    pidParameters posCtrlPID;
    linSysParameters altCtrlRefDyn;
    pidParameters altCtrlPID;
    attCtrlTiltPrioParameters attCtrlTiltPrio;

private:
    droneParameters(); // Private constructor to enforce Singleton pattern
    droneParameters(const droneParameters&) = delete;
    droneParameters& operator=(const droneParameters&) = delete;
};

// PHYSICS PARAMETERS
class physicsParameters
{
public:
    static physicsParameters& getInstance();
    double gravity;
    double timeStep;
    double timeEnd;
    double PI;
private:
    physicsParameters(); // Private constructor to enforce Singleton pattern
    physicsParameters(const physicsParameters&) = delete;
    physicsParameters& operator=(const physicsParameters&) = delete;
};
