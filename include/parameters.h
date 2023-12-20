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

struct pdParameters
{
    double Kp;
    double Kd;
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
    double imuAccStdev;
    double imuGyrStdev;
    double baroPressureStdev;
    Eigen::Matrix3d inertiaMatrix;
    Eigen::Vector3d cogOffset;
    Eigen::Vector3d initPos;
    pdParameters posCtrlRefDyn;
    pidParameters posCtrlPID;
    pdParameters altCtrlRefDyn;
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
    double R;  // Universal gas constant in J/(mol·K)
    double L;  // temperature lapse rate (K/m) at P0 and T0
    double airMolarMass;  // Molar mass of Earth's air in kg/mol
    double P0; // Standard atmospheric pressure at sea level (Pa)
    double T0; // Standard temperature at sea level in K
    double T0CK; // Zero Celcius in Kelvin
    double TAmbient_C; // Static outside temperature in Celcius (C). Note: it can be approximated with TAmbient_C = T0 - L * altitudeAboveSeaLevel.

private:
    physicsParameters(); // Private constructor to enforce Singleton pattern
    physicsParameters(const physicsParameters&) = delete;
    physicsParameters& operator=(const physicsParameters&) = delete;
};

// GEOMETRY PARAMETERS

class geometricParameters
{
public:
    static geometricParameters& getInstance();
    double PI;
private:
    geometricParameters(); // Private constructor to enforce Singleton pattern
    geometricParameters(const geometricParameters&) = delete;
    geometricParameters& operator=(const geometricParameters&) = delete;
};