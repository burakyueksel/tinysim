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
    float Kp;
    float Kd;
    float Ki;
};

struct attCtrlTiltPrioParameters
{
    Eigen::Matrix3f KP;
    Eigen::Matrix3f KD;
};

struct pdParameters
{
    float Kp;
    float Kd;
};

// DRONE PARAMETERS
class droneParameters
{
public:
    static droneParameters& getInstance();

    DroneTypes::Type droneType;
    float mass_kg;
    float actBW_rps;
    float indiOmegaBW_rps;
    float indiMuBW_rps;
    float imuAccStdev;
    float imuGyrStdev;
    float baroPressureStdev;
    Eigen::Matrix3f inertiaMatrix_kgm2;
    Eigen::Vector3f cogOffset_m;
    Eigen::Vector3f initPos_m;
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
    float gravity_mps2;
    float timeStep_s;
    float timeEnd_s;
    float R;  // Universal gas constant in J/(mol·K)
    float L_Kpm;  // temperature lapse rate (K/m) at P0 and T0
    float airMolarMass;  // Molar mass of Earth's air in kg/mol
    float P0_pa; // Standard atmospheric pressure at sea level (Pa)
    float T0_K; // Standard temperature at sea level in K
    float T0C_K; // Zero Celcius in Kelvin
    float TAmbient_C; // Static outside temperature in Celcius (C). Note: it can be approximated with TAmbient_C = T0 - L * altitudeAboveSeaLevel.

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
    float PI;
private:
    geometricParameters(); // Private constructor to enforce Singleton pattern
    geometricParameters(const geometricParameters&) = delete;
    geometricParameters& operator=(const geometricParameters&) = delete;
};