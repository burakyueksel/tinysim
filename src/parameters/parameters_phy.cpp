 /**
 * @file parameters_phy.cpp
 * @brief Physics and environment parameters
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#include "parameters.h"

// Physics aux parameters


physicsParameters::physicsParameters(): 
    gravity(9.80665), // m/s2
    timeStep(0.01), // s
    timeEnd(10), //s
    R(8.31447),  // Universal gas constant in J/(mol·K)
    L(-0.0065),  // Temperature lapse rate (K/m) at P0 and T0
    airMolarMass(0.0289644),  // Molar mass of Earth's air in kg/mol
    P0(101325.0),  // standard atmospheric pressure at sea level (Pa)
    T0(288.15),  // Standard temperature at sea level in K
    T0CK(273.15), // Zero (0) Celcius in Kelvins (K)
    TAmbient_C(20) // Static outside temperature in Celcius (C). Note: it can be approximated with TAmbient_C = T0 - L * altitudeAboveSeaLevel.
{
    // Additional initialization if needed
}

physicsParameters& physicsParameters::getInstance()
{
    static physicsParameters instance;
    return instance;
}