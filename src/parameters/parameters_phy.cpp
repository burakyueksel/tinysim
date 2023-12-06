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
    gravity(9.81), // m/s2
    timeStep(0.01), // s
    timeEnd(10) //s
{
    // Additional initialization if needed
}

physicsParameters& physicsParameters::getInstance()
{
    static physicsParameters instance;
    return instance;
}