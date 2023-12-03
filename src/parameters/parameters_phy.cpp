#include "parameters.h"

// Physics aux parameters


physicsParameters::physicsParameters(): 
    gravity(9.81), // m/s2
    timeStep(0.01), // s
    timeEnd(10), //s
    PI(3.141592)        
{
    // Additional initialization if needed
}

physicsParameters& physicsParameters::getInstance()
{
    static physicsParameters instance;
    return instance;
}