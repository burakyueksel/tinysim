 /**
 * @file parameters_geometry.cpp
 * @brief Geometric parameters
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-06
 */
#include "parameters.h"

// Geo aux parameters


geometricParameters::geometricParameters(): 
    PI(3.141592)        
{
    // Additional initialization if needed
}

geometricParameters& geometricParameters::getInstance()
{
    static geometricParameters instance;
    return instance;
}