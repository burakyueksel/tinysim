 /**
 * @file parameters_fusion.cpp
 * @brief (Sensor) fusion parameters
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-29
 */
#include "parameters.h"

// Geo aux parameters


fusionParameters::fusionParameters(): 
    dim(3),
    xInit((Eigen::VectorXf(dim) << 0.0, 0.0, 0.0).finished()),
    PInit(Eigen::MatrixXf::Identity(dim, dim))
    /*TODO: update and expand this with
        1- all KF matrices
        2- different KF instances, i.e. different dims and H and A, etc for different fusions
    */
{
    // Additional initialization if needed
}

fusionParameters& fusionParameters::getInstance()
{
    static fusionParameters instance;
    return instance;
}