 /**
 * @file smoothFilt.cpp
 * @brief Smoothening filters.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-05
 */
#include "filters.h"
#include "parameters.h"
// First order smoother. Inputs are the data, and the bandwidth of the smoothing filter. Acts like low-pass.
firstOrderSmoothFiltStates Filter::firstOrderSmoothFilter(float input, float bw, float timeStep_s)
{
    // Compute the control input (acceleration)
    float error = input - g_firstOrderSmoothFiltStates.x;
    float Kp = bw * bw; // getting KP from outside is more efficient, but this is teaching more to everyone.
    float dx = error *  Kp;
    g_firstOrderSmoothFiltStates.dx = dx;
    // integrate to state
    g_firstOrderSmoothFiltStates.x = g_firstOrderSmoothFiltStates.x + g_firstOrderSmoothFiltStates.dx * timeStep_s;
    // return
    return g_firstOrderSmoothFiltStates;
}

// Second order smoother. Inputs are the data, the bandwidth of the smoothing filter and its damping characteristics. Acts like low-pass.
secondOrderSmoothFiltStates Filter::secondOrderSmoothFilter(float input, float bw, float damping, float timeStep_s)
{
    // Compute the control input (acceleration)
    float error = input - g_secondOrderSmoothFiltStates.x;
    float derror= 0.0 - g_secondOrderSmoothFiltStates.dx;
    float Kp = bw * bw; // getting Kp from outside is more efficient, but this is teaching more to everyone. bw is omega.
    float Kd = 2 * bw * damping; // getting Kd from outside is more efficient, but this is teaching more to everyone. damping is xi 
    float ddx = error *  Kp  + derror * Kd ;
    g_secondOrderSmoothFiltStates.ddx = ddx;
    // integrate to state
    g_secondOrderSmoothFiltStates.dx = g_secondOrderSmoothFiltStates.dx + g_secondOrderSmoothFiltStates.ddx * timeStep_s;
    // integrate to state
    g_secondOrderSmoothFiltStates.dx = g_secondOrderSmoothFiltStates.x + g_secondOrderSmoothFiltStates.dx * timeStep_s;    
    // return
    return g_secondOrderSmoothFiltStates;
}

// first order filter, but to 3D data. Applies the same bandwidth to all inputs.
firstOrderSmoothFiltStates3d Filter::firstOrderSmoothFilter3d(Eigen::Vector3f input, float bw, float timeStep_s)
{
    // Compute the control input (acceleration)
    Eigen::Vector3f error = input - g_firstOrderSmoothFiltStates3d.pos;
    float Kp = bw * bw; // getting KP from outside is more efficient, but this is teaching more to everyone.
    // Put the gains and dt into matrices
    Eigen::Matrix3f KP;
    KP << Kp, 0,  0,
          0,  Kp, 0,
          0,  0,  Kp;
    Eigen::Matrix3f DT;
    DT << timeStep_s,     0,          0,
          0,              timeStep_s, 0,
          0,              0,          timeStep_s;
    Eigen::Vector3f vel = KP*error;
    g_firstOrderSmoothFiltStates3d.vel = vel;
    // integrate to state
    g_firstOrderSmoothFiltStates3d.pos = g_firstOrderSmoothFiltStates3d.pos + DT*g_firstOrderSmoothFiltStates3d.vel;
    // return
    return g_firstOrderSmoothFiltStates3d;
}