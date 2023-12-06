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
firstOrderSmoothFiltStates Filter::firstOrderSmoothFilter(double input, double bw, double timeStep_s)
{
    // Compute the control input (acceleration)
    double error = input - g_firstOrderSmoothFiltStates.x;
    double Kp = bw * bw; // getting KP from outside is more efficient, but this is teaching more to everyone.
    double dx = error *  Kp;
    g_firstOrderSmoothFiltStates.dx = dx;
    // integrate to state
    g_firstOrderSmoothFiltStates.x = g_firstOrderSmoothFiltStates.x + g_firstOrderSmoothFiltStates.dx * timeStep_s;
    // return
    return g_firstOrderSmoothFiltStates;
}

// Second order smoother. Inputs are the data, the bandwidth of the smoothing filter and its damping characteristics. Acts like low-pass.
secondOrderSmoothFiltStates Filter::secondOrderSmoothFilter(double input, double bw, double damping, double timeStep_s)
{
    // Compute the control input (acceleration)
    double error = input - g_secondOrderSmoothFiltStates.x;
    double derror= 0.0 - g_secondOrderSmoothFiltStates.dx;
    double Kp = bw * bw; // getting Kp from outside is more efficient, but this is teaching more to everyone. bw is omega.
    double Kd = 2 * bw * damping; // getting Kd from outside is more efficient, but this is teaching more to everyone. damping is xi 
    double ddx = error *  Kp  + derror * Kd ;
    g_secondOrderSmoothFiltStates.ddx = ddx;
    // integrate to state
    g_secondOrderSmoothFiltStates.dx = g_secondOrderSmoothFiltStates.dx + g_secondOrderSmoothFiltStates.ddx * timeStep_s;
    // integrate to state
    g_secondOrderSmoothFiltStates.dx = g_secondOrderSmoothFiltStates.x + g_secondOrderSmoothFiltStates.dx * timeStep_s;    
    // return
    return g_secondOrderSmoothFiltStates;
}

// first order filter, but to 3D data. Applies the same bandwidth to all inputs.
firstOrderSmoothFiltStates3d Filter::firstOrderSmoothFilter3d(Eigen::Vector3d input, double bw, double timeStep_s)
{
    // Compute the control input (acceleration)
    Eigen::Vector3d error = input - g_firstOrderSmoothFiltStates3d.pos;
    double Kp = bw * bw; // getting KP from outside is more efficient, but this is teaching more to everyone.
    // Put the gains and dt into matrices
    Eigen::Matrix3d KP;
    KP << Kp, 0,  0,
          0,  Kp, 0,
          0,  0,  Kp;
    Eigen::Matrix3d DT;
    DT << timeStep_s,     0,          0,
          0,              timeStep_s, 0,
          0,              0,          timeStep_s;
    Eigen::Vector3d vel = KP*error;
    g_firstOrderSmoothFiltStates3d.vel = vel;
    // integrate to state
    g_firstOrderSmoothFiltStates3d.pos = g_firstOrderSmoothFiltStates3d.pos + DT*g_firstOrderSmoothFiltStates3d.vel;
    // return
    return g_firstOrderSmoothFiltStates3d;
}