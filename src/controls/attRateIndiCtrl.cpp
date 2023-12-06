 /**
 * @file attRateIndi.cpp
 * @brief Attitude rate controller based on INDI
 *        Inversion is done in attitude level, without any error controller
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-06
 */
#include "controls.h"
#include "filters.h"
#include "parameters.h"

// Attitude, tilt prioritizing quaternion based control
Eigen::Vector3d Control::attRateIndiCtrl(Eigen::Vector3d omega_rps, Eigen::Vector3d domega_des_rps, Eigen::Vector3d mu_achieved_Nm, double timeStep_s)
{
    // set the filters
    Filter omegaFilt;
    Filter muFilt;
    firstOrderSmoothFiltStates3d omega_filt_states;
    firstOrderSmoothFiltStates3d mu_filt_states;
    // get drone parameters
    droneParameters& params_drone = droneParameters::getInstance();

    // filter omega and mu
    omega_filt_states = omegaFilt.firstOrderSmoothFilter3d(omega_rps, params_drone.indiOmegaBW, timeStep_s);
    mu_filt_states = omegaFilt.firstOrderSmoothFilter3d(mu_achieved_Nm, params_drone.indiMuBW, timeStep_s);

    // delta nu: increment in state
    Eigen::Vector3d dNu = domega_des_rps - omega_filt_states.vel;

    // increment in moment, and then integration
    Eigen::Vector3d mu_demand_Nm = mu_filt_states.pos + params_drone.inertiaMatrix * dNu;

    // return demanded moments.
    return mu_demand_Nm;
}