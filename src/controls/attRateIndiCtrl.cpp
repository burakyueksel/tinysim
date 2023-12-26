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
Eigen::Vector3f Control::attRateIndiCtrl(Eigen::Vector3f omega_rps, Eigen::Vector3f domega_des_rps, Eigen::Vector3f mu_achieved_Nm, float timeStep_s)
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
    mu_filt_states = muFilt.firstOrderSmoothFilter3d(mu_achieved_Nm, params_drone.indiMuBW, timeStep_s);

    // delta nu: increment in state
    Eigen::Vector3f dNu = domega_des_rps - omega_filt_states.vel;

    // increment in moment, and then integration
    Eigen::Vector3f mu_demand_Nm = mu_filt_states.pos + params_drone.inertiaMatrix * dNu;

    // return demanded moments.
    return mu_demand_Nm;

    // Notes:
    /* This way absolute moments are computed and after control allocation absolute motor commands will be computed.
     * Another way is first compute the increment in moment, do control allocation and then do integration.
     * In this case no filtering is done to mu (we still do to omega), but to u. For G_inv is control allocation:
     * Eigen::Vector 3d deltaMu = params_drone.inertiaMatrix * dNu; // delta moments
     * deltaU = G_inv * deltaMu; // delta motor commands
     * u_filt_states = uFilt.firstOrderSmoothFilterXd(u, bw, timeStep_s); // motor commands filtered for time equivalence
     * u = u_filt_states.pos + deltaU; // motor commands are integrated
    */
}