 /**
 * @file baro.cpp
 * @brief Barometer sensor function bodies.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-10
 */
#include "sensors.h"
#include "parameters.h"

Sensor::Baro::Baro()
{
    droneParameters& params_drone = droneParameters::getInstance();
    pressure_noise_stddev_ = params_drone.baroPressureStdev;
    // Seed the random number generator
    random_generator_.seed(std::random_device()());
}

float computeAltitude(float pressure, float temperature)
{
    physicsParameters& params_phy = physicsParameters::getInstance();
    // Compute the altitude using the ideal gas law and the international standard atmosphere model
    float altitude = 0 + 
                    ((temperature + params_phy.T0C_K) / params_phy.L_Kpm) * (pow((pressure / params_phy.P0_pa), (-1.0 * params_phy.L_Kpm * params_phy.R / (params_phy.airMolarMass * params_phy.gravity_mps2)))-1.0);
    return altitude;
}

// Function to simulate barometer readings from true simulation data
BaroStates Sensor::Baro::measurementModel(float temperature, float trueAltitude)
{
  BaroStates barometer;
  physicsParameters& params_phy = physicsParameters::getInstance();
  droneParameters& params_drone = droneParameters::getInstance();

  barometer.temperature_c = temperature;

  // Compute pressure using the barometric formula. Zero bias.
  barometer.pressure_pa = 0 + generateNormalDistribution(0.0, pressure_noise_stddev_) +
                          params_phy.P0_pa * pow(1 + params_phy.L_Kpm * (trueAltitude-0) / (temperature + params_phy.T0C_K), -1.0 * (params_phy.gravity_mps2 * params_phy.airMolarMass) / (params_phy.R * params_phy.L_Kpm));

  // compute altitude from pressure and temperature
  barometer.altitude_m = computeAltitude(barometer.pressure_pa, barometer.temperature_c);


  return barometer;
}