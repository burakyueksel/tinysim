 /**
 * @file main.cpp
 * @brief main routine/loop of tinysim.
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#include "fusion.h"
#include "controls.h"
#include "parameters.h"
#include "physics.h"
#include "sensors.h"
#include <iostream>
#include <fstream>

int main()
{
    // Open a file to store the logs
    std::ofstream logFile("log.txt");
    // Set Physics
    RigidPhysics phy;
    // Set Controls
    Control ctrl;
    // Set Sensors
    Sensor sens;
    Sensor::IMU imu;
    Sensor::Baro baro;
    // Get parameters
    droneParameters& params_drone = droneParameters::getInstance();
    physicsParameters& params_phy = physicsParameters::getInstance();
    // Get Kalman Filter Parameters
    fusionParameters& params_fusion = fusionParameters::getInstance();

    // Set Fusion
    // TODO: so far 1 KF is created. But the code is written in a way that one can crate
    // as many as one wants. parameters_fusion shall be updated and expanded accordingly.
    KalmanFilter kf(params_fusion.dim, params_phy.timeStep_s, params_fusion.xInit, params_fusion.PInit);

    // Set simulation
    int numSteps = params_phy.timeEnd_s/params_phy.timeStep_s;

    for (int step = 0; step < numSteps; ++step)
    {
        float currentTime = step * params_phy.timeStep_s;
        std::cout << "Simulation Time: " << currentTime << " seconds" << std::endl;
        std::cout << "Mass: " << params_drone.mass_kg << " kg" << std::endl;
        /* STATES*/
        // get the physical true states of each drone
        Eigen::Vector3f position = phy.getPosition();
        Eigen::Vector3f velocity = phy.getVelocity();
        Eigen::Vector3f acceleration = phy.getAcceleration();
        Eigen::Quaternionf quaternion = phy.getQuaternion();
        Eigen::Vector3f angVel_rps = phy.getBodyRates();
        /* SENS */
        // IMU
        IMUStates imustates;
        imustates = imu.measurementModel(acceleration, angVel_rps);
        // BARO
        BaroStates barostates;
        barostates = baro.measurementModel(params_phy.TAmbient_C, -position[2]);
        // GNSS
        // RADAR
        // LIDAR
        // FUSION
        /* Example.
        */
        // Prediction step
        kf.predict();
        // Get measurement
        Eigen::VectorXf measurement = Eigen::VectorXf::Random(params_fusion.dim);
        // Update step
        kf.update(measurement);
        // Print current state estimate
        std::cout << "Time: " << step << ", State: " << kf.getState().transpose() << std::endl;
        /* CONTROL*/
        // float zCmd = -10.0; // meaning 10 meters up
        // altCtrlErrOutputs altCtrl = ctrl.altPidControl(zCmd, position.z(), velocity.z(), quaternion, params_phy.timeStep_s);
        //Eigen::Vector3f torqueCtrl = ctrl.attTiltPrioControl(quatDes, quaternion, angVelDes_rps, angVel_prs, angVelDotEst_rps);
        // set the external torques and forces
        phy.setExternalTorqueBody(Eigen::Vector3f(0.0, 0.0, 0.0));
        phy.setExternalForceBody(Eigen::Vector3f(0.0, 0.0, 0.0));
        // Update states
        // update all states
        phy.updateState(params_phy.timeStep_s);
        // OUTPUT TO THE TERMINAL
        std::cout   << " position (NED): "
                    << position.x() << ", "
                    << position.y() << ", "
                    << position.z() << std::endl;
        std::cout   << " Baro altitude "
                    << barostates.altitude_m << std::endl;
        // Print the quaternion of each drone
        std::cout   << " unit quaternion: "
                    << quaternion.w() << ", "
                    << quaternion.x() << ", "
                    << quaternion.y() << ", "
                    << quaternion.z() << std::endl;
        // Perform other simulation tasks
        // OUTPUT TO THE FILE
        // Store the positions in the file
        logFile << currentTime
                    << " " << position.x() << " " << position.y() << " " << position.z()
                    << " " << quaternion.w() << " " << quaternion.x() << " " << quaternion.y() << " " << quaternion.z()
                    << "\n";
    }
    // Close the output file
    logFile.close();

    return 0;
}