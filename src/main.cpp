/*
 * File: main.cpp
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#include "controls.h"
#include "parameters.h"
#include "physics.h"
#include <iostream>
#include <fstream>

int main()
{
    // Set Physics
    RigidPhysics phy;
    // Set Controls
    Control ctrl;
    // Get parameters
    droneParameters& params_drone = droneParameters::getInstance();
    physicsParameters& params_phy = physicsParameters::getInstance();

    int numSteps = params_phy.timeEnd/params_phy.timeStep;

    for (int step = 0; step < numSteps; ++step)
    {
        double currentTime = step * params_phy.timeStep;
        std::cout << "Simulation Time: " << currentTime << " seconds" << std::endl;
        std::cout << "Mass: " << params_drone.mass << " kg" << std::endl;
        /* STATES*/
        // get the translational states of each drone
        Eigen::Vector3d position = phy.getPosition();
        Eigen::Vector3d velocity = phy.getVelocity();
        Eigen::Quaterniond quaternion = phy.getQuaternion();
        Eigen::Vector3d angVel_prs = phy.getBodyRates();
        /* SENS */
        /* CONTROL*/
        //Eigen::Vector3d torqueCtrl = ctrl.attTiltPrioControl(quatDes, quaternion, angVelDes_rps, angVel_prs, angVelDotEst_rps);
        // set the external torques and forces
        phy.setExternalTorqueBody(Eigen::Vector3d(0.0, 0.0, 0.0));
        phy.setExternalForceBody(Eigen::Vector3d(0.0, 0.0, 0.0));
        // Update states
        // update all states
        phy.updateState(params_phy.timeStep);
        // OUTPUT TO THE TERMINAL
        std::cout   << " position (NED): "
                    << position.x() << ", "
                    << position.y() << ", "
                    << position.z() << std::endl;
        // Print the quaternion of each drone
        std::cout   << " unit quaternion: "
                    << quaternion.w() << ", "
                    << quaternion.x() << ", "
                    << quaternion.y() << ", "
                    << quaternion.z() << std::endl;
        // Perform other simulation tasks
    }

    return 0;
}
