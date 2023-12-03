/*
 * File: main.cpp
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#include "control.h"
#include "parameters.h"
#include <iostream>
#include <fstream>

int main()
{
    // Controls
    Control ctrl;
    float timeEnd = 10.0;
    float timeStep= 0.01;

    int numSteps = timeEnd/timeStep;

    for (int step = 0; step < numSteps; ++step)
    {
        double currentTime = step * timeStep;
        std::cout << "Simulation Time: " << currentTime << " seconds" << std::endl;
        /* ATTITUDE CONTROLLER */
        Eigen::Quaterniond quatDes (1.0, 0.0, 0.0, 0.0);
        Eigen::Quaterniond quaternion (1.0, 0.0, 0.0, 0.0);
        // test direct attitude commands
        //Eigen::Quaterniond quatDes = drone.eulerToQuaternion(30, 30, 30);
        Eigen::Vector3d angVelDes_rps (0,0,0);
        Eigen::Vector3d angVel_prs (0,0,0);
        Eigen::Vector3d angVelDotEst_rps (0,0,0);
        Eigen::Vector3d torqueCtrl = ctrl.attTiltPrioControl(quatDes, quaternion, angVelDes_rps, angVel_prs, angVelDotEst_rps);
            // OUTPUT TO THE TERMINAL
        // Perform other simulation tasks
    }

    return 0;
}
