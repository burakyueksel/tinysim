#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

class Control
{
public:
    /**
     * @brief Control functions
     */
    // tilt priorizing quaternion based attitude controller
    Eigen::Vector3d attTiltPrioControl(Eigen::Quaterniond quatDes, Eigen::Quaterniond quat, Eigen::Vector3d angVelDes_rps, Eigen::Vector3d angVel_rps, Eigen::Vector3d angVelDotEst_rps);

private:

};