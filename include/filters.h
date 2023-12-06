 /**
 * @file controls.h
 * @brief Control library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-02
 */
#pragma once
#include <eigen3/Eigen/Dense> // Include Eigen library for vector and matrix operations

struct firstOrderSmoothFiltStates
{
    double x;
    double dx;
};

struct secondOrderSmoothFiltStates
{
    double x;
    double dx;
    double ddx;
};

struct firstOrderSmoothFiltStates3d
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
};


class Filter
{
public:
    /**
     * @brief Filter functions
     */
    firstOrderSmoothFiltStates firstOrderSmoothFilter(double input, double bw, double timeStep_s);
    secondOrderSmoothFiltStates secondOrderSmoothFilter(double input, double bw, double damping, double timeStep_s);
    firstOrderSmoothFiltStates3d firstOrderSmoothFilter3d(Eigen::Vector3d input, double bw, double timeStep_s);


private:
    firstOrderSmoothFiltStates g_firstOrderSmoothFiltStates;
    secondOrderSmoothFiltStates g_secondOrderSmoothFiltStates;
    firstOrderSmoothFiltStates3d g_firstOrderSmoothFiltStates3d;
};