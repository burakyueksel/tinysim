 /**
 * @file filters.h
 * @brief Filter library declerations
 */
/*
 * Author: Burak Yueksel
 * Date: 2023-12-05
 */

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


class Filter
{
public:
    /**
     * @brief Filter functions
     */
    firstOrderSmoothFiltStates firstOrderSmoothFilter(double input, double bw, double timeStep_s);
    secondOrderSmoothFiltStates secondOrderSmoothFilter(double input, double bw, double damping, double timeStep_s);


private:
    firstOrderSmoothFiltStates g_firstOrderSmoothFiltStates;
    secondOrderSmoothFiltStates g_secondOrderSmoothFiltStates;
};