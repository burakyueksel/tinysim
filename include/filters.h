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
    float x;
    float dx;
};

struct secondOrderSmoothFiltStates
{
    float x;
    float dx;
    float ddx;
};

struct firstOrderSmoothFiltStates3d
{
    Eigen::Vector3f pos;
    Eigen::Vector3f vel;
};


class Filter
{
public:
    /**
     * @brief Filter functions
     */
    firstOrderSmoothFiltStates firstOrderSmoothFilter(float input, float bw, float timeStep_s);
    secondOrderSmoothFiltStates secondOrderSmoothFilter(float input, float bw, float damping, float timeStep_s);
    firstOrderSmoothFiltStates3d firstOrderSmoothFilter3d(Eigen::Vector3f input, float bw, float timeStep_s);


private:
    firstOrderSmoothFiltStates g_firstOrderSmoothFiltStates;
    secondOrderSmoothFiltStates g_secondOrderSmoothFiltStates;
    firstOrderSmoothFiltStates3d g_firstOrderSmoothFiltStates3d;
};