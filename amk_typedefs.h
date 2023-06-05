/*
 * traction_control.c
 *
 *  Created on: 05 cze 2023
 *      Author: Jakub
 */


#ifndef INC_TORQUE_VECTORING_H_
#define INC_TORQUE_VECTORING_H_

#ifdef FLOAT_COMPUTING
    typedef float sComputeType_t;
    typedef float uComputeType_t;
#else
    typedef int sComputeType_t;
    typedef uint32_t uComputeType_t;
#endif

#ifdef FLOAT_COMPUTING
#define WHEEL_BASE 1.6f
#define WHEEL_DIST 1.242f
#define STEERING_GEAR_RATIO 7.0f
#define TIRE_RADIUS 0.3f
#define G_ACC 9.81f
#else
//values x1000
#define WHEEL_BASE 1600
#define WHEEL_DIST 1242
#define STEERING_GEAR_RATIO 7000
#define TIRE_RADIUS 300
#define G_ACC 9810
#endif
#define MAX_TORQUE_VECTORING_INSTANCES	4
#endif
