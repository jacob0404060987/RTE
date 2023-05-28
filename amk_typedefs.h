#include "pid.h"
#ifndef INC_TORQUE_VECTORING_H_

#endif
#define INC_TORQUE_VECTORING_H_
#ifdef FLOAT_COMPUTING
#define WHEEL_BASE 1.6
#define WHEEL_DIST 1.242
#define STEERING_GEAR_RATIO 7.0
#define TIRE_RADIUS 0.3
#define G_ACC 9.81
#else
//values x1000
#define WHEEL_BASE 1600
#define WHEEL_DIST 1242
#define STEERING_GEAR_RATIO 7000
#define TIRE_RADIUS 300
#define G_ACC 9810
#endif