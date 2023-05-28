/*
 * torque_vectoring.h
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson & Grubson
 */

#ifndef INC_TORQUE_VECTORING_H_
#define INC_TORQUE_VECTORING_H_

/*****************************************************************
							INCLUDES
*****************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid.h"
/*****************************************************************
					PUBLIC DEFINES / MACROS
*****************************************************************/
/*****************************************************************
						TYPES DEFINITIONS
*****************************************************************/
typedef struct
{
	sComputeType_t turnRadius;
	sComputeType_t yawRateSetpoint;
	pidController_t* pidController;
	sComputeType_t turnRadiusFlag;

}torqueVectoring_t;
typedef struct 
{
	sComputeType_t carXVelocity;
	sComputeType_t	steerAngleSignal;
	uComputeType_t torqueFromPedal;
	sComputeType_t lambdaRatio;//how much torque front vs rear
	sComputeType_t yawRateSensorSignal;
	
}torqueVectoringInput_t;
typedef struct 
{
	sComputeType_t torqueValueFrontLeft;
	sComputeType_t torqueValueFrontRight;
	sComputeType_t torqueValueRearLeft;
	sComputeType_t torqueValueRearRight;
}torqueVectoringOutput_t;

/*****************************************************************
					PUBLIC FUNCTION DECLARATIONS
*****************************************************************/
//torqueVectoringOutput_t* TV_Calculate(torqueVectoring_t* const tv, const torqueVectoringInput_t* input);
torqueVectoring_t* TV_Init();
#endif /* INC_TORQUE_VECTORING_H_ */
