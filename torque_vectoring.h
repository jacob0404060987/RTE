/*
 * torque_vectoring.c
 *
 *  Created on: 05 cze 2023
 *      Author: Jakub
 */



/*****************************************************************
							INCLUDES
*****************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "pid.h"
#include "amk_typedefs.h"
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
					PUBLIC FUNCTION DECLARATIONS*/
void TV_Calculate(torqueVectoring_t* const tv, const torqueVectoringInput_t* input, torqueVectoringOutput_t* const output);
torqueVectoring_t* TV_Init();

