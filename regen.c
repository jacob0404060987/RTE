/*
 * traction_control.c
 *
 *  Created on: 05 cze 2023
 *      Author: Jakub
 */


/*****************************************************************
							INCLUDES
*****************************************************************/

#include "amk_pte.h"
#include "torque_vectoring.h"
#include "regen.h"
/*****************************************************************
						DEFINES / MACROS
*****************************************************************/
typedef struct
{
	regenBreak_t regenBreakingBuffer[MAX_REGEN_BREAKING_INSTANCES];
	int regenBreakIndex;
}memoryBuffer_t;
/*****************************************************************
						GLOBAL VARIABLES
*****************************************************************/
static memoryBuffer_t memBuffer;
/*****************************************************************
					PRIVATE FUNCTION DEFINITIONS
*****************************************************************/


static void regenSOCScheduling(regenBreak_t* const regen,const regenBreakInput_t* input)
{
    sComputeType_t current,torque;



    regen->maxChargingCurrent=current;
    regen->maxRegenTorque=torque;
}


s
/*****************************************************************
					PUBLIC FUNCTION DEFINITIONS
*****************************************************************/
static void TV_Calculate(torqueVectoring_t* const tv, const torqueVectoringInput_t* input, torqueVectoringOutput_t* const output)
{
	SteerAngleToRadius(tv,input->steerAngleSignal, input->carXVelocity);
	YawRefCalculation(tv,input->carXVelocity);
	TorqueDistrubution(tv,output,input,input->torqueFromPedal,input->lambdaRatio);
}
torqueVectoring_t* TV_Init()
{
	if(memBuffer.tvBufferIndex >= MAX_TORQUE_VECTORING_INSTANCES)
	{
		return NULL;
	}
	torqueVectoring_t* result = &memBuffer.TorqueVetoringBuffer[memBuffer.tvBufferIndex];
	memBuffer.tvBufferIndex++;

	result->pidController = Pid_Init(1,1,1,10);

	return result;
}
