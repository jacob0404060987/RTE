/*
 * traction_control.c
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson
 */

/*****************************************************************
							INCLUDES
*****************************************************************/
#include "traction_control.h"
#include "amk_pte.h"

/*****************************************************************
						DEFINES / MACROS
*****************************************************************/

struct tractionControl_t
{
	pidController_t* pid;
	tractionControlInput_t* input;
	sComputeType_t	slipRatio;
	sComputeType_t	slipRatioBest;
};


typedef struct
{
	tractionControl_t tractionControlBuffer[MAX_TRACTION_CONTROL_INSTANCES];
	int tcBufferIndex;
}memoryBuffer_t;

/*****************************************************************
						GLOBAL VARIABLES
*****************************************************************/

static memoryBuffer_t memBuffer;

/*****************************************************************
					PRIVATE FUNCTION DEFINITIONS
*****************************************************************/

sComputeType_t SlipRatio(tractionControl_t* tc)
{
	if(tc->input->realWheelSpeed != 0)
	{
		return (tc->input->actualWheelSpeed - tc->input->realWheelSpeed) / tc->input->realWheelSpeed;
	}
	else
	{
		return 0;
	}
}

/*****************************************************************
					PUBLIC FUNCTION DEFINITIONS
*****************************************************************/

tractionControl_t* Tc_Init(void)
{
	if(memBuffer.tcBufferIndex >= MAX_TRACTION_CONTROL_INSTANCES)
	{
		return NULL;
	}
	tractionControl_t* result = &memBuffer.tractionControlBuffer[memBuffer.tcBufferIndex];
	memBuffer.tcBufferIndex++;

	result->pid = Pid_Init(1,1,1,10);

	return result;
}

uComputeType_t Tc_Calculate(tractionControl_t* const tc, const tractionControlInput_t* input)
{
	tc->slipRatio = SlipRatio(tc);
	if(tc->slipRatio <= tc->slipRatioBest)
	{
		return tc->input->torqueSetpoint;
	}
	else
	{
		return Pid_CalculateControlSignal(tc->pid, 10, tc->slipRatioBest, tc->slipRatio);
	}
}
