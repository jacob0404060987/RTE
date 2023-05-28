/*
 * pid.c
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson
 */

/*****************************************************************
							INCLUDES
*****************************************************************/

#include "pid.h"

/*****************************************************************
						DEFINES / MACROS
*****************************************************************/
typedef struct
{
	gainScheduleEntry_t *gainScheduleEntry;
	uint32_t numberOfLookUpTableThresholds;	
	GainSchedulingState_t gainSchedulingState;
	pidGains_t(*EntryGainFunction)(sComputeType_t setPoint);
}gainScheduling_t;

struct pidController_t
{
	pidGains_t	pidGains;
	gainScheduling_t gainScheduling;
	sComputeType_t integralBuffer;
	sComputeType_t error;
	sComputeType_t previouserror;
	sComputeType_t setpoint;
	sComputeType_t feedback;
	sComputeType_t output;
	uComputeType_t dt;
	sComputeType_t maxOutput;
	sComputeType_t minOutput;
	sComputeType_t maxIntegralBuffer;
	sComputeType_t minIntegralBuffer;
	bool isAntiWindUpEnabled;
};

typedef struct
{
	pidController_t pidControllerBuffer[MAX_PID_INSTANCES];
	int controllerBufferIndex;
}memoryBuffer_t;
/*****************************************************************
						GLOBAL VARIABLES
*****************************************************************/
//static pid_t pidBuffer[MAX_PID_INSTANCES];
//static pidController_t pidPrvBuffer[MAX_PID_INSTANCES];
static memoryBuffer_t memBuffer;
/*****************************************************************
					PRIVATE FUNCTION DEFINITIONS
*****************************************************************/

static pidGains_t GainSchedulingCalculateGains(pidController_t* pid)
{
	pidGains_t result = {0};
	gainScheduling_t* gainScheduling = &pid->gainScheduling;

	switch (gainScheduling->gainSchedulingState)
	{
		case GAIN_SCHEDULING_LOOKUP_TABLE:
		{
			if(pid->setpoint > gainScheduling->gainScheduleEntry[0].setPoint)
			{
				for (int i = 0; i < gainScheduling->numberOfLookUpTableThresholds; i++) 
				{
					if (pid->setpoint < gainScheduling->gainScheduleEntry[i].setPoint) 
					{
						return gainScheduling->gainScheduleEntry[i].params;
					}
				}
			}
			else if(pid->setpoint < gainScheduling->gainScheduleEntry[0].setPoint)
			{
				for (int i = 0; i < gainScheduling->numberOfLookUpTableThresholds; i++) 
				{
					if (pid->setpoint > gainScheduling->gainScheduleEntry[i].setPoint) 
					{
						return gainScheduling->gainScheduleEntry[i].params;
					}
				}
			}
			else
			{
				return gainScheduling->gainScheduleEntry[0].params;
			}
			//shouldnt go there error return zero
			return result;
		}break;

		case GAIN_SCHEDULING_ENTRY_FUNCTION:
		{
			return gainScheduling->EntryGainFunction(pid->setpoint);
		}break;
	}
}

/*****************************************************************
					PUBLIC FUNCTION DEFINITIONS
*****************************************************************/

pidController_t* Pid_Init(uComputeType_t kp,
							uComputeType_t ki, 
							uComputeType_t kd, 
							uComputeType_t dt)
{
	if(memBuffer.controllerBufferIndex >= MAX_PID_INSTANCES)
	{
		return NULL;
	}
	pidController_t* result = &memBuffer.pidControllerBuffer[memBuffer.controllerBufferIndex];
	memBuffer.controllerBufferIndex++;

	// Inicjalizacja zmiennych
	result->pidGains.kp = kp;
	result->pidGains.ki = ki;
	result->pidGains.ki = kd;
	result->dt = dt;
	result->minIntegralBuffer = MIN_INTEGRAL_BUFFER_DEFAULT;
	result->maxIntegralBuffer = MAX_INTEGRAL_BUFFER_DEFAULT;
	result->minOutput = MIN_OUTPUT_DEFAULT;
	result->maxOutput = MAX_OUTPUT_DEFAULT;

	return result;
}

void Pid_SetGains(pidController_t* const pid, 
					uComputeType_t kp,
					uComputeType_t ki, 
					uComputeType_t kd)
{
	pid->pidGains.kp = kp;
	pid->pidGains.ki = ki;
	pid->pidGains.ki = kd;
}

void Pid_EnableAntiWindup(pidController_t* const pid, 
							sComputeType_t minIntegralBuffer, 
							sComputeType_t maxIntegralBuffer)
{
	pid->isAntiWindUpEnabled = true;
	pid->minIntegralBuffer = minIntegralBuffer;
	pid->maxIntegralBuffer = maxIntegralBuffer;
}

void Pid_DisableAntiWindup(pidController_t* const pid)
{
	pid->isAntiWindUpEnabled = false;
}

bool Pid_IsWindupEnabled(const pidController_t* pid)
{
	return pid->isAntiWindUpEnabled;
}

void Pid_EnableGainSchedulingLookUpTable(pidController_t* const pid,
                                         const gainScheduleEntry_t* table,
                                         uint32_t n)
{
	pid->gainScheduling.gainSchedulingState = GAIN_SCHEDULING_LOOKUP_TABLE;
	pid->gainScheduling.gainScheduleEntry = table;
	pid->gainScheduling.numberOfLookUpTableThresholds;
}

void Pid_EnableGainSchedulingEntryFunction(pidController_t* const pid,
											pidGains_t(*EntryGainFunction)(sComputeType_t setPoint))
{
	pid->gainScheduling.gainSchedulingState = GAIN_SCHEDULING_ENTRY_FUNCTION;
	pid->gainScheduling.EntryGainFunction = EntryGainFunction;
}

void Pid_DisableGainScheduling(pidController_t* const pid)
{
	pid->gainScheduling.gainSchedulingState = GAIN_SCHEDULING_DISABLED;
}

GainSchedulingState_t Pid_GetGainSchedulingState(const pidController_t* pid)
{
	return pid->gainScheduling.gainSchedulingState;
}

sComputeType_t Pid_CalculateControlSignal(pidController_t* const pid, 
											uComputeType_t dt, 
											sComputeType_t setpoint, 
											sComputeType_t feedback)
{
	sComputeType_t result = 0;

    // Obliczanie uchybu
    pid->error = setpoint - feedback;

	if(pid->gainScheduling.gainSchedulingState != GAIN_SCHEDULING_DISABLED)
	{
		pid->pidGains = GainSchedulingCalculateGains(pid);
	}

    // Obliczanie skłądowych sum 
    sComputeType_t proportionalTerm = pid->pidGains.kp * pid->error;
	pid->integralBuffer += pid->error * pid->dt;

	//TODO można zastosować clamping jak w simulinku
    // Anti wind-up
	if (pid->integralBuffer > pid->maxIntegralBuffer) 
	{
		pid->integralBuffer = pid->maxIntegralBuffer;
	} 
	else if (pid->integralBuffer < pid->minIntegralBuffer) 
	{
		pid->integralBuffer = pid->minIntegralBuffer;
	}

    sComputeType_t integralTerm = pid->pidGains.ki * pid->integralBuffer;
    sComputeType_t derivativeTerm = pid->pidGains.kd * ((pid->error - pid->previouserror)/pid->dt);

    // Obliczanie sygnału sterującego
    sComputeType_t result = proportionalTerm + integralTerm + derivativeTerm;

    // Ograniczenie sygnału sterującego do zakresu [minOutput, maxOutput]
	if (result > pid->maxOutput) 
	{
		result = pid->maxOutput;
	} 
	else if (result < pid->minOutput) 
	{
		result = pid->minOutput;
	}
	pid->previouserror = pid->error;

    return result;
}