/*
 * torque_vectoring.c
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson & Grubson
 */

/*****************************************************************
							INCLUDES
*****************************************************************/

#include "amk_pte.h"
#include "torque_vectoring.h"
#include "amk_typedefs.h"
/*****************************************************************
						DEFINES / MACROS
*****************************************************************/

struct torqueVectoring_t
{
	pidController_t* pid;
	torqueVectoringInput_t* input;
};

/*****************************************************************
						GLOBAL VARIABLES
*****************************************************************/

/*****************************************************************
					PRIVATE FUNCTION DEFINITIONS
*****************************************************************/
static sComputeType_t AckermanGeometry(sComputeType_t SteeringAngle)
{
	sComputeType_t thetaAck,alpha,beta,alphaBetaSum;
	thetaAck=SteeringAngle/STEERING_GEAR_RATIO;
	alpha=atanf((WHEEL_BASE*tanf(thetaAck))/(WHEEL_BASE+0.5*WHEEL_DIST*tanf(thetaAck)));
	beta=atanf((WHEEL_BASE*tanf(thetaAck))/(WHEEL_BASE-0.5*WHEEL_DIST*tanf(thetaAck)));
	alphaBetaSum=(alpha+beta)/2;
	return alphaBetaSum;
}
static void SteerAngleToRadius(torqueVectoring_t* const tv,sComputeType_t steeringAngle, sComputeType_t vx)
{
	//na steerng angle byl filtr
	sComputeType_t ackermanOutput=AckermanGeometry(steeringAngle);
	sComputeType_t vxSquared=vx*vx;
	sComputeType_t output;
	output=(vxSquared+WHEEL_BASE)/ackermanOutput;
	tv->turnRadius=output;
//stara patera
}
static void RadiusFlagCalculate(torqueVectoring_t* const tv)
{
	sComputeType_t turnRadius=tv->turnRadius;
	sComputeType_t result;
	if(turnRadius>=600 || turnRadius <=-600)
	{
		result=0;
	}else if(turnRadius>=0 && turnRadius <600)
	{
		result=1;
	}else if(turnRadius<0 && turnRadius >-600)
	{
		result=-1;
	}
	 tv->turnRadiusFlag=result;
}

static sComputeType_t YawRefCalculation(torqueVectoring_t* const tv, sComputeType_t vx)
{
	sComputeType_t result;
	result=vx/(tv->turnRadius);
	return tv->yawRateSetpoint;
}


static void TorqueDistrubution(torqueVectoring_t* const tv,torqueVectoringOutput_t* const output,const torqueVectoringInput_t* input,sComputeType_t pedalTorque,sComputeType_t Lambda)
{
	static sComputeType_t steering_signal,torque_diff,turnFlag;
	tv->pidController=Pid_Init(1,1,1,10);
	steering_signal=Pid_CalculateControlSignal(tv->pidController,10,tv->yawRateSetpoint,input->yawRateSensorSignal);
	torque_diff=pedalTorque*steering_signal;
	turnFlag=tv->turnRadiusFlag;
	switch (turnFlag)
	{
	case -1: //skret w lewo
	output->torqueValueFrontLeft=pedalTorque-(1-Lambda)*torque_diff;
	output->torqueValueRearLeft=pedalTorque-Lambda*torque_diff;
    output->torqueValueFrontRight=pedalTorque+(1-Lambda)*torque_diff;
	output->torqueValueFrontRight=pedalTorque+Lambda*torque_diff;
		break;
	case 0:
	output->torqueValueFrontLeft=pedalTorque;
	output->torqueValueRearLeft=pedalTorque;
    output->torqueValueFrontRight=pedalTorque;
	output->torqueValueFrontRight=pedalTorque;
		break;
	case 1: //skret w prawo
	output->torqueValueFrontLeft=pedalTorque+(1-Lambda)*torque_diff;
	output->torqueValueRearLeft=pedalTorque+Lambda*torque_diff;
    output->torqueValueFrontRight=pedalTorque-(1-Lambda)*torque_diff;
	output->torqueValueFrontRight=pedalTorque-Lambda*torque_diff;
	default:
		break;
	}
}
/*****************************************************************
					PUBLIC FUNCTION DEFINITIONS
*****************************************************************/
static void TV_Calculate(torqueVectoring_t* const tv, const torqueVectoringInput_t* input, torqueVectoringOutput_t* const output)
{
	SteerAngleToRadius(&tv,input->steerAngleSignal, input->carXVelocity);
	YawRefCalculation(&tv,input->carXVelocity);
	TorqueDistrubution(&tv, &input,&output,input->torqueFromPedal,input->lambdaRatio );
}
torqueVectoring_t* TV_Init()
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
