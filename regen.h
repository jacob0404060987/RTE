/*
 * traction_control.c
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
	sComputeType_t wheelSpeed;
    uComputeType_t breakPedalSignal;
    uComputeType_t stateOfCharge;
}regenBreakInput_t;
typedef struct 
{
	uComputeType_t startBreakingPosition;
    uComputeType_t endBreakingPosition;
    sComputeType_t maxRegenTorque;
	sComputeType_t maxChargingCurrent;
}regenBreak_t;
typedef struct 
{   
    sComputeType_t regenTorque;
}regenBreakOutput_t;

/*****************************************************************
					PUBLIC FUNCTION DECLARATIONS*/
void Regen_Calculate(regenBreak_t* const regen, const regenBreakInput_t* input, regenBreakOutput_t* const output);
regenBreak_t* Regen_Init();

