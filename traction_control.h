/*
 * traction_control.h
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson
 */

#ifndef INC_TRACTION_CONTROL_H_
#define INC_TRACTION_CONTROL_H_

/*****************************************************************
							INCLUDES
*****************************************************************/
#include "pid.h"
#include "math.h"
/*****************************************************************
					PUBLIC DEFINES / MACROS
*****************************************************************/
#define MAX_TRACTION_CONTROL_INSTANCES	4
/*****************************************************************
						TYPES DEFINITIONS
*****************************************************************/
typedef struct 
{
	uComputeType_t torqueSetpoint;
	uComputeType_t actualWheelSpeed;
	uComputeType_t realWheelSpeed;
}tractionControlInput_t;


typedef struct tractionControl_t tractionControl_t;

/*****************************************************************
					PUBLIC FUNCTION DECLARATIONS
*****************************************************************/
/**
 * @brief Function that creates object tractionControl, returns NULL if there is no more 
 * 			memory to allocate it.
 * 
 * @return tractionControl_t* pointer to the created traction control object, NULL if error
 */
tractionControl_t* Tc_Init(void);

/**
 * @brief Function calculates max torque for one wheel
 * 
 * @param tc pointer to the traction control object
 * @param input pointer to input parameters
 * @return uComputeType_t max torque for wheel in the moment
 */
uComputeType_t Tc_Calculate(tractionControl_t* const tc, const tractionControlInput_t* input);

#endif /* INC_TRACTION_CONTROL_H_ */
