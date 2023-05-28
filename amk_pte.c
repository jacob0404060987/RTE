/*
 * amk_pte.c
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson
 */

/*****************************************************************
							INCLUDES
*****************************************************************/

#include "amk_pte.h"
#include "traction_control.h"
#include "torque_vectoring.h"

/*****************************************************************
						DEFINES / MACROS
*****************************************************************/



/*****************************************************************
						GLOBAL VARIABLES
*****************************************************************/

/*****************************************************************
					PRIVATE FUNCTION DEFINITIONS
*****************************************************************/

static void TractionControlCalculate()
{

}

static void TorqueVectoringCalculate()
{

}

/*****************************************************************
					PUBLIC FUNCTION DEFINITIONS
*****************************************************************/

void StartTaskTractionControl(void const * argument)
{
  /* USER CODE BEGIN StartTaskTractionControl */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskTractionControl */
}
