/*
 * amk_pte.h
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson
 */

#ifndef INC_AMK_PTE_H_
#define INC_AMK_PTE_H_

/*****************************************************************
							INCLUDES
*****************************************************************/

#include <stdint.h>

/*****************************************************************
					PUBLIC DEFINES / MACROS
*****************************************************************/
//choose computing on floats or int
//#define FLOAT_COMPUTING

#ifdef FLOAT_COMPUTING
typedef float uComputeType_t;
typedef float sComputeType_t;
#else
typedef int uComputeType_t;
typedef uint32_t sComputeType_t;
#endif

/*****************************************************************
						TYPES DEFINITIONS
*****************************************************************/

/*****************************************************************
					PUBLIC FUNCTION DECLARATIONS
*****************************************************************/


void AMK_Start(void);



#endif /* INC_AMK_PTE_H_ */
