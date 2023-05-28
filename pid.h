/*
 * pid.h
 *
 *  Created on: 8 kwi 2023
 *      Author: Olson
 */

#ifndef INC_PID_H_
#define INC_PID_H_

/*****************************************************************
							INCLUDES
*****************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
/*****************************************************************
					PUBLIC DEFINES / MACROS
*****************************************************************/

// maksymalna ilość instancji(w celu uniknięcia użycia danamicznej alokacji pamięci)
#define MAX_PID_INSTANCES	4

#define MIN_INTEGRAL_BUFFER_DEFAULT	-1000
#define MAX_INTEGRAL_BUFFER_DEFAULT 1000
#define MIN_OUTPUT_DEFAULT			-2000
#define MAX_OUTPUT_DEFAULT			2000

/*****************************************************************
						TYPES DEFINITIONS
*****************************************************************/

#ifdef FLOAT_COMPUTING
typedef float uComputeType_t;
typedef float sComputeType_t;
#else
typedef int uComputeType_t;
typedef uint32_t sComputeType_t;
#endif

typedef enum {
    GAIN_SCHEDULING_DISABLED,
    GAIN_SCHEDULING_LOOKUP_TABLE,
    GAIN_SCHEDULING_ENTRY_FUNCTION
}GainSchedulingState_t;


// Struktura reprezentująca parametry regulatora PID
typedef struct {
    uComputeType_t kp;
    uComputeType_t ki;
    uComputeType_t kd;
}pidGains_t;

// Struktura reprezentująca tabelę look-up dla wzmocnień
typedef struct {
    sComputeType_t setPoint;
    pidGains_t params;
}gainScheduleEntry_t;

//wskażnik do niepełnego typu tzw. Abstract Data Type
// typedef struct pidController_t pidController_t;
typedef struct pidController_t pidController_t;


/*****************************************************************
					PUBLIC FUNCTION DECLARATIONS
*****************************************************************/
/**
 * @brief Function that creates object pid, returns NULL if there is no more 
 * 			memory to allocate it.
 * 
 * @param kp Kp gain initialization value
 * @param ki Ki gain initialization value
 * @param kd Kd gain initialization value
 * @param dt Time step (sampling time), 
 * @return pidController_t* pointer to the created PID controller object, NULL if error
 */
pidController_t* Pid_Init(uComputeType_t kp,
							uComputeType_t ki, 
							uComputeType_t kd, 
							uComputeType_t dt);

/**
 * @brief Function sets gains, mostly for tuning, gain scheduling
 * 
 * @param pid pointer to the PID controller object
 * @param kp Kp gain value
 * @param ki Ki gain value
 * @param kd Kd gain value
 */
void Pid_SetGains(pidController_t* const pid, 
					uComputeType_t kp,
					uComputeType_t ki, 
					uComputeType_t kd);

/**
 * @brief Function to enable anti-windup and set the minimum and maximum integral limits.
 * 
 * @param pid pointer to the PID controller object
 * @param minIntegral Minimum integral limit
 * @param maxIntegral Maximum integral limit
 */
void Pid_EnableAntiWindup(pidController_t* const pid, 
							sComputeType_t minIntegralBuffer, 
							sComputeType_t maxIntegralBuffer);

/**
 * @brief Function to disable anti-windup.
 * 
 * @param pid pointer to the PID controller object
 */
void Pid_DisableAntiWindup(pidController_t* const pid);

/**
 * @brief Function to check if windup prevention is enabled.
 *
 * @param pid Pointer to the PID controller object
 * @return bool True if windup prevention is enabled, false otherwise
 */
bool Pid_IsWindupEnabled(const pidController_t* pid);

/**
 * @brief Function to enable gain scheduling using a look-up table 
 * and disable entry function if it was enabled.
 *
 * @param pid Pointer to the PID controller object
 * @param table Pointer to the gain schedule look-up table
 * @param n Number of entries in the look-up table
 */
void Pid_EnableGainSchedulingLookUpTable(pidController_t* const pid,
                                         const gainScheduleEntry_t* table,
                                         uint32_t n);

/**
 * @brief Function to enable gain scheduling by gain scheduling function 
 * and disables look-up-table if was enabled.
 *
 * @param pid pointer to the PID controller object
 * @param pointer to function which calculates gains from setpoint
 */
void Pid_EnableGainSchedulingEntryFunction(pidController_t* const pid,
											pidGains_t(*EntryGainFunction)(sComputeType_t setPoint));

/**
 * @brief Function to disable gain scheduling.
 *
 * @param pid pointer to the PID controller object
 */
void Pid_DisableGainScheduling(pidController_t* const pid);

/**
 * @brief Function to get the current gain scheduling state.
 *
 * @param pid Pointer to the PID controller object
 * @return GainSchedulingState Current gain scheduling state
 */
GainSchedulingState_t Pid_GetGainSchedulingState(const pidController_t* pid);

/**
 * @brief Function that calculates the control signal based on the PID algorithm.
 * 
 * @param pid pointer to the PID controller object
 * @param setPoint Desired setpoint value
 * @param feedback Feedback (process variable) value
 * @param dt Time step (sampling time), if time is strictly observed & doesnt change pass 0
 * @return sComputeType_t The calculated control signal
 */
sComputeType_t Pid_CalculateControlSignal(pidController_t* const pid, 
											uComputeType_t dt, 
											sComputeType_t setpoint, 
											sComputeType_t feedback);


#endif /* INC_PID_H_ */
