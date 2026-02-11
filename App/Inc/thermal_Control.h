/*
 * thermal_control.h
 *
 *  Created on: Feb 10, 2026
 *      Author: GitHub Copilot
 */

#ifndef INC_THERMAL_CONTROL_H_
#define INC_THERMAL_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_config.h"
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* Error Codes */
#define THERMAL_ERROR_NONE              0x00
#define THERMAL_ERROR_SENSOR_RANGE      0x01
#define THERMAL_ERROR_TEMPERATURE       -999.0f /* Sentinel value for float temp errors */
#define THERMAL_ERROR_I2C_COMM          0x02
#define THERMAL_ERROR_TEMP_LIMIT        0x03

#define ADN8834_VMID            1.25f

/* Exported types ------------------------------------------------------------*/

/** 
 * PID Controller Structure 
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integrator;
    float prev_error;
    float output_min;
    float output_max;
    float integrator_min;
    float integrator_max;
} PIDController_t;

/** 
 * Thermal Control Context Structure 
 */
typedef struct {
    float temperature;          /* Current temperature (deg C) */
    float target_temp;          /* Target temperature (deg C) */
    float error;                /* Current error (deg C) */
    uint32_t error_code;        /* Last error code */
    uint16_t tec_output;        /* Current DAC output (0-4095) */
    PIDController_t pid;        /* PID controller instance */
} ThermalControlContext_t;

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

void Thermal_Init(void);
void Thermal_Start(void);
void Thermal_Stop(void);
float Thermal_ReadOnce(void);

void Thermal_SetTarget(float target_temp);
void Thermal_SetPIDGains(float kp, float ki, float kd);
void Thermal_GetPIDGains(float* kp, float* ki, float* kd);

float Thermal_GetTemperature(void);
uint16_t Thermal_GetTECOutput(void);
bool Thermal_IsStable(void);

/* Interrupt Handlers */
void Thermal_TIM6_Handler(void);
void Thermal_I2C_DMAComplete_Handler(void);

/* External Dependencies (to be implemented in other modules) */
/* This is referenced in thermal_control.c, declaring here to satisfy compiler if needed, 
   or the user needs to provide the header for it. 
   Assuming it comes from the laser control module. */
#define ERROR_THERMAL_FAULT  0x01
void LaserCurrent_NotifyThermalError(uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* INC_THERMAL_CONTROL_H_ */
