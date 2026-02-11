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
#include <stdbool.h>

/* Exported constants --------------------------------------------------------*/

/* Error Codes */
#define THERMAL_ERROR_NONE              0x00
#define THERMAL_ERROR_SENSOR_RANGE      0x01
#define THERMAL_ERROR_TEMPERATURE       -999.0f /* Sentinel value for float temp errors */
#define THERMAL_ERROR_I2C_COMM          0x02
#define THERMAL_ERROR_TEMP_LIMIT        0x03

/* DAC Limits (12-bit DAC: 0-4095) */
/* 1.25V @ 3.3V VREF = ~1551 counts */
/* Safety limit: ±10% of full range or specific requirements? 
 * The code mentions ±10% of 1.25V midpoint.
 * 1.25V * 0.9 = 1.125V -> ~1396 counts
 * 1.25V * 1.1 = 1.375V -> ~1706 counts
 * Let's define safe bounded limits around the midpoint.
 */
#define TEC_DAC_MID             1551    /* ~1.25V with 3.3V reference */
#define TEC_DAC_SAFETY_RANGE    310     /* +/- ~0.25V range (safe) */
#define TEC_DAC_SAFETY_MIN      (TEC_DAC_MID - TEC_DAC_SAFETY_RANGE)
#define TEC_DAC_SAFETY_MAX      (TEC_DAC_MID + TEC_DAC_SAFETY_RANGE)

#define INTERNAL_DAC_MAX        4095
#define DAC_VREF                3.3f
#define ADN8834_VMID            1.25f

/* Temperature Limits (Celsius) */
#define THERMAL_TEMP_MIN        10.0f
#define THERMAL_TEMP_MAX        45.0f

/* PID Parameters */
#define THERMAL_LOOP_DT         0.025f  /* 40Hz = 25ms */

/* Tuned PID Gains (Example values, should be tuned for actual system) */
#define THERMAL_PID_KP          0.5f
#define THERMAL_PID_KI          0.05f
#define THERMAL_PID_KD          0.0f

/* PID Output Limits (Normalized -1.0 to 1.0, but clamped for safety) */
#define THERMAL_PID_OUTPUT_MIN  -0.1f   /* -10% cooling effort */
#define THERMAL_PID_OUTPUT_MAX  0.1f    /* +10% heating effort */

/* PID Integrator Limits */
#define THERMAL_PID_INTEGRATOR_MIN  THERMAL_PID_OUTPUT_MIN
#define THERMAL_PID_INTEGRATOR_MAX  THERMAL_PID_OUTPUT_MAX

/* Stability Check */
#define THERMAL_ERROR_LIMIT     0.1f    /* +/- 0.1 degrees C considered stable */


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
