/*
 * app_config.h
 *
 *  Created on: Feb 11, 2026
 *      Author: Elizabeth
 *  
 *  This file contains all application-wide configuration constants,
 *  safety limits, and tuning parameters.
 */

#ifndef INC_APP_CONFIG_H_
#define INC_APP_CONFIG_H_

/* ============================================================================
 * USB COMMUNICATION CONFIGURATION
 * ============================================================================ */
#define USB_COMM_RX_BUFFER_SIZE     128     /* Max command string length */
#define USB_COMM_CMD_QUEUE_SIZE     5       /* Depth of command queue */
#define USB_COMM_TX_BUFFER_SIZE     512     /* Output buffer size */

/* ============================================================================
 * THERMAL CONTROL CONFIGURATION
 * ============================================================================ */

/* Temperature Safety Limits (Celsius) */
#define THERMAL_TEMP_MIN            5.0f
#define THERMAL_TEMP_MAX            50.0f

/* PID Parameters */
#define THERMAL_PID_KP          0.5f
#define THERMAL_PID_KI          0.05f
#define THERMAL_PID_KD          0.0f

/* PID Output Limits (Normalized -1.0 to 1.0, but clamped for safety) */
#define THERMAL_PID_OUTPUT_MIN  -0.1f   /* -10% cooling effort */
#define THERMAL_PID_OUTPUT_MAX  0.1f    /* +10% heating effort */

/* PID Integrator Limits */
/** Anti-windup limit for thermal integrator
 * Set so i_term (Ki * integrator) can reach desired output range.
 * With Ki=0.04, limits of ±1.25 allow i_term up to ±0.05 (5%) */
#define THERMAL_PID_INTEGRATOR_MIN  (-1.25f)
#define THERMAL_PID_INTEGRATOR_MAX  1.25f

/* Stability Check */
#define THERMAL_ERROR_LIMIT     0.1f    /* +/- 0.1 degrees C considered stable */

/* PID Loop Timing */
#define THERMAL_LOOP_DT             0.025f  /* 40Hz = 25ms */

/* TEC Drive Limits (Internal DAC 12-bit) 

 The ADN8834 TEC driver expects 0-2.5V input:
 *   - 0V = Maximum cooling
 *   - 1.25V = No heating/cooling (idle)
 *   - 2.5V = Maximum heating
 *
 * The STM32 DAC outputs 0-3.3V with 12-bit resolution:
 *   - DAC value 0 = 0V
 *   - DAC value 4095 = 3.3V
 *   - DAC value for 1.25V = 4095 * (1.25 / 3.3) = 1551
 *   - DAC value for 2.5V = 4095 * (2.5 / 3.3) = 3103
 *
 * We only use the 0-2.5V portion of the DAC range.*/

#define INTERNAL_DAC_MAX            4095U
#define DAC_VREF                    3.3f

/** ADN8834 maximum input voltage (V) */
#define ADN8834_VMAX 2.5f

/** ADN8834 midpoint voltage (V) - no heating/cooling */
#define ADN8834_VMID 1.25f

/** DAC value for ADN8834 maximum (2.5V) */
/* Value not used since TEC_DAC_SAFETY_MAX is used instead and is much more
 * conservative (+10% 1.25V)*/
#define TEC_DAC_MAX                                                            \
  ((uint16_t)(INTERNAL_DAC_MAX * ADN8834_VMAX / DAC_VREF)) /* ~3103 */

/** DAC value for ADN8834 midpoint (1.25V) - idle/safe */
#define TEC_DAC_MID                                                            \
  ((uint16_t)(INTERNAL_DAC_MAX * ADN8834_VMID / DAC_VREF)) /* ~1551 */

/** DAC value for ADN8834 minimum (0V) */
/* Value not used since TEC_DAC_SAFETY_MIN is used instead and is much more
 * conservative (-10% 1.25V)*/
#define TEC_DAC_MIN 0U

/* ---- TEC Safety Limits ----
 * Limit DAC output to ±10% of midpoint to prevent excessive TEC current.
 * 1.25V ± 10% = 1.125V to 1.375V
 */
#define TEC_SAFETY_PERCENT 0.10f
#define TEC_DAC_SAFETY_MIN                                                     \
  ((uint16_t)(TEC_DAC_MID * (1.0f - TEC_SAFETY_PERCENT))) /* ~1396 (1.125V) */
#define TEC_DAC_SAFETY_MAX                                                     \
  ((uint16_t)(TEC_DAC_MID * (1.0f + TEC_SAFETY_PERCENT))) /* ~1706 (1.375V) */


/* ADN8834 Chopper Amplifier Circuit Constants */
/* Circuit parameters:
 *   - RFB = 82kΩ (feedback resistor)
 *   - RTH = 10kΩ @ 25°C (thermistor)
 *   - RX = 20kΩ (compensation resistor)
 *   - VREF = 2.5V
 */
#define ADN8834_VREF_HALF       1.25f      /* VREF/2 = 2.5V / 2 */
#define ADN8834_TEMP_COEFF      0.025f     /* 25 mV/°C from datasheet/circuit */
#define ADN8834_NOMINAL_TEMP    25.0f      /* Center point at 25°C */
#define ADN8834_VOLTAGE_MIN     0.2f       /* Min expected voltage (-10C approx) */
#define ADN8834_VOLTAGE_MAX     2.3f       /* Max expected voltage (+65C approx) */


 /* SENSOR CONFIGURATION (ADS1115)*/
#define ADS1115_ADDRESS         0x48    /* ADDR pin to GND */

/** 
 * ADS1115 Config Register Value
 * 
 * Bit fields:
 *   [15]    OS=1      Start single conversion
 *   [14:12] MUX=100   AIN0 to GND (single-ended)
 *   [11:9]  PGA=010   +/- 2.048V FSR
 *   [8]     MODE=0    Continuous conversion
 *   [7:5]   DR=100    128 SPS
 *   [4]     COMP_MODE=0
 *   [3]     COMP_POL=0
 *   [2]     COMP_LAT=0
 *   [1:0]   COMP_QUE=11 Disable comparator
 * 
 * Value: 0b1_100_010_0_100_0_0_0_11 = 0xC483
 */
#define ADS1115_CONFIG_VALUE    0xC483

/* 
 * ADS1115 Conversion Constants (Must match PGA setting in CONFIG_VALUE)
 * Current PGA = +/- 2.048V
 */
#define ADS1115_FSR_VOLTAGE     2.048f
#define ADS1115_LSB_VOLTAGE     (ADS1115_FSR_VOLTAGE / 32768.0f)

/* ============================================================================
 * LASER CONTROL CONFIGURATION
 * ============================================================================ */

/* Laser Current DAC (External DAC8411 16-bit) */
#define LASER_DAC_BITS              16
#define LASER_DAC_MAX_VALUE         65535U  /*DO NOT EXCEED DAC_HARDWARE_LIMIT */

/** DO NOT CHANGE. This corresponds to 3.3V on the DAC. The BJT is not capabale
 * of a higher voltage than this despite the DAC being 5V */
#define LASER_DAC_HARDWARE_LIMIT    43253U

#endif /* INC_APP_CONFIG_H_ */
