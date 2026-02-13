/*
 * thermal_control.c
 *
 *  Created on: Feb 10, 2026
 *      Author: Elizabeth
 *      Based on thermal_control.c file from 638 repositiory. 
 */

#include "main.h"

// TODO: Implement thermal control logic

/**
 * @file    thermal_control.c
 * @brief   Thermal Control Loop Implementation (40Hz Timer-driven)
 * 
 * Implements thermal regulation using:
 * - TIM6 ISR at 40Hz for deterministic sampling
 * - I2C2 DMA for ADS1115 thermistor reading
 * - ADN8834 Chopper 1 amplifier linearized temperature conversion
 * - 40Hz PID output to DAC1 for TEC control (ADN8834) 
 * 
 * @author  Your Name
 * @date    2025
 */

#include "thermal_control.h"
#include "laser_current.h"
#include "main.h"  /* For HAL handles */

/* ============================================================================
 * HELPER MACROS
 * ============================================================================ */

/** Clamp value between min and max */
#define CLAMP(val, min, max) \
    (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

/* ============================================================================
 * EXTERNAL HAL HANDLES (Defined in main.c by CubeMX)
 * ============================================================================ */
extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

/* ============================================================================
 * ADS1115 CONFIGURATION
 * ============================================================================ */

/* ADS1115 address and config defined in app_config.h */

/** ADS1115 register addresses */
#define ADS1115_REG_CONVERSION  0x00
#define ADS1115_REG_CONFIG      0x01

/** Number of bytes to read from ADS1115 (16-bit result) */
#define ADS1115_READ_SIZE       2

/* ============================================================================
 * ADS1115 VOLTAGE CONVERSION
 * ============================================================================
 * 
 * ADS1115 configured for:
 *   - PGA = ±2.048V (FSR)
 *   - 16-bit signed result
 *   - LSB = 2.048V / 32768 = 62.5µV
 */
/* FSR and LSB defined in app_config.h to match CONFIG_VALUE */

/* ============================================================================
 * ADN8834 TEMPERATURE CONVERSION
 * ============================================================================
 * 
 * The ADN8834 linearizes the thermistor output using its Chopper 1 amplifier.
 * 
 * Circuit parameters:
 *   - RFB = 82kΩ (feedback resistor)
 *   - RTH = 10kΩ @ 25°C (thermistor)
 *   - RX = 20kΩ (compensation resistor)
 *   - VREF = 2.5V
 * 
 * At 25°C: VOUT1 = 1.25V (VREF/2)
 * Temperature coefficient: ~25 mV/°C (positive slope: higher V = higher T)
 */
/*  #define ADN8834_VREF_HALF       1.25f       VREF/2 = 2.5V / 2 
    #define ADN8834_TEMP_COEFF      0.025f      25 mV/°C from datasheet 
    #define ADN8834_NOMINAL_TEMP    25.0f       Center point at 25°C 
    #define ADN8834_VOLTAGE_MIN     0.2f        Min expected voltage 
    #define ADN8834_VOLTAGE_MAX     2.3f        Max expected voltage 
    */

/* ============================================================================
 * GLOBAL CONTEXT AND BUFFERS
 * ============================================================================ */

/** Global thermal control context */
ThermalControlContext_t g_thermal_ctx;

/** I2C RX buffer for ADS1115 reading */
static volatile uint8_t i2c_rx_buffer[ADS1115_READ_SIZE] __attribute__((aligned(4)));

/* Static counter for stability monitoring */
static uint32_t s_unstable_counter = 0;
static uint32_t s_stable_dwell_counter = 0;

/* ============================================================================
 * PID CONTROLLER Functions
 * ============================================================================
 * Simple PID controller for thermal regulation.
 * Uses single-precision floating point for STM32F4 FPU.
 */

/**
 * @brief   Initialize a PID controller with given parameters
 * This is kind of a stupid function it doesnt initialize integrator limits properly 
 * so we just overwrite later on in code with 
 * "g_thermal_ctx.pid.integrator_min = THERMAL_PID_INTEGRATOR_MIN; 
 * g_thermal_ctx.pid.integrator_max = THERMAL_PID_INTEGRATOR_MAX; "
 */
static void PID_Init(PIDController_t* pid, 
                     float kp, float ki, float kd,
                     float output_min, float output_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integrator = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integrator_min = output_min;  /* Default: match output limits */
    pid->integrator_max = output_max;
}

/**
 * @brief   Update PID gains at runtime
 */
static void PID_SetGains(PIDController_t* pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief   Calculate PID output (call at control loop rate)
 */
static float PID_Calculate(PIDController_t* pid, 
                           float setpoint, 
                           float measurement, 
                           float dt)
{
    /* Calculate error */
    float error = setpoint - measurement;
    
    /* Proportional term */
    float p_term = pid->kp * error;
    
    /* Integral term with anti-windup */
    pid->integrator += error * dt;
    pid->integrator = CLAMP(pid->integrator, 
                            pid->integrator_min, 
                            pid->integrator_max);
    float i_term = pid->ki * pid->integrator;
    
    /* Derivative term (on error) */
    float derivative = (error - pid->prev_error) / dt;
    float d_term = pid->kd * derivative;
    pid->prev_error = error;
    
    /* Calculate total output */
    float output = p_term + i_term + d_term;
    
    /* Clamp output to limits */
    output = CLAMP(output, pid->output_min, pid->output_max);
    
    return output;
}

/* ============================================================================
 * PRIVATE HELPER FUNCTIONS
 * ============================================================================ */

/**
 * @brief   Convert I2C buffer (ADS1115 raw) to voltage
 * @return  Voltage in volts
 */
static inline float ConvertADCToVoltage(void)
{
    /* ADS1115 returns 16-bit signed value, MSB first */
    int16_t raw = ((int16_t)i2c_rx_buffer[0] << 8) | i2c_rx_buffer[1];
    return (float)raw * ADS1115_LSB_VOLTAGE;
}

/**
 * @brief   Convert ADN8834 Chopper 1 amplifier output voltage to temperature
 * 
 * The ADN8834 linearizes the thermistor output using its Chopper 1 amplifier.
 * Linear conversion: higher voltage = higher temperature (positive slope)
 * 
 * @param   voltage  Output voltage from ADN8834 OUT1 pin (V)
 * @return  Temperature in degrees Celsius, or THERMAL_ERROR_TEMPERATURE on error
 */
static float ConvertVoltageToTemperature(float voltage)
{
    /* Sanity check on input voltage (expected range based on -10°C to 65°C)  is this necessayr??*/
    if (voltage < ADN8834_VOLTAGE_MIN || voltage > ADN8834_VOLTAGE_MAX)
    {
        g_thermal_ctx.error_code = THERMAL_ERROR_SENSOR_RANGE;
        return THERMAL_ERROR_TEMPERATURE;
    }
    
    /* Linear conversion from amplifier output to temperature */
    /* Positive slope: higher voltage = higher temperature */
    float temp = ADN8834_NOMINAL_TEMP + ((voltage - ADN8834_VREF_HALF) / ADN8834_TEMP_COEFF);
    
    return temp;
}

/**
 * @brief   Write to DAC1 (direct register write) with safety limits
 * @param   value   12-bit DAC value (0-4095)
 * @note    Output is clamped to ±10% of 1.25V midpoint for TEC safety
 */
static inline void WriteDAC1(uint16_t value)
{
    /* Clamp to TEC safety limits (±30% of midpoint) */
    if (value < TEC_DAC_SAFETY_MIN)
    {
        value = TEC_DAC_SAFETY_MIN;
    }
    else if (value > TEC_DAC_SAFETY_MAX)
    {
        value = TEC_DAC_SAFETY_MAX;
    }
    
    /* Write to DAC directly via HAL */
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, value);
}

/**
 * @brief   Configure ADS1115 for continuous conversion
 * 
 * Writes the config register via blocking I2C to set up:
 *   - AIN0 single-ended input (thermistor signal)
 *   - ±2.048V full scale range (PGA)
 *   - Continuous conversion mode (always sampling)
 *   - 128 SPS data rate (7.8ms per conversion, fits in 40Hz loop)
 * 
 * After this, the ADS1115 continuously converts and updates its
 * conversion register. We just read whenever TIM6 triggers.
 * 
 * @return  true on success, false on I2C error
 * @note    Only called once at startup from Thermal_Init()
 */
static bool ADS1115_Configure(void)
{
    /* Pack 16-bit config value into 2 bytes, MSB first (big-endian) */
    uint8_t config[2];
    config[0] = (ADS1115_CONFIG_VALUE >> 8) & 0xFF;  /* MSB first */
    config[1] = ADS1115_CONFIG_VALUE & 0xFF;         /* LSB second */
    
    /* 
     * Write config register (0x01) via blocking I2C
     * 
     * I2C transaction on the wire:
     *   START → [0x90 addr+W] → ACK → [0x01 reg] → ACK → 
     *           [config MSB] → ACK → [config LSB] → ACK → STOP
     * 
     * Blocking is fine here - only runs once at init (~50µs)
     */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2,
                                                  ADS1115_ADDRESS << 1,  /* 7-bit addr → 8-bit */
                                                  ADS1115_REG_CONFIG,    /* Register 0x01 */
                                                  I2C_MEMADD_SIZE_8BIT,
                                                  config, 2,             /* 2 bytes of config */
                                                  100);                  /* 100ms timeout */
    
    if (status != HAL_OK)
    {
        g_thermal_ctx.error_code = THERMAL_ERROR_I2C_COMM;
        return false;
    }
    
    /* Small delay for config to take effect before first conversion */
    HAL_Delay(1);
    
    return true;
}

/* ============================================================================
 * Thermal Control & Monitoring Functions (some for startup, some helper functions, some purely for USB status queries)
 * ============================================================================ */

void Thermal_Init(void)
{
    /* Initialize context */
    g_thermal_ctx.temperature = 0.0f;
    g_thermal_ctx.target_temp = 25.0f;  /* Default to 25°C */
    g_thermal_ctx.tec_output = TEC_DAC_MID;  /* 1.25V = no heating/cooling */
    g_thermal_ctx.error = 0.0f;
    g_thermal_ctx.error_code = THERMAL_ERROR_NONE;
    g_thermal_ctx.state = THERMAL_STATE_OFF;
    
    /* Initialize thermal PID controller
     * Output is normalized: -1.0 (max cool) to +1.0 (max heat)
     * Limited to ±0.1 (10% of full range) for safety
     */
    PID_Init(&g_thermal_ctx.pid,
             THERMAL_PID_KP,
             THERMAL_PID_KI,
             THERMAL_PID_KD,
             THERMAL_PID_OUTPUT_MIN,
             THERMAL_PID_OUTPUT_MAX);
    
    /* Set anti-windup limits */
    g_thermal_ctx.pid.integrator_min = THERMAL_PID_INTEGRATOR_MIN;
    g_thermal_ctx.pid.integrator_max = THERMAL_PID_INTEGRATOR_MAX;
    
    /* Clear I2C buffer */
    i2c_rx_buffer[0] = 0;
    i2c_rx_buffer[1] = 0;
    
    /* Configure ADS1115 for continuous conversion */
    ADS1115_Configure();
    
    /* Ensure TEC driver is disabled during init */
    HAL_GPIO_WritePin(TEC_ENABLE_GPIO_Port, TEC_ENABLE_Pin, GPIO_PIN_RESET);
    
    /* Start DAC */
    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
    
    /* Set initial DAC output to 1.25V (ADN8834 idle - no heating/cooling) */
    WriteDAC1(TEC_DAC_MID);

    /*Both TEC driver enable and DAC set must be engaged before the TEC sees power*/
}

void Thermal_Start(void)
{
    /* Clear any previous errors */
    g_thermal_ctx.error_code = THERMAL_ERROR_NONE;
    s_unstable_counter = 0;
    s_stable_dwell_counter = 0;
    
    /* Enable TEC driver hardware */
    HAL_GPIO_WritePin(TEC_ENABLE_GPIO_Port, TEC_ENABLE_Pin, GPIO_PIN_SET);

    g_thermal_ctx.state = THERMAL_STATE_STABILIZING;
    
    /* TIM6 interrupts were already enabled, but didnt matter till now because  
    TIM6 was not started. Start TIM6 now */
    HAL_TIM_Base_Start_IT(&htim6);
}

void Thermal_Stop(void)
{
    g_thermal_ctx.state = THERMAL_STATE_OFF;

    /* Stop TIM6 */
    HAL_TIM_Base_Stop_IT(&htim6);
    
    /* Disable TEC driver hardware (immediate hardware shutoff) */
    HAL_GPIO_WritePin(TEC_ENABLE_GPIO_Port, TEC_ENABLE_Pin, GPIO_PIN_RESET);
    
    /* Set TEC to safe idle value (1.25V - no heating/cooling) */
    WriteDAC1(TEC_DAC_MID);
}

float Thermal_ReadOnce(void)
{
    uint8_t rx_buf[2];
    
    /* Blocking I2C read of conversion register */
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2,
                                                 ADS1115_ADDRESS << 1,
                                                 ADS1115_REG_CONVERSION,
                                                 I2C_MEMADD_SIZE_8BIT,
                                                 rx_buf, 2,
                                                 100);  /* 100ms timeout */
    
    if (status != HAL_OK)
    {
        g_thermal_ctx.error_code = THERMAL_ERROR_I2C_COMM;
        return THERMAL_ERROR_TEMPERATURE;
    }
    
    /* Convert to voltage */
    int16_t raw = ((int16_t)rx_buf[0] << 8) | rx_buf[1];
    float voltage = (float)raw * ADS1115_LSB_VOLTAGE;
    
    /* Convert to temperature */
    float temperature = ConvertVoltageToTemperature(voltage);
    
    /* Store in context for status queries */
    if (temperature != THERMAL_ERROR_TEMPERATURE)
    {
        g_thermal_ctx.temperature = temperature;
    }
    
    return temperature;
}

void Thermal_SetTarget(float target_temp)
{
    g_thermal_ctx.target_temp = target_temp;
}

void Thermal_SetPIDGains(float kp, float ki, float kd)
{
    PID_SetGains(&g_thermal_ctx.pid, kp, ki, kd);
}

void Thermal_GetPIDGains(float* kp, float* ki, float* kd)
{
    if (kp) *kp = g_thermal_ctx.pid.kp;
    if (ki) *ki = g_thermal_ctx.pid.ki;
    if (kd) *kd = g_thermal_ctx.pid.kd;
}

float Thermal_GetTemperature(void)
{
    return g_thermal_ctx.temperature;
}

uint16_t Thermal_GetTECOutput(void)
{
    return g_thermal_ctx.tec_output;
}

bool Thermal_IsStable(void)
{
    /* Stable only if we have entered the LOCKED state */
    /* This implies we have been within error limits for > 1 second */
    return (g_thermal_ctx.state == THERMAL_STATE_LOCKED);
}

/* ============================================================================
 * ISR HANDLERS... where the actual monitoring and control happens at 40Hz!
 * ============================================================================ */

void Thermal_TIM6_Handler(void)
{
    /* 
     * TIM6 fires at 40Hz
     * Trigger I2C DMA read from ADS1115
     */
    
    /* Start I2C DMA read of conversion register */
    HAL_I2C_Mem_Read_DMA(&hi2c2,
                         ADS1115_ADDRESS << 1,
                         ADS1115_REG_CONVERSION,
                         I2C_MEMADD_SIZE_8BIT,
                         (uint8_t*)i2c_rx_buffer,
                         ADS1115_READ_SIZE);
    
    /* 
     * When DMA completes, Thermal_I2C_DMAComplete_Handler() will be called
     * by HAL_I2C_MemRxCpltCallback()
     */
}

void Thermal_I2C_DMAComplete_Handler(void)
{
    /* ========================================
     * Step 1: Convert ADC reading to voltage
     * ======================================== */
    float voltage = ConvertADCToVoltage();
    
    /* ========================================
     * Step 2: Convert voltage to temperature
     * ======================================== */
    float temperature = ConvertVoltageToTemperature(voltage);
    
    /* Update context immediately so we don't hold stale values on error */
    g_thermal_ctx.temperature = temperature;
    
    /* Check for conversion error */
    if (temperature == THERMAL_ERROR_TEMPERATURE)
    {
        g_thermal_ctx.state = THERMAL_STATE_ERROR;
        /* Sensor out of range - disable TEC/thermistor hardware and notify laser current control */
        Thermal_Stop(); /* Stop loop to prevent error flooding and ensure hardware off */
        
        /* Just turn off laser hear directly instead of setting error flag??*/
        LaserCurrent_NotifyThermalError(THERMAL_ERROR_SENSOR_RANGE);
        return;
    }
    
    /* ========================================
     * Step 3: Temperature limit check
     * ======================================== */
    if (temperature < THERMAL_TEMP_MIN || temperature > THERMAL_TEMP_MAX)
    {
        /* Temperature out of safe operating range */
        g_thermal_ctx.error_code = THERMAL_ERROR_TEMP_LIMIT;
        g_thermal_ctx.state = THERMAL_STATE_ERROR;
        
        /* Disable TEC hardware and set DAC to idle */
        Thermal_Stop(); /* Stop loop */
        
        /* Just turn off laser hear directly instead of setting error flag??*/
        /* Notify laser current control to enter error state AND TURN OFF LASER*/
        LaserCurrent_NotifyThermalError(THERMAL_ERROR_TEMP_LIMIT);
        return;
    }
    
    /* ========================================
     * Step 4: PID Controller (Normalized Output)
     * ======================================== */
    g_thermal_ctx.error = g_thermal_ctx.target_temp - temperature;

    /* ========================================
     * Step 4b: Runtime Stability State Machine
     * ======================================== */
    if (g_thermal_ctx.state != THERMAL_STATE_OFF && g_thermal_ctx.state != THERMAL_STATE_ERROR) {
        float abs_error = g_thermal_ctx.error;
        if (abs_error < 0) abs_error = -abs_error;

        bool currently_within_limits = (abs_error <= THERMAL_ERROR_LIMIT);

        /* STATE: STABILIZING */
        if (g_thermal_ctx.state == THERMAL_STATE_STABILIZING) {
            if (currently_within_limits) {
                s_stable_dwell_counter++;
                /* If we have been stable for N cycles, promote to LOCKED */
                if (s_stable_dwell_counter >= THERMAL_ENTRY_LOCK_COUNT) {
                    g_thermal_ctx.state = THERMAL_STATE_LOCKED;
                    s_unstable_counter = 0; /* Reset loss-of-lock counter */
                }
            } else {
                /* Reset dwell counter if we dip out of limits */
                s_stable_dwell_counter = 0;
            }
        }
        
        /* STATE: LOCKED */
        else if (g_thermal_ctx.state == THERMAL_STATE_LOCKED) {
            if (!currently_within_limits) {
                s_unstable_counter++;
                
                /* If we are unstable for too long, treat as error/loss of lock */
                if (s_unstable_counter > THERMAL_LOSS_LOCK_COUNT) {
                    /* System has lost lock for >10 seconds. Turn off laser, but NOT TEC/Thermistor (no thermal_Stop() call). */
                    LaserCurrent_NotifyThermalError(THERMAL_ERROR_UNSTABLE); // 
                    
                    /* Demote back to Stabilizing? Or Error? 
                       Current logic: Just turn off laser but keep trying to cool. 
                       So we go back to Stabilizing. */
                    g_thermal_ctx.state = THERMAL_STATE_STABILIZING;
                    s_stable_dwell_counter = 0;
                    s_unstable_counter = 0;
                }
            } else {
                /* Error is within limits, reset counter */
                s_unstable_counter = 0;
            }
        }
    }
    
    /* PID outputs normalized control effort: -1.0 (cool) to +1.0 (heat)
     * Limited to ±0.1 by PID output limits
     */
    float pid_output = PID_Calculate(&g_thermal_ctx.pid,
                                     g_thermal_ctx.target_temp,
                                     temperature,
                                     THERMAL_LOOP_DT);
    
    /* ========================================
     * Step 5: Convert Normalized Output to DAC
     * ======================================== */
    /* 
     * pid_output: -0.1 to +0.1 (already clamped by PID_Calculate)
     * 
     * Conversion chain:
     *   1. Normalized → Voltage: voltage = 1.25V × (1 + pid_output)
     *   2. Voltage → DAC: dac = voltage × (4095 / 3.3V)
     * 
     * At pid_output = 0:    voltage = 1.25V → DAC = 1551 (idle)
     * At pid_output = +0.1: voltage = 1.375V → DAC = 1706 (heating)
     * At pid_output = -0.1: voltage = 1.125V → DAC = 1396 (cooling)
     */

    
    float dac_voltage = ADN8834_VMID * (1.0f + pid_output);
    uint16_t dac_value = (uint16_t)(dac_voltage * (float)INTERNAL_DAC_MAX / DAC_VREF);
    
    /* ========================================
     * Step 6: Update DAC output
     * ======================================== */
    g_thermal_ctx.tec_output = (uint16_t)dac_value;
    WriteDAC1((uint16_t)dac_value);
}

