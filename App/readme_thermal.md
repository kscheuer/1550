# Thermal Control Application

This module handles the thermal control logic for the project.

## Structure
- **Src/thermal_control.c**: Implementation of thermal control functions.
- **Inc/thermal_control.h**: Header file with function prototypes and definitions.

## Overview

The thermal control module regulates the 1550 laser diode temperature using a TEC (Thermoelectric Cooler) driven by an ADN8834. Temperature feedback comes from a thermistor read via an ADS1115 ADC over I2C.

## Architecture

Two interrupts. 
```
TIM6 (Interrupt 40Hz) ──► I2C2 DMA Read ──► ADS1115 Thermistor ADC
                              │
                              ▼ (Interrupt DMA transfer complete)
                    Convert to Temperature
                              │
                              ▼
                    Temperature Limit Check
                              │
                              ▼
                       PID Controller
                              │
                              ▼
                    DAC1 ──► ADN8834 TEC Driver
```

## Hardware

| Component | Interface | Details |
|-----------|-----------|---------|
| ADS1115 | I2C2 | 16-bit ADC, thermistor input |
| DAC1 | Internal | 12-bit, channel 1, TEC control |
| TIM6 | Timer | 40Hz sampling trigger |
| ADN8834 | Analog | TEC driver, controlled by DAC1 |

### ADN8834 vs ADS1115 Interface

These are two separate chips with different roles:

```
  Thermistor ──────► ADN8834 ──────► ADS1115 ──────► STM32
  (10kΩ NTC)        (Chopper 1      (16-bit ADC)    (I2C read)
                     linearizes)         
                         ▲
  STM32 DAC1 ────────────┘
  (control voltage)    (Chopper 2
                        drives TEC)
```

| Chip | Type | Input | Output | Job |
|------|------|-------|--------|-----|
| **ADN8834** | TEC Driver | Thermistor + DAC voltage | Linearized temp (OUT1) + TEC current | Linearizes thermistor, drives TEC |
| **ADS1115** | 16-bit ADC | ADN8834 OUT1 voltage | Digital value (I2C) | Digitizes temperature signal |

**Why this architecture?**
- **ADN8834 Chopper 1**: Linearizes the nonlinear thermistor response (saves complex math)
- **ADS1115**: 16-bit precision (vs 12-bit STM32 ADC) for accurate temperature control
- **ADN8834 Chopper 2**: Handles high-current TEC drive (STM32 can't drive TEC directly)

## DAC to ADN8834 Voltage Mapping

The ADN8834 TEC driver expects a **0-2.5V input** with 1.25V midpoint, but the STM32 DAC outputs **0-3.3V**. We only use the lower portion of the DAC range:

| Voltage | DAC Value | TEC Action |
|---------|-----------|------------|
| 0V | 0 | Maximum cooling |
| 1.25V | ~1551 | Idle (no heating/cooling) |
| 2.5V | ~3103 | Maximum heating |
| 3.3V | 4095 | *Not used* |

**DAC value calculation:**
```
DAC_value = 4095 × (Voltage / 3.3V)
```

The 1.25V midpoint is dictated by the ADN8834, not the MCU. Constants defined in `app_config.h`:
- `TEC_DAC_MID` = 1551 (1.25V - safe/idle)
- `TEC_DAC_MAX` = 3103 (2.5V - max heating)
- `TEC_DAC_MIN` = 0 (0V - max cooling)

## Temperature Conversion

The ADN8834 Chopper 1 amplifier linearizes the thermistor output:

| Parameter | Value |
|-----------|-------|
| VREF/2 | 1.25V (center point) |
| Temp coefficient | 25 mV/°C |
| Nominal temp | 25°C at 1.25V |
| Valid range | 0.2V to 2.3V (~-10°C to 65°C) |

**Conversion formula:**
```
Temperature = 25°C + (Voltage - 1.25V) / 0.025 V/°C
```

Positive slope: higher voltage = higher temperature.

## DMA Configuration

| DMA | Stream | Channel | Purpose |
|-----|--------|---------|---------|
| DMA1 | Stream 2 | 7? | I2C2 RX (thermistor reading) |

## Timing

- **Sampling rate**: 40Hz (TIM6 period = 25ms)
- **PID update rate**: 40Hz
- **ADS1115 data rate**: 128 SPS (7.8ms conversion time)

### Why 128 SPS?

The ADS1115 data rate must be fast enough for the 40Hz control loop (25ms period):

| Data Rate | Conversion Time | Fits in 25ms? |
|-----------|-----------------|---------------|
| 8 SPS | 125ms | ❌ Too slow |
| 16 SPS | 62.5ms | ❌ Too slow |
| 32 SPS | 31.25ms | ❌ Too slow |
| 64 SPS | 15.6ms | ⚠️ Marginal |
| **128 SPS** | **7.8ms** | ✅ Safe margin |
| 250 SPS | 4ms | ✅ Overkill |
| 860 SPS | 1.16ms | ✅ Fastest, but more noise |

128 SPS provides a good balance: fast enough to complete before the next TIM6 trigger, with lower noise than higher sample rates.

The 40Hz update rate is appropriate for thermal control dynamics.

## Interrupt Flow

1. **TIM6 fires** at 40Hz
2. **HAL_TIM_PeriodElapsedCallback()** triggers I2C DMA read
3. **DMA1_Stream2** transfers 2 bytes from ADS1115
4. **HAL_I2C_MemRxCpltCallback()** processes the data:
   - Converts ADC reading to temperature
   - Checks temperature limits
   - Runs PID controller
   - Updates DAC output

### Complete Callback Chain

| Step | File | Function |
|------|------|----------|
| 1 | `Core/Src/stm32f4xx_it.c` | `TIM6_DAC_IRQHandler()` |
| 2 | HAL driver | `HAL_TIM_IRQHandler()` |
| 3 | `Core/Src/main.c` | `HAL_TIM_PeriodElapsedCallback()` |
| 4 | `App/Src/thermal_control.c` | `Thermal_TIM6_Handler()` → starts I2C DMA |
| 5 | `Core/Src/stm32f4xx_it.c` | `DMA1_Stream2_IRQHandler()` |
| 6 | HAL driver | `HAL_DMA_IRQHandler()` |
| 7 | `Core/Src/main.c` | `HAL_I2C_MemRxCpltCallback()` |
| 8 | `App/Src/thermal_control.c` | `Thermal_I2C_DMAComplete_Handler()` → PID → DAC |

## HAL Callback Integration

The thermal module uses HAL callbacks (not direct ISR calls) because:
- 40Hz timing is not time-critical (25ms budget vs. microseconds used)
- HAL handles I2C DMA flag management automatically
- Cleaner separation from CubeMX-generated code

Implement callbacks in `main.c`:

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6)
    {
        Thermal_TIM6_Handler();
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c2)
    {
        Thermal_I2C_DMAComplete_Handler();
    }
}
```

## Files

| File | Purpose |
|------|---------|
| `App/Inc/thermal_control.h` | Public API and types |
| `App/Src/thermal_control.c` | Implementation |
| `App/Inc/app_config.h` | TEC PID gains and timing constants |

## API

### Initialization
```c
Thermal_Init();   // Initialize PID, DAC, set safe defaults
Thermal_Start();  // Start TIM6, begin 40Hz sampling
Thermal_Stop();   // Stop TIM6, set TEC to safe mid-range
```

### Configuration
```c
Thermal_SetTarget(float temp);              // Set temperature setpoint
Thermal_SetPIDGains(float kp, ki, kd);      // Update PID gains at runtime
```

### Status
```c
float temp = Thermal_GetTemperature();      // Current temperature (°C)
uint16_t dac = Thermal_GetTECOutput();      // Current DAC value (0-4095)
bool stable = Thermal_IsStable();           // Within error threshold?
```

## PID Tuning (Normalized Output)

The PID operates in normalized space:
- **Input**: Temperature error in °C
- **Output**: -0.1 to +0.1 (±10% of control range)
- **Conversion**: `voltage = 1.25V × (1 + pid_output)` → DAC value

Default gains in `app_config.h`:

```c
#define THERMAL_PID_KP    0.1f   // 1°C error → 10% output
#define THERMAL_PID_KI    0.01f
#define THERMAL_PID_KD    0.0f
```

Start with small gains—thermal systems have long time constants.

## Safety

- **DAC midpoint**: 1551 (~1.25V) — idle, no heating/cooling
- **PID output limits**: ±0.1 (±10% of control range)
- **DAC safety clamp**: 1396-1706 (~1.125V-1.375V) in `WriteDAC1()`
- **Anti-windup**: Integrator limited to ±0.1
- **Temperature limits**: 5°C to 50°C

### Thermal-Optical Interlocks

The thermal module provides safety interlocks to protect the laser:

**1. Temperature Limit Exceeded → Optical Error State**

When temperature goes outside the safe operating range (5-50°C) or sensor voltage is invalid (0.2-2.3V):
- TEC hardware is disabled (GPIO)
- DAC is set to idle (1.25V)

- Laser DAC is set to 0 (safe shutdown) 
- System requires `CMD_RESET_ERROR` to recover



## Error Codes

| Code | Meaning |
|------|--------|
| `THERMAL_ERROR_NONE` | No error |
| `THERMAL_ERROR_SENSOR_RANGE` | ADC voltage outside 0.2-2.3V range |
| `THERMAL_ERROR_TEMP_LIMIT` | Temperature outside 5-50°C limits |
| `THERMAL_ERROR_I2C_COMM` | I2C communication failure |

On error, TEC output is set to mid-range (safe) and PID is paused, Laser current needs to be forced to zero as well!

## Notes

### Why HAL for Thermal (vs Direct Registers for Optical)

| Aspect | Thermal (40Hz) | Optical (10kHz) |
|--------|----------------|-----------------|
| Period | 25,000 µs | 100 µs |
| HAL overhead | ~2 µs (0.008%) | ~2 µs (2%) |
| Recommendation | HAL is fine | Direct registers |

The I2C read itself takes ~50µs at 400kHz, so optimizing flag clearing saves nothing meaningful for thermal control.

## Changelog

**2026-02-10**
- **Critical Fix**: Updated `Thermal_I2C_DMAComplete_Handler` to call `Thermal_Stop()` upon detecting a temperature error (sensor range or limits). This prevents an infinite error loop where the hardware is disabled but the 40Hz interrupt continues to fire and report errors.
- **Refactor**: Renamed `Optical_NotifyThermalError` to `LaserCurrent_NotifyThermalError` to better reflect the dependency structure.

