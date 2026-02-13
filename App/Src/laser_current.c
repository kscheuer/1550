/*
 * laser_current.c
 *
 *  Created on: Feb 10, 2026
 *      Author: Elizabeth
 */

#include "laser_current.h"
#include "thermal_control.h"
#include "main.h"
#include "app_config.h"

/* Private Definitions */
#define SPI_TX_BUFFER_SIZE 3

/* Global buffer for DMA transfer (MUST be in RAM) */
/* Marked volatile just in case, though DMA doesn't care about CPU cache coherency on M4 usually */
static volatile uint8_t g_spi5_tx_buffer[SPI_TX_BUFFER_SIZE];

/* Stored run current setting (normalized 0.0-1.0) */
static volatile float g_run_current = 0.2424f; // 0.2424 corresponds to 80mA/330mA max dac current (15ohm pot) 
//80ma was determined ideal from laser characterization. this is laser specific and will need to be stored in flash eventually and accessed via spi
static volatile bool g_laser_enabled = false;

/* Flag to handle "Power applied while Trigger held HIGH" startup case */
static volatile bool g_startup_pending_thermal_lock = false;

/* Stabilization Delay Variables */
static volatile uint32_t g_stabilization_start_tick = 0;
static volatile bool g_stabilization_pending = false;

/* ============================================================================
 * INTERNAL FUNCTIONS
 * ============================================================================ */

/**
 * @brief   Write DAC value using SPI5 DMA (Fire-and-Forget) with CS handling
 *
 * APPROACH:
 *   DMA2 Stream 6 is configured for single-shot (non-circular)
 * memory-to-peripheral transfers. SPI5 transfers are explicitly triggered each time we want to
 * update the DAC. After transferring NDTR bytes, the stream auto-disables (NDTR
 * reaches 0, EN bit clears).
 *
 * TRIGGER SEQUENCE:
 *   1. Populate g_spi5_tx_buffer with 24-bit DAC frame
 *   2. Disable stream (required before reconfiguring - EN bit must be 0)
 *   3. Wait for EN bit to clear (hardware requirement)
 *   4. Clear interrupt flags from previous transfer
 *   5. Set NDTR to 3 (transfer count must be reloaded each time)
 *   6. Assert CS low (start SPI frame)
 *   7. Enable stream (DMA_SxCR_EN) → DMA immediately begins transfer
 *   8. DMA moves 3 bytes: g_spi5_tx_buffer → SPI5->DR
 *   9. Transfer Complete interrupt fires → Laser_DAC_DMA_Handler() deasserts
 * CS
 *
 * WHY SINGLE-SHOT (not circular):
 *   - DAC8411 requires CS toggle per transaction to latch data
 *   - Each DAC write is a discrete 24-bit frame, not continuous streaming
 *
 * NOTE: This function asserts CS (Low). The DMA2_Stream6_IRQHandler (via
 * Laser_DAC_DMA_Handler) will deassert CS (High) when transfer completes.
 */
static inline void WriteDACFireAndForget(float value) {
  uint16_t uvalue;
  // NOTE DO NOT WANT ANY TEC/TEMP CHECKS IN HERE. If so, we might not be able to turn laser off

  /* 1. Safety Clamp */
  const float SAFE_MAX_VAL = (float)LASER_DAC_HARDWARE_LIMIT / 65535.0f;
  if (value < 0.0f) {
    value = 0.0f;
  } else if (value > SAFE_MAX_VAL) {
    value = SAFE_MAX_VAL;
  }

  /* 2. Convert to 16-bit integer (0-65535) */
  uvalue = (uint16_t)(value * 65535.0f);

  /* 3. Pack into 24-bit buffer (3 bytes) */
  /* Format dictated by DAC8411:
   *         Byte 0: 00 (PD) + D15-D10 (6 bits)
   *         Byte 1: D9-D2 (8 bits)
   *         Byte 2: D1-D0 + 000000 (2 bits + padding)
   */
  g_spi5_tx_buffer[0] = (uint8_t)((uvalue >> 10) & 0x3F);
  g_spi5_tx_buffer[1] = (uint8_t)((uvalue >> 2) & 0xFF);
  g_spi5_tx_buffer[2] = (uint8_t)((uvalue << 6) & 0xC0);

  /* 4. Trigger DMA Transfer (Direct Register Access) */

  /* Ensure CS is High (Deasserted) initially to reset frame */
  SPI5_EN_GPIO_Port->BSRR = SPI5_EN_Pin;

  /* Disable DMA stream before reconfiguring (required by hardware) */
  if (DMA2_Stream6->CR & DMA_SxCR_EN)
  {
      DMA2_Stream6->CR &= ~DMA_SxCR_EN;
      
      /* HARDWARE REQUIREMENT: Wait for EN bit to clear before changing config */
      while (DMA2_Stream6->CR & DMA_SxCR_EN) {
        __NOP();
      }
  }

  /* Clear any pending interrupt flags */
  DMA2->HIFCR = (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                 DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6);

  /* Set Addresses - MOVED TO Laser_Init() */
  /* DMA2_Stream6->PAR = (uint32_t)&(SPI5->DR); */
  /* DMA2_Stream6->M0AR = (uint32_t)g_spi5_tx_buffer; */

  /* Configure transfer length (Must be done every time) */
  DMA2_Stream6->NDTR = SPI_TX_BUFFER_SIZE;

  /* Enable SPI/DMA Request - MOVED TO Laser_Init() */
  /* SPI5->CR2 |= SPI_CR2_TXDMAEN; */
  /* SPI5->CR1 |= SPI_CR1_SPE; */

  /* Assert CS (Low) - Start Frame */
  /* BSRR upper half resets bit (Low) */
  SPI5_EN_GPIO_Port->BSRR = (uint32_t)SPI5_EN_Pin << 16U;

  /* Enable stream with TC Interrupt (to deassert CS later) */
  /* OPTIMIZATION: PAR, M0AR, TXDMAEN, SPE set in Laser_Init() */
  DMA2_Stream6->CR |= (DMA_SxCR_EN | DMA_SxCR_TCIE);
}

/* ============================================================================
 * EXPORTED FUNCTIONS
 * ============================================================================ */

/**
 * @brief   One-time initialization for Laser DAC DMA and SPI
 *          Call this in main() after MX_DMA_Init and MX_SPI5_Init.
 */
void Laser_Init(void) {
    // in 638 repo this (1. 2.) was historically set every writedacfireandforget() 
  
    /* 1. Set Fixed DMA Addresses */
    /* These registers can only be written when EN=0. 
       MX_DMA_Init leaves the stream disabled, so this is safe here. */
    DMA2_Stream6->PAR = (uint32_t)&(SPI5->DR);
    DMA2_Stream6->M0AR = (uint32_t)g_spi5_tx_buffer;

    /* 2. Enable SPI DMA Request and Enable SPI Peripheral */
    /* This allows SPI to issue requests to DMA whenever CS is asserted and clock runs */
    SPI5->CR2 |= SPI_CR2_TXDMAEN;
    SPI5->CR1 |= SPI_CR1_SPE;

    // in 638 repo, this safety is added in optical_init() (app_init())

    /* 3. Explicitly Zero the DAC on Startup for Safety */
    WriteDACFireAndForget(0.0f);

    // in 638 repo, this is done in app_Start

    /* 4. Enable EXTI Interrupts (Trigger) now that system is safe */
    /* Clears any pending interrupts first just in case */
    __HAL_GPIO_EXTI_CLEAR_IT(MASTER_START_Pin);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    
    /* 5. Check Initial State */
    /* THIS IS LIKELY NOT AN ISSUE as main boards has delays and doesnt set pin high till 1550 is ready anyways */
    /* If the pin is HIGH right now (e.g. user holding button during boot),
       attempt to turn the laser on. If Thermal is not stable yet, Laser_TurnOn will
       handle setting the g_startup_pending_thermal_lock flag for us. */
    if (HAL_GPIO_ReadPin(MASTER_START_GPIO_Port, MASTER_START_Pin) == GPIO_PIN_SET) {
        Laser_TurnOn();
    }
}


/**
 * @brief   Only used for USB manual setpoint currently. 
 *          Sets the "Run" current level but does NOT turn on the laser immediately.
 *          The laser will go to this level when Laser_TurnOn() is called.
 *          However!! if  the laser is ALREADY on, this updates the output immediately.
 */
void Laser_SetRunCurrent(float current_normalized) {
    // should i have a check here or is it ok since writedacfireandforget has safety clamp
    const float SAFE_MAX_VAL = (float)LASER_DAC_HARDWARE_LIMIT / 65535.0f;
    if (current_normalized < 0.0f) {
        current_normalized = 0.0f;
    } else if (current_normalized > SAFE_MAX_VAL) {
        return; // ignore attempts to set above hardware limit. 
        //THIS WILL BE SILENT THOUGH. Print error message?? 
        //since this function is currently exclusively used by usb?
    }

    g_run_current = current_normalized;
    
    if (g_laser_enabled) {
        Laser_SetCurrent(g_run_current);
    }
}

/**
 * @brief   Enables the laser output at the stored run current level.
 *          Called by EXTI rising edge or USB command.
 */
void Laser_TurnOn(void) {
    /* CRITICAL SECTION: Do not want another interrupt to execute during this (could cause wacky dma transfer, could cause laser on even after error state fires, etc.)  */
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();

    // safety check
    // will not turn laser on if thermal control is not stable OR if the tec is off (likely error but maybe manually disabled).
     if (!Thermal_IsStable()) {
        /* Improved Logic: If Trigger is asserted but Thermal is unstable, mark as pending */
        /* This handles both Startup Race Conditions and "Early" Triggers where edge happens before stable */
        if (HAL_GPIO_ReadPin(MASTER_START_GPIO_Port, MASTER_START_Pin) == GPIO_PIN_SET) {
            g_startup_pending_thermal_lock = true;
        }
        __set_PRIMASK(primask_bit); /* Restore interrupts before returning */
        return;
    }

    g_laser_enabled = true;
    /* Ensure flag is cleared if we successfully turn on */
    g_startup_pending_thermal_lock = false; 

    /* Start the stabilization timer for the reply signal */
    g_stabilization_start_tick = HAL_GetTick();
    g_stabilization_pending = true;

    Laser_SetCurrent(g_run_current);
    
    __set_PRIMASK(primask_bit); /* Restore interrupts */
}

/**
 * @brief   Disables the laser output (sets DAC to 0).
 *          Called by EXTI falling edge, thermal error, or USB command.
 */
void Laser_TurnOff(void) {
    /* CRITICAL SECTION: Atomic State Update */
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();

    g_laser_enabled = false;
    g_startup_pending_thermal_lock = false; /* Cancel startup retry if user cycled the pin */

    /* Immediately drop the reply signal */
    HAL_GPIO_WritePin(FLEX2_GPIO_Port, FLEX2_Pin, GPIO_PIN_RESET);
    g_stabilization_pending = false;

    Laser_SetCurrent(0.0f);
    
    __set_PRIMASK(primask_bit);
}

/**
 * @brief   Check synchronization and handle delays. 
 *          Call this field in the main while(1) loop.
 */
void Laser_RunMaintenance(void) {
    /* TASK 1: Handle Startup Race Condition (Trigger HIGH while Thermal Unstable) */
    if (g_startup_pending_thermal_lock) {
        /* Check if Trigger is still HIGH (Master still wants Laser ON) */
        if (HAL_GPIO_ReadPin(MASTER_START_GPIO_Port, MASTER_START_Pin) == GPIO_PIN_RESET) {
            /* Trigger went LOW. Master gave up or cycled. Cancel retry. */
            g_startup_pending_thermal_lock = false;
        } 
        /* Check if Thermal is successfully stable now */
        else if (Thermal_IsStable()) {
            /* Attempt to turn on again */
            Laser_TurnOn();
            
            if (g_laser_enabled) {
                /* Success! We are running. Clear the startup flag so we don't auto-retry on future errors. */
                g_startup_pending_thermal_lock = false;
            }
        }
    }

    /* TASK 2: Handle Stabilization Delay for FLEX2 Signal */
    if (g_stabilization_pending && g_laser_enabled)
    {
        /* Check if enough time has passed from TurnOn */
        if ((HAL_GetTick() - g_stabilization_start_tick) >= LASER_RISE_TIME_MS)
        {
            HAL_GPIO_WritePin(FLEX2_GPIO_Port, FLEX2_Pin, GPIO_PIN_SET);
            g_stabilization_pending = false; /* One-shot */
        }
    }
}

void Laser_SetCurrent(float current_normalized) {
    /* Simply call the fast implementation */
    WriteDACFireAndForget(current_normalized);
}

void LaserCurrent_NotifyThermalError(uint8_t error_code) {
    // Turn off laser, ARE WE SETTING error flag??, etc.
    Laser_TurnOff(); 
}

/**
 * @brief   Called from DMA2_Stream6_IRQHandler to complete the transaction
 */
void Laser_DAC_DMA_Handler(void) {
  /* Check for Transfer Complete Flag */
  if (DMA2->HISR & DMA_HISR_TCIF6) {
    /* Clear flag */
    DMA2->HIFCR = DMA_HIFCR_CTCIF6;

    /* Deassert CS (High) - Latch Data in DAC8411 */
    SPI5_EN_GPIO_Port->BSRR = SPI5_EN_Pin;
  }
}

