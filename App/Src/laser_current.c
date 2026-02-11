/*
 * laser_current.c
 *
 *  Created on: Feb 10, 2026
 *      Author: Elizabeth
 */

#include "laser_current.h"
#include "main.h"
#include "app_config.h"

/* Private Definitions */
#define SPI_TX_BUFFER_SIZE 3

/* Global buffer for DMA transfer (MUST be in RAM) */
/* Marked volatile just in case, though DMA doesn't care about CPU cache coherency on M4 usually */
static volatile uint8_t g_spi5_tx_buffer[SPI_TX_BUFFER_SIZE];

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
    /* 1. Set Fixed DMA Addresses */
    /* These registers can only be written when EN=0. 
       MX_DMA_Init leaves the stream disabled, so this is safe here. */
    DMA2_Stream6->PAR = (uint32_t)&(SPI5->DR);
    DMA2_Stream6->M0AR = (uint32_t)g_spi5_tx_buffer;

    /* 2. Enable SPI DMA Request and Enable SPI Peripheral */
    /* This allows SPI to issue requests to DMA whenever CS is asserted and clock runs */
    SPI5->CR2 |= SPI_CR2_TXDMAEN;
    SPI5->CR1 |= SPI_CR1_SPE;

    /* 3. Explicitly Zero the DAC on Startup for Safety */
    Laser_SetCurrent(0.0f);
}

void Laser_SetCurrent(float current_normalized) {
    /* Simply call the fast implementation */
    WriteDACFireAndForget(current_normalized);
}

void LaserCurrent_NotifyThermalError(uint8_t error_code) {
    // Placeholder implementation
    // Turn off laser, set error flag, etc.
    WriteDACFireAndForget(0.0f); /* Safe state */
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

