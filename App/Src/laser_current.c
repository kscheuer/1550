/*
 * laser_current.c
 *
 *  Created on: Feb 10, 2026
 *      Author: Elizabeth
 */

#include "laser_current.h"
#include "main.h"

// TODO: Implement laser current control logic

void LaserCurrent_NotifyThermalError(uint8_t error_code) {
    // Placeholder implementation
    // Turn off laser, set error flag, etc.
}


// Approach 1 
/**
 * @brief   Write DAC value using SPI5 DMA (Fire-and-Forget) with CS handling
 *
 * APPROACH:
 *   DMA2 Stream 6 is configured for single-shot (non-circular)
 * memory-to-peripheral transfers. Unlike SPI1 which runs continuously in
 * circular mode, SPI5 transfers are explicitly triggered each time we want to
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
 *   9. Transfer Complete interrupt fires → Optical_DAC_DMA_Handler() deasserts
 * CS
 *
 * WHY SINGLE-SHOT (not circular):
 *   - DAC8411 requires CS toggle per transaction to latch data
 *   - Each DAC write is a discrete 24-bit frame, not continuous streaming
 *   - Circular mode would continuously pump stale data to DAC
 *
 * NOTE: This function asserts CS (Low). The DMA2_Stream6_IRQHandler (via
 * Optical_DAC_DMA_Handler) will deassert CS (High) when transfer completes.
 */
static inline void WriteDACFireAndForget(float value) {
  uint16_t uvalue;

  /* 1. Safety Clamp */
  const float SAFE_MAX_VAL = (float)DAC_HARDWARE_LIMIT / 65535.0f;
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
  DMA2_Stream6->CR &= ~DMA_SxCR_EN;

  /* HARDWARE REQUIREMENT: Wait for EN bit to clear before changing config */
  while (DMA2_Stream6->CR & DMA_SxCR_EN) {
    __NOP();
  }

  /* Clear any pending interrupt flags */
  DMA2->HIFCR = (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 |
                 DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6);

  /* CRITICAL: Explicitly set addresses to ensure correctness */
  DMA2_Stream6->PAR = (uint32_t)&(SPI5->DR);
  DMA2_Stream6->M0AR = (uint32_t)g_spi5_tx_buffer;

  /* Configure transfer length (Must be done every time) */
  DMA2_Stream6->NDTR = SPI5_TX_BUFFER_SIZE;

  /* Enable SPI DMA Request (TXDMAEN) and Enable SPI (SPE) */
  /* If HAL init already set SPE, this is redundant but safe */
  SPI5->CR2 |= SPI_CR2_TXDMAEN;
  SPI5->CR1 |= SPI_CR1_SPE;

  /* Assert CS (Low) - Start Frame */
  SPI5_EN_GPIO_Port->BSRR = (uint32_t)SPI5_EN_Pin << 16U;

  /* Enable stream with TC Interrupt (to deassert CS later) */
  /* OPTIMIZATION: PAR, M0AR, TXDMAEN, SPE are set once in Optical_Start() */
  DMA2_Stream6->CR |= (DMA_SxCR_EN | DMA_SxCR_TCIE);
}



// Alternative approach 

#include "stdbool.h"

#include "config.h"
#include "error.h"
#include "main.h"
#include "spi_DAC8411.h"
#include <stdint.h>

static SpiDac_t spidac;

void spidac_init(SPI_HandleTypeDef *hspi) {
  spidac = (SpiDac_t){
      .hspi = hspi,
      .transfer_in_progress = 0,
      .last_value = 0.0,
  };
}

SpiDac_t *spidac_get_state(void) { return &spidac; }

float spidac_get(void) { return spidac.last_value; }

uint32_t spidac_get_overruns(void) { return spidac.overruns; }

error_t spidac_set(float value) {
  if (value < 0.0f) {
    value = 0.0f;
  } else if (value > DAC_MAX_OUTPUT_VALUE) {
    // warning_printf("DAC8411_SetPower too high! %f\r\n", value);
    value = 0.0f;
  }

  uint16_t uvalue = (value * 65535.0f);

  if (spidac.transfer_in_progress) {
    spidac.overruns += 1;
    return ERROR_FAIL;
  }

  spidac.transfer_in_progress = true;
  spidac.last_value = value;

  // Create the 24-bit frame
  uint8_t spi_data[3];
  spi_data[0] = (uvalue >> 10) & 0x3F; // First 6 bits (00 + 4 MSB of data)
  spi_data[1] = (uvalue >> 2) & 0xFF;  // Next 8 bits (middle 8 bits of data)
  spi_data[2] = (uvalue << 6) & 0xC0;  // Last 2 bits of data + 6 bits (000000)

  HAL_GPIO_WritePin(SPI5_EN_GPIO_Port, SPI5_EN_Pin, GPIO_PIN_RESET);

  // Transmit the data via SPI using DMA
  HAL_SPI_Transmit_DMA(spidac.hspi, spi_data, 3);
  return ERROR_OK;
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi == spidac.hspi) {
    // Set SPI5_EN high after data transmission is complete
    HAL_GPIO_WritePin(SPI5_EN_GPIO_Port, SPI5_EN_Pin, GPIO_PIN_SET);

    // Clear the transfer in progress flag
    spidac.transfer_in_progress = false;
  }
}
