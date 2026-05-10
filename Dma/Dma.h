/**
 * Dma.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  DMA driver for USART1 TX (bonus: zero-CPU telemetry).
 *  Uses DMA2 Stream7 Channel4 (USART1_TX on STM32F401).
 */

#ifndef DMA_H
#define DMA_H

#include "Std_Types.h"

/**
 * @brief  Initialise DMA2 Stream7 for USART1 TX.
 */
void Dma_Usart1TxInit(void);

/**
 * @brief  Start a DMA transfer from memory to USART1->DR.
 * @param  data   Source buffer (must remain valid until transfer completes)
 * @param  len    Number of bytes to send
 */
void Dma_Usart1TxStart(const uint8 *data, uint16 len);

/**
 * @brief  Check if the previous DMA transfer is complete.
 * @return TRUE if idle (transfer done or not started), FALSE if busy.
 */
boolean Dma_Usart1TxBusy(void);

#endif /* DMA_H */
