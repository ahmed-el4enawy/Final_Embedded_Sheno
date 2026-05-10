/**
 * Spi.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Full-duplex SPI driver (Master + Slave) for inter-MCU IPC.
 *
 *  [NON-BLOCKING] Both Master and Slave use interrupt-driven transfers.
 *  Master: Spi_MasterTransferAsync() — ISR-driven, zero CPU polling.
 *  Slave:  Spi_SlaveStartAsync()     — RXNE interrupt-driven.
 */

#ifndef SPI_H
#define SPI_H

#include "Std_Types.h"

/* SPI peripheral IDs */
#define SPI_1    1U
#define SPI_2    2U

/* SPI modes */
#define SPI_MODE_MASTER   1U
#define SPI_MODE_SLAVE    0U

/* Baud rate prescaler selections (BR[2:0]) */
#define SPI_BAUD_DIV2     0U
#define SPI_BAUD_DIV4     1U
#define SPI_BAUD_DIV8     2U
#define SPI_BAUD_DIV16    3U
#define SPI_BAUD_DIV32    4U
#define SPI_BAUD_DIV64    5U
#define SPI_BAUD_DIV128   6U
#define SPI_BAUD_DIV256   7U

/* Callback for slave non-blocking reception */
typedef void (*SpiRxCallback)(uint8 *RxBuf, uint8 Length);

/**
 * @brief  Initialise an SPI peripheral.
 * @param  SpiId      SPI_1 or SPI_2
 * @param  Mode       SPI_MODE_MASTER or SPI_MODE_SLAVE
 * @param  BaudDiv    SPI_BAUD_DIVx  (ignored for slave)
 */
void Spi_Init(uint8 SpiId, uint8 Mode, uint8 BaudDiv);

/**
 * @brief  Full-duplex blocking transfer (Master).
 *         Simultaneously sends TxBuf and receives into RxBuf.
 * @param  SpiId   SPI_1 or SPI_2
 * @param  TxBuf   Pointer to transmit buffer
 * @param  RxBuf   Pointer to receive buffer
 * @param  Length  Number of bytes
 */
void Spi_TransmitReceive(uint8 SpiId, const uint8 *TxBuf, uint8 *RxBuf, uint8 Length);

/**
 * @brief  Pre-load the slave TX buffer so data is ready before the
 *         Master initiates a transfer.  (Slave only)
 * @param  SpiId   SPI_1 or SPI_2
 * @param  TxBuf   Pointer to data the slave will send
 * @param  Length  Number of bytes
 */
void Spi_SlavePreload(uint8 SpiId, const uint8 *TxBuf, uint8 Length);

/**
 * @brief  Register a callback invoked (from ISR) when the slave
 *         has received a complete frame.
 * @param  SpiId     SPI_1 or SPI_2
 * @param  Callback  Function pointer
 * @param  RxBuf     Buffer where received bytes are stored
 * @param  Length    Expected frame length
 */
void Spi_SlaveStartAsync(uint8 SpiId, SpiRxCallback Callback,
                          uint8 *RxBuf, uint8 Length);

/**
 * @brief  Assert CS low (Master only, manual GPIO).
 */
void Spi_CsLow(void);

/**
 * @brief  Deassert CS high (Master only, manual GPIO).
 */
void Spi_CsHigh(void);

/**
 * @brief  [NON-BLOCKING MASTER] Start an interrupt-driven full-duplex
 *         transfer.  CS is asserted automatically.  When all bytes are
 *         exchanged, the ISR deasserts CS and invokes the Callback.
 *         The main loop is NOT blocked — it can continue running the FSM.
 * @param  SpiId     SPI_1 or SPI_2
 * @param  TxBuf     Pointer to transmit buffer (must remain valid until callback)
 * @param  RxBuf     Pointer to receive buffer
 * @param  Length    Number of bytes
 * @param  Callback  Function invoked from ISR when transfer completes
 */
void Spi_MasterTransferAsync(uint8 SpiId, const uint8 *TxBuf, uint8 *RxBuf,
                              uint8 Length, SpiRxCallback Callback);

/**
 * @brief  Check if a master async transfer is currently in progress.
 * @return TRUE if busy, FALSE if idle.
 */
boolean Spi_MasterBusy(void);

#endif /* SPI_H */
