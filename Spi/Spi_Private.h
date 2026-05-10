/**
 * Spi_Private.h
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  SPI register map for STM32F401 (bare-metal, no HAL).
 */

#ifndef SPI_PRIVATE_H
#define SPI_PRIVATE_H

#include "Std_Types.h"

typedef struct {
    volatile uint32 CR1;       /* 0x00 - Control register 1       */
    volatile uint32 CR2;       /* 0x04 - Control register 2       */
    volatile uint32 SR;        /* 0x08 - Status register          */
    volatile uint32 DR;        /* 0x0C - Data register            */
    volatile uint32 CRCPR;     /* 0x10 - CRC polynomial           */
    volatile uint32 RXCRCR;    /* 0x14 - RX CRC                   */
    volatile uint32 TXCRCR;    /* 0x18 - TX CRC                   */
    volatile uint32 I2SCFGR;   /* 0x1C - I2S config               */
    volatile uint32 I2SPR;     /* 0x20 - I2S prescaler            */
} SpiType;

/* Base addresses */
#define SPI1_BASE_ADDR    0x40013000UL   /* APB2 */
#define SPI2_BASE_ADDR    0x40003800UL   /* APB1 */
#define SPI3_BASE_ADDR    0x40003C00UL   /* APB1 */

/* CR1 bit positions */
#define SPI_CR1_CPHA       0U
#define SPI_CR1_CPOL       1U
#define SPI_CR1_MSTR       2U
#define SPI_CR1_BR0        3U    /* BR[2:0] = bits 3-5 */
#define SPI_CR1_SPE        6U
#define SPI_CR1_LSBFIRST   7U
#define SPI_CR1_SSI        8U
#define SPI_CR1_SSM        9U
#define SPI_CR1_RXONLY    10U
#define SPI_CR1_DFF       11U
#define SPI_CR1_CRCNEXT   12U
#define SPI_CR1_CRCEN     13U
#define SPI_CR1_BIDIOE    14U
#define SPI_CR1_BIDIMODE  15U

/* CR2 bit positions */
#define SPI_CR2_RXDMAEN    0U
#define SPI_CR2_TXDMAEN    1U
#define SPI_CR2_SSOE       2U
#define SPI_CR2_FRF        4U
#define SPI_CR2_ERRIE      5U
#define SPI_CR2_RXNEIE     6U
#define SPI_CR2_TXEIE      7U

/* SR bit positions */
#define SPI_SR_RXNE        0U
#define SPI_SR_TXE         1U
#define SPI_SR_CHSIDE      2U
#define SPI_SR_UDR         3U
#define SPI_SR_CRCERR      4U
#define SPI_SR_MODF        5U
#define SPI_SR_OVR         6U
#define SPI_SR_BSY         7U
#define SPI_SR_FRE         8U

/* NVIC IRQ numbers */
#define SPI1_IRQ_NUMBER    35U
#define SPI2_IRQ_NUMBER    36U

#endif /* SPI_PRIVATE_H */
