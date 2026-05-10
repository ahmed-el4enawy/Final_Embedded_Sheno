/**
 * Spi.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Full-duplex SPI driver for STM32F401 (bare-metal).
 *  Master: blocking transfer with manual CS.
 *  Slave : interrupt-driven, non-blocking.
 */

#include "Spi.h"
#include "Spi_Private.h"
#include "Bit_Operations.h"
#include "Nvic.h"
#include "Gpio.h"
#include "Board_Config.h"
#include "Critical.h"

/* ------------------------------------------------------------------ */
/*  Private state                                                     */
/* ------------------------------------------------------------------ */

static uint32 Spi_BaseAddr[2] = { SPI1_BASE_ADDR, SPI2_BASE_ADDR };

#define SPI_PERIPH(id)  ((SpiType *)Spi_BaseAddr[(id) - 1])

/* Slave async context */
static volatile uint8  *Spi_SlaveTxBuf   = 0;
static volatile uint8   Spi_SlaveTxLen   = 0;
static volatile uint8   Spi_SlaveTxIdx   = 0;

static volatile uint8  *Spi_SlaveRxBuf   = 0;
static volatile uint8   Spi_SlaveRxLen   = 0;
static volatile uint8   Spi_SlaveRxIdx   = 0;

static SpiRxCallback    Spi_SlaveCallback = 0;

/* ------------------------------------------------------------------ */
/*  CS helpers (PA4 as manual GPIO)                                   */
/* ------------------------------------------------------------------ */
void Spi_CsLow(void) {
    Gpio_WritePin(SPI_CS_PORT, SPI_CS_PIN, LOW);
}

void Spi_CsHigh(void) {
    Gpio_WritePin(SPI_CS_PORT, SPI_CS_PIN, HIGH);
}

/* ------------------------------------------------------------------ */
/*  Init                                                              */
/* ------------------------------------------------------------------ */
void Spi_Init(uint8 SpiId, uint8 Mode, uint8 BaudDiv) {
    SpiType *spi = SPI_PERIPH(SpiId);

    /* Disable SPI while configuring */
    spi->CR1 = 0;
    spi->CR2 = 0;

    if (Mode == SPI_MODE_MASTER) {
        /* CS pin as GPIO output, default high */
        Gpio_Init(SPI_CS_PORT, SPI_CS_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL);
        Spi_CsHigh();

        spi->CR1 = (1U << SPI_CR1_MSTR)         /* Master mode              */
                  | (1U << SPI_CR1_SSM)          /* Software slave mgmt      */
                  | (1U << SPI_CR1_SSI)          /* Internal SS high         */
                  | ((uint32)BaudDiv << SPI_CR1_BR0); /* Baud rate           */
        /* CPOL=0, CPHA=0, 8-bit, MSB first (defaults) */

    } else {
        /* Slave mode: hardware NSS or software */
        spi->CR1 = (1U << SPI_CR1_SSM);  /* SSM=1, SSI=0 → slave           */
        /* CPOL=0, CPHA=0, 8-bit, MSB first */
    }

    /* Enable SPI peripheral */
    SET_BIT(spi->CR1, SPI_CR1_SPE);
}

/* ------------------------------------------------------------------ */
/*  Master: full-duplex blocking transfer                             */
/* ------------------------------------------------------------------ */
void Spi_TransmitReceive(uint8 SpiId, const uint8 *TxBuf, uint8 *RxBuf, uint8 Length) {
    SpiType *spi = SPI_PERIPH(SpiId);
    uint8 i;
    volatile uint8 dummy;

    /* Clear any old OVR by reading DR then SR */
    dummy = (uint8)spi->DR;
    dummy = (uint8)spi->SR;
    (void)dummy;

    for (i = 0; i < Length; i++) {
        /* Wait until TXE = 1 */
        while (!READ_BIT(spi->SR, SPI_SR_TXE)) {}

        /* Write byte to DR → clocks out on MOSI, clocks in on MISO */
        spi->DR = TxBuf[i];

        /* Wait until RXNE = 1 */
        while (!READ_BIT(spi->SR, SPI_SR_RXNE)) {}

        RxBuf[i] = (uint8)spi->DR;
    }

    /* Wait until not busy */
    while (READ_BIT(spi->SR, SPI_SR_BSY)) {}
}

/* ------------------------------------------------------------------ */
/*  Slave: pre-load TX buffer                                         */
/* ------------------------------------------------------------------ */
void Spi_SlavePreload(uint8 SpiId, const uint8 *TxBuf, uint8 Length) {
    uint32 pm = Enter_Critical();
    Spi_SlaveTxBuf = TxBuf;
    Spi_SlaveTxLen = Length;
    Spi_SlaveTxIdx = 1;   /* byte 0 goes into DR now */
    Spi_SlaveRxIdx = 0;

    /* Pre-load first byte into DR so it's ready when master clocks */
    SpiType *spi = SPI_PERIPH(SpiId);
    while (!READ_BIT(spi->SR, SPI_SR_TXE)) {}
    spi->DR = TxBuf[0];

    Exit_Critical(pm);
}

/* ------------------------------------------------------------------ */
/*  Slave: start async reception with RXNE interrupt                  */
/* ------------------------------------------------------------------ */
void Spi_SlaveStartAsync(uint8 SpiId, SpiRxCallback Callback,
                          uint8 *RxBuf, uint8 Length) {
    SpiType *spi = SPI_PERIPH(SpiId);

    Spi_SlaveCallback = Callback;
    Spi_SlaveRxBuf    = RxBuf;
    Spi_SlaveRxLen    = Length;
    Spi_SlaveRxIdx    = 0;

    /* Enable RXNE interrupt */
    SET_BIT(spi->CR2, SPI_CR2_RXNEIE);

    uint8 irq = (SpiId == SPI_1) ? SPI1_IRQ_NUMBER : SPI2_IRQ_NUMBER;
    Nvic_EnableIrq(irq);
}

/* ------------------------------------------------------------------ */
/*  SPI1 IRQ Handler (Slave non-blocking driver)                      */
/* ------------------------------------------------------------------ */
void SPI1_IRQHandler(void) {
    SpiType *spi = (SpiType *)SPI1_BASE_ADDR;

    if (READ_BIT(spi->SR, SPI_SR_RXNE)) {
        uint8 rxByte = (uint8)spi->DR;

        /* Store received byte */
        if (Spi_SlaveRxBuf && Spi_SlaveRxIdx < Spi_SlaveRxLen) {
            Spi_SlaveRxBuf[Spi_SlaveRxIdx] = rxByte;
            Spi_SlaveRxIdx++;
        }

        /* Load next TX byte (if available) */
        if (Spi_SlaveTxBuf && Spi_SlaveTxIdx < Spi_SlaveTxLen) {
            spi->DR = Spi_SlaveTxBuf[Spi_SlaveTxIdx];
            Spi_SlaveTxIdx++;
        }

        /* Frame complete? */
        if (Spi_SlaveRxIdx >= Spi_SlaveRxLen) {
            Spi_SlaveRxIdx = 0;
            Spi_SlaveTxIdx = 0;

            /* Re-preload first TX byte for next frame */
            if (Spi_SlaveTxBuf && Spi_SlaveTxLen > 0) {
                Spi_SlaveTxIdx = 1;
                while (!READ_BIT(spi->SR, SPI_SR_TXE)) {}
                spi->DR = Spi_SlaveTxBuf[0];
            }

            if (Spi_SlaveCallback) {
                Spi_SlaveCallback((uint8 *)Spi_SlaveRxBuf, Spi_SlaveRxLen);
            }
        }
    }

    /* Clear OVR if set (read DR then SR) */
    if (READ_BIT(spi->SR, SPI_SR_OVR)) {
        volatile uint8 d = (uint8)spi->DR;
        d = (uint8)spi->SR;
        (void)d;
    }
}
