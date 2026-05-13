/**
 * Spi.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  Full-duplex SPI driver for STM32F401 (bare-metal).
 *
 *  [REWRITE — Non-Blocking Master]
 *  Both Master and Slave now use interrupt-driven, non-blocking transfers.
 *
 *  Master: Spi_MasterTransferAsync() enables TXEIE/RXNEIE interrupts.
 *          The ISR feeds bytes from TxBuf and collects into RxBuf.
 *          On frame completion, a user callback fires and CS is raised.
 *          The old blocking Spi_TransmitReceive() is kept for boot-time
 *          one-shot use only (startup message) but is NOT used during
 *          the main loop.
 *
 *  Slave:  Interrupt-driven via RXNEIE (unchanged from previous version).
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

/* ======================== Slave async context ===================== */
static volatile const uint8  *Spi_SlaveTxBuf   = 0;
static volatile uint8   Spi_SlaveTxLen   = 0;
static volatile uint8   Spi_SlaveTxIdx   = 0;

static volatile uint8  *Spi_SlaveRxBuf   = 0;
static volatile uint8   Spi_SlaveRxLen   = 0;
static volatile uint8   Spi_SlaveRxIdx   = 0;

static SpiRxCallback    Spi_SlaveCallback = 0;

/* ======================== Master async context ==================== */
/* [NON-BLOCKING MASTER]
 * The Master ISR feeds TxBuf bytes into DR on TXE, and reads RxBuf
 * bytes from DR on RXNE.  When all bytes are exchanged, the ISR
 * disables TXEIE/RXNEIE and invokes the user callback. */
static const uint8     *Spi_MasterTxBuf     = 0;
static       uint8     *Spi_MasterRxBuf     = 0;
static volatile uint8   Spi_MasterLen       = 0;
static volatile uint8   Spi_MasterTxIdx     = 0;
static volatile uint8   Spi_MasterRxIdx     = 0;
static SpiRxCallback    Spi_MasterCallback  = 0;
static volatile uint8   Spi_MasterMode      = 0; /* 0=idle, 1=active */

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

        /* [NON-BLOCKING MASTER] Enable NVIC for SPI so the Master ISR
         * can fire when TXEIE/RXNEIE are later enabled per-transfer. */
        uint8 irq = (SpiId == SPI_1) ? IRQ_SPI1 : IRQ_SPI1; /* simplified as only SPI1 used */
        Nvic_EnableIrq(irq);

    } else {
        /* Slave mode: HARDWARE NSS (SSM=0).
         * PA4 = SPI1_NSS (AF5) — Proteus requires the physical NSS
         * pin for the slave SPI model to function. */
        Gpio_Init(SPI_CS_PORT, SPI_CS_PIN, GPIO_AF, GPIO_PUSH_PULL);
        Gpio_SetAF(SPI_CS_PORT, SPI_CS_PIN, SPI_AF);  /* AF5 */
        spi->CR1 = 0;  /* SSM=0, MSTR=0 → hardware NSS slave */

        /* Enable NVIC for slave ISR-driven reception */
        uint8 irq = (SpiId == SPI_1) ? IRQ_SPI1 : IRQ_SPI1;
        Nvic_EnableIrq(irq);
    }

    /* Enable SPI peripheral */
    SET_BIT(spi->CR1, SPI_CR1_SPE);
}

/* ------------------------------------------------------------------ */
/*  Master: full-duplex BLOCKING (polling) transfer.                   */
/*  Guaranteed to work in Proteus — no dependency on SPI interrupts.   */
/* ------------------------------------------------------------------ */
void Spi_TransmitReceive(uint8 SpiId, const uint8 *TxBuf, uint8 *RxBuf, uint8 Length) {
    SpiType *spi = SPI_PERIPH(SpiId);

    /* Clear any old OVR by reading DR then SR */
    { volatile uint8 d = (uint8)spi->DR; d = (uint8)spi->SR; (void)d; }

    Spi_CsLow();

    for (uint8 i = 0; i < Length; i++) {
        /* Wait until TX buffer empty */
        while (!READ_BIT(spi->SR, SPI_SR_TXE)) {}
        spi->DR = TxBuf[i];

        /* Wait until RX byte ready */
        while (!READ_BIT(spi->SR, SPI_SR_RXNE)) {}
        RxBuf[i] = (uint8)spi->DR;

        /* Wait for shift register to finish — critical for Proteus!
         * Without this, the master starts the next byte before the
         * slave MCU's RXNE ISR has run, causing OVR on the slave
         * and corrupting the slave's TX response data. */
        while (READ_BIT(spi->SR, SPI_SR_BSY)) {}

        /* Extra delay for Proteus dual-MCU simulation timing.
         * Gives the slave MCU enough simulation cycles to detect
         * RXNE in its main loop and enter the tight receive loop.
         * 800 iterations ≈ 200 µs — total 8-byte transfer ≈ 1.6 ms. */
        { volatile uint32 d; for (d = 0; d < 800; d++) {} }
    }

    Spi_CsHigh();
}

/* ------------------------------------------------------------------ */
/*  [NON-BLOCKING MASTER]                                             */
/*  Master: full-duplex interrupt-driven transfer                     */
/*                                                                     */
/*  Initiates an 8-byte exchange by enabling TXEIE.  The SPI1 ISR    */
/*  feeds bytes on TXE and collects on RXNE.  When all bytes are      */
/*  received, the ISR deasserts CS, disables interrupts, and invokes  */
/*  the user Callback(RxBuf, Length).                                  */
/*                                                                     */
/*  The main loop only needs to set a flag and return — zero CPU      */
/*  time spent polling SPI status registers.                          */
/* ------------------------------------------------------------------ */
void Spi_MasterTransferAsync(uint8 SpiId, const uint8 *TxBuf, uint8 *RxBuf,
                              uint8 Length, SpiRxCallback Callback) {
    SpiType *spi = SPI_PERIPH(SpiId);

    uint32 pm = Enter_Critical();

    /* Store transfer context */
    Spi_MasterTxBuf    = TxBuf;
    Spi_MasterRxBuf    = RxBuf;
    Spi_MasterLen      = Length;
    Spi_MasterTxIdx    = 0;
    Spi_MasterRxIdx    = 0;
    Spi_MasterCallback = Callback;
    Spi_MasterMode     = 1;

    /* Clear any old OVR by reading DR then SR */
    { volatile uint8 d = (uint8)spi->DR; d = (uint8)spi->SR; (void)d; }

    /* Assert CS */
    Spi_CsLow();

    /* Enable TXE and RXNE interrupts — ISR will feed bytes */
    SET_BIT(spi->CR2, SPI_CR2_TXEIE);
    SET_BIT(spi->CR2, SPI_CR2_RXNEIE);

    Exit_Critical(pm);
}

/* ------------------------------------------------------------------ */
/*  Check if a master async transfer is in progress                   */
/* ------------------------------------------------------------------ */
boolean Spi_MasterBusy(void) {
    return Spi_MasterMode ? TRUE : FALSE;
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

    /* Pre-load first byte into DR if TXE is ready */
    SpiType *spi = SPI_PERIPH(SpiId);
    if (READ_BIT(spi->SR, SPI_SR_TXE)) {
        spi->DR = TxBuf[0];
    }

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

    uint8 irq = (SpiId == SPI_1) ? IRQ_SPI1 : IRQ_SPI1;
    Nvic_EnableIrq(irq);
}

/* ------------------------------------------------------------------ */
/*  SPI1 IRQ Handler                                                  */
/*                                                                     */
/*  Shared between Master (non-blocking transfer) and Slave (async    */
/*  reception).  The Spi_MasterMode flag disambiguates.               */
/* ------------------------------------------------------------------ */
void SPI1_IRQHandler(void) {
    SpiType *spi = (SpiType *)SPI1_BASE_ADDR;

    /* ============================================================== */
    /*  MASTER interrupt-driven transfer                              */
    /* ============================================================== */
    if (Spi_MasterMode) {
        /* ---- TXE: feed next TX byte ---- */
        if (READ_BIT(spi->CR2, SPI_CR2_TXEIE) && READ_BIT(spi->SR, SPI_SR_TXE)) {
            if (Spi_MasterTxIdx < Spi_MasterLen) {
                spi->DR = Spi_MasterTxBuf[Spi_MasterTxIdx];
                Spi_MasterTxIdx++;
            }
            if (Spi_MasterTxIdx >= Spi_MasterLen) {
                /* All TX bytes loaded — disable TXE interrupt */
                CLEAR_BIT(spi->CR2, SPI_CR2_TXEIE);
            }
        }

        /* ---- RXNE: collect RX byte ---- */
        if (READ_BIT(spi->SR, SPI_SR_RXNE)) {
            uint8 rxByte = (uint8)spi->DR;
            if (Spi_MasterRxIdx < Spi_MasterLen) {
                Spi_MasterRxBuf[Spi_MasterRxIdx] = rxByte;
                Spi_MasterRxIdx++;
            }

            /* Frame complete? */
            if (Spi_MasterRxIdx >= Spi_MasterLen) {
                /* Disable RXNE interrupt */
                CLEAR_BIT(spi->CR2, SPI_CR2_RXNEIE);

                /* [NON-BLOCKING] Last RXNE implies shifting is done. 
                 * Small hardware latency exists but CS can be raised. */

                /* Deassert CS */
                Spi_CsHigh();

                Spi_MasterMode = 0;

                /* Invoke user callback */
                if (Spi_MasterCallback) {
                    Spi_MasterCallback(Spi_MasterRxBuf, Spi_MasterLen);
                }
            }
        }
        return;   /* Master handled — do not fall through to Slave */
    }

    /* ============================================================== */
    /*  SLAVE interrupt-driven reception (unchanged)                  */
    /* ============================================================== */
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

                /* Re-preload first TX byte if ready */
                if (Spi_SlaveTxBuf && Spi_SlaveTxLen > 0) {
                    Spi_SlaveTxIdx = 1;
                    if (READ_BIT(spi->SR, SPI_SR_TXE)) {
                        spi->DR = Spi_SlaveTxBuf[0];
                    }
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
