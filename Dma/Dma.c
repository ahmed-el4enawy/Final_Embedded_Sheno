/**
 * Dma.c
 *
 *  Created on: 5/10/2026
 *  Author    : Final Project
 *
 *  DMA2 Stream7 Channel4 → USART1_TX  (bonus +5 points).
 *  Memory-to-peripheral, single transfer, auto-complete.
 */

#include "Dma.h"
#include "Bit_Operations.h"
#include "Std_Types.h"

/* ------------------------------------------------------------------ */
/*  Register map                                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    volatile uint32 CR;        /* 0x00 - Stream config               */
    volatile uint32 NDTR;      /* 0x04 - Number of data items        */
    volatile uint32 PAR;       /* 0x08 - Peripheral address          */
    volatile uint32 M0AR;      /* 0x0C - Memory 0 address            */
    volatile uint32 M1AR;      /* 0x10 - Memory 1 address            */
    volatile uint32 FCR;       /* 0x14 - FIFO control                */
} DmaStreamType;

typedef struct {
    volatile uint32 LISR;      /* 0x00 */
    volatile uint32 HISR;      /* 0x04 */
    volatile uint32 LIFCR;     /* 0x08 */
    volatile uint32 HIFCR;     /* 0x0C */
} DmaType;

#define DMA2_BASE          0x40026400UL
#define DMA2              ((DmaType *)DMA2_BASE)
/* Stream 7 offset = 0x10 + 7*0x18 = 0x10 + 0xA8 = 0xB8 */
#define DMA2_STREAM7      ((DmaStreamType *)(DMA2_BASE + 0x10UL + 7UL * 0x18UL))

/* USART1 DR address */
#define USART1_DR_ADDR     0x40011004UL

/* CR bit positions */
#define DMA_CR_EN           0U
#define DMA_CR_DMEIE        1U
#define DMA_CR_TEIE         2U
#define DMA_CR_HTIE         3U
#define DMA_CR_TCIE         4U
#define DMA_CR_PFCTRL       5U
#define DMA_CR_DIR_POS      6U     /* 2 bits */
#define DMA_CR_CIRC         8U
#define DMA_CR_PINC         9U
#define DMA_CR_MINC        10U
#define DMA_CR_PSIZE_POS   11U     /* 2 bits */
#define DMA_CR_MSIZE_POS   13U     /* 2 bits */
#define DMA_CR_PL_POS      16U     /* 2 bits */
#define DMA_CR_CHSEL_POS   25U     /* 3 bits */

/* Direction values */
#define DMA_DIR_MEM_TO_PERIPH  1U

/* HISR / HIFCR bit positions for Stream 7 */
#define DMA_HISR_TCIF7     27U
#define DMA_HISR_HTIF7     26U
#define DMA_HISR_TEIF7     25U
#define DMA_HISR_DMEIF7    24U
#define DMA_HISR_FEIF7     22U

/* ------------------------------------------------------------------ */
void Dma_Usart1TxInit(void) {
    /* Disable stream before configuring */
    DMA2_STREAM7->CR &= ~(1UL << DMA_CR_EN);
    while (DMA2_STREAM7->CR & (1UL << DMA_CR_EN)) {}

    /* Clear all Stream 7 interrupt flags (HIFCR bits 22-27) */
    DMA2->HIFCR = (1UL << DMA_HISR_TCIF7) | (1UL << DMA_HISR_HTIF7)
                | (1UL << DMA_HISR_TEIF7)  | (1UL << DMA_HISR_DMEIF7)
                | (1UL << DMA_HISR_FEIF7);

    /* Configure CR:
     *  Channel 4        (CHSEL = 4)
     *  Memory-to-Periph (DIR   = 01)
     *  Memory increment (MINC  = 1)
     *  Byte size        (PSIZE = 00, MSIZE = 00)
     *  Medium priority  (PL    = 01)
     *  No circular mode
     */
    DMA2_STREAM7->CR = (4UL << DMA_CR_CHSEL_POS)       /* Channel 4      */
                     | (DMA_DIR_MEM_TO_PERIPH << DMA_CR_DIR_POS)
                     | (1UL << DMA_CR_MINC)             /* Memory inc     */
                     | (1UL << DMA_CR_PL_POS);          /* Priority med   */

    /* Peripheral address = USART1->DR */
    DMA2_STREAM7->PAR = USART1_DR_ADDR;

    /* Disable FIFO (direct mode) */
    DMA2_STREAM7->FCR = 0;
}

/* ------------------------------------------------------------------ */
void Dma_Usart1TxStart(const uint8 *data, uint16 len) {
    /* Wait if a previous transfer is still running */
    while (DMA2_STREAM7->CR & (1UL << DMA_CR_EN)) {}

    /* Clear flags */
    DMA2->HIFCR = (1UL << DMA_HISR_TCIF7) | (1UL << DMA_HISR_HTIF7)
                | (1UL << DMA_HISR_TEIF7)  | (1UL << DMA_HISR_DMEIF7)
                | (1UL << DMA_HISR_FEIF7);

    DMA2_STREAM7->M0AR = (uint32)data;
    DMA2_STREAM7->NDTR = len;

    /* Enable stream */
    SET_BIT(DMA2_STREAM7->CR, DMA_CR_EN);

    /* Enable USART1 DMA TX request (bit 7 of USART CR3) */
    /* USART1 CR3 is at offset 0x14 from USART1 base 0x40011000 */
    volatile uint32 *usart1_cr3 = (volatile uint32 *)0x40011014UL;
    SET_BIT(*usart1_cr3, 7);   /* DMAT bit */
}

/* ------------------------------------------------------------------ */
boolean Dma_Usart1TxBusy(void) {
    if (DMA2_STREAM7->CR & (1UL << DMA_CR_EN)) {
        return TRUE;
    }
    if (DMA2->HISR & (1UL << DMA_HISR_TCIF7)) {
        /* Transfer complete, clear flag */
        DMA2->HIFCR = (1UL << DMA_HISR_TCIF7);
        return FALSE;
    }
    return FALSE;
}
