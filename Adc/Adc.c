/**
 * Adc.c
 *
 *  Created on: 4/19/2026
 *  Author    : AbdallahDarwish
 */

#include "Adc.h"

#include "stddef.h"

#include "Adc_Private.h"
#include "Bit_Operations.h"
#include "Nvic.h"


#define ADC1              ((AdcType *)ADC1_BASE_ADDR)
#define ADC_IRQ_NUMBER    18U


static volatile Adc_AsyncStateType Adc_CurrentAsyncState = ADC_ASYNC_STATE_IDLE;

/*  state for async scan-group read  */
static uint16* Adc_AsyncBuf = NULL;
static uint8 Adc_AsyncTotal = 0;
static uint8 Adc_AsyncIndex = 0;
static AdcMultiChannelCallback Adc_AsyncCallback = NULL;

/*  state for async single-channel read  */
static AdcSingleChannelCallback Adc_SingleCallback = NULL;

/* ------------------------------------------------------------------ */
/*  Helper: set sample time for a channel (84 cycles)                 */
/* ------------------------------------------------------------------ */
static void Adc_SetSampleTime(uint8 Channel)
{
    if (Channel < 10)
    {
        ADC1->SMPR2 &= ~(0x07UL << (Channel * 3));
        ADC1->SMPR2 |= ((uint32)SMPR_84_CYCLES << (Channel * 3));
    }
    else
    {
        ADC1->SMPR1 &= ~(0x07UL << ((Channel - 10) * 3));
        ADC1->SMPR1 |= (SMPR_84_CYCLES << ((Channel - 10) * 3));
    }
}

/* ------------------------------------------------------------------ */
/*  Helper: set a channel into the regular sequence at position index */
/* ------------------------------------------------------------------ */
static void Adc_SetSequence(uint8 Index, uint8 Channel)
{
    if (Index < 6)
    {
        ADC1->SQR3 |= ((uint32)Channel << (Index * 5));
    }
    else if (Index < 12)
    {
        ADC1->SQR2 |= ((uint32)Channel << ((Index - 6) * 5));
    }
    else
    {
        ADC1->SQR1 |= ((uint32)Channel << ((Index - 12) * 5));
    }
}

void Adc_Init(uint8 Resolution)
{
    /* ADC prescaler  PCLK2 / 2  (16 MHz / 2 = 8 MHz ADC clock) */
    ADC_CCR &= ~(0x03UL << CCR_ADCPRE);

    /* Set resolution (bits 25:24 of CR1) */
    ADC1->CR1 &= ~(0x03UL << CR1_RES);
    ADC1->CR1 |= ((uint32)Resolution << CR1_RES);

    SET_BIT(ADC1->CR2, CR2_ADON);
}

void Adc_StartConversion(void)
{
    ADC1->SR = 0;
    SET_BIT(ADC1->CR2, CR2_SWSTART);
}

void Adc_StopConversion(void)
{
    CLEAR_BIT(ADC1->CR2, CR2_ADON);
}

/* ================================================================== */
/*               SINGLE MODE  (one channel, one-shot)                 */
/* ================================================================== */

void Adc_ConfigSingleChannel_OneShot(uint8 Channel)
{
    /* Disable scan & continuous */
    CLEAR_BIT(ADC1->CR1, CR1_SCAN);
    CLEAR_BIT(ADC1->CR2, CR2_CONT);
    SET_BIT(ADC1->CR2, CR2_EOCS);

    /* 1 conversion: L = 0 */
    ADC1->SQR1 &= ~(0x0FUL << 20);

    /* Place channel in SQ1 (bits 4:0 of SQR3) */
    ADC1->SQR3 &= ~(0x1FUL << 0);
    ADC1->SQR3 |= (uint32)Channel;

    Adc_SetSampleTime(Channel);

    SET_BIT(ADC1->CR2, CR2_ADON);
}

uint16 Adc_ReadSingleChannel(void)
{
    /* Wait for EOC */
    while (!READ_BIT(ADC1->SR, SR_EOC))
    {
        /* poll */
    }
    return (uint16)(ADC1->DR & 0xFFFFU);
}

void Adc_ReadSingleChannelAsync(AdcSingleChannelCallback Callback)
{
    /* Clear scan-async state so ISR uses single path */
    Adc_AsyncBuf = 0;
    Adc_AsyncTotal = 0;
    Adc_AsyncIndex = 0;

    /* Store callback */
    Adc_SingleCallback = Callback;
    Adc_CurrentAsyncState = ADC_ASYNC_STATE_SINGLE;

    /* Enable EOC interrupt */
    SET_BIT(ADC1->CR1, CR1_EOCIE);
    Nvic_EnableIrq(ADC_IRQ_NUMBER);
}

/* ================================================================== */
/*                   SCAN MODE  (multi-channel)                       */
/* ================================================================== */


/**
 * @brief  Configure Scan Mode with Continuous Conversion.
 *         Sets up the regular-channel sequence and sample times.
 * @param  Channels     Array of channel numbers (e.g. ADC_CHANNEL_0)
 * @param  NumChannels  Number of channels in the array (1-16)
 */
void Adc_ConfigScanGroup_Continuous(uint8* Channels, uint8 NumChannels)
{
    uint8 i;

    /* Enable Scan mode */
    SET_BIT(ADC1->CR1, CR1_SCAN);

    /* Enable Continuous conversion */  
    CLEAR_BIT(ADC1->CR2, CR2_CONT);

    /* EOCS = 1  →  EOC flag after *each* channel (not after sequence) */
    SET_BIT(ADC1->CR2, CR2_EOCS);

    /* Number of conversions: L[3:0] = NumChannels - 1  (bits 23:20) */
    ADC1->SQR1 &= ~(0x0FUL << 20);
    ADC1->SQR1 |= ((uint32)(NumChannels - 1) << 20);

    /* Clear sequence registers */
    ADC1->SQR3 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR1 &= (0x0FUL << 20); /* keep L bits only */

    for (i = 0; i < NumChannels; i++)
    {
        Adc_SetSequence(i, Channels[i]);
        Adc_SetSampleTime(Channels[i]);
    }
}

/* Synchronous read */
void Adc_ScanChannelGroup(uint16* Results, uint8 NumChannels)
{
    uint8 i;
    for (i = 0; i < NumChannels; i++)
    {
        while (!READ_BIT(ADC1->SR, SR_EOC))
        {
            /* poll */
        }
        Results[i] = (uint16)(ADC1->DR & 0xFFFFU);
    }
}

/*  Asynchronous read  */
void Adc_ScanChannelGroupAsync(uint16* Results, uint8 NumChannels,
                               AdcMultiChannelCallback Callback)
{
    /* Store callback context */
    Adc_AsyncBuf = Results;
    Adc_AsyncTotal = NumChannels;
    Adc_AsyncIndex = 0;
    Adc_AsyncCallback = Callback;
    Adc_CurrentAsyncState = ADC_ASYNC_STATE_SCAN_GROUP;

    /* Enable EOC interrupt → ISR will collect each result */
    SET_BIT(ADC1->CR1, CR1_EOCIE);
    Nvic_EnableIrq(ADC_IRQ_NUMBER);
    SET_BIT(ADC1->CR2, CR2_ADON);
}

/* ------------------------------------------------------------------ */
/*  ADC ISR — collects one result per EOC, calls back when done       */
/* ------------------------------------------------------------------ */
void ADC_IRQHandler(void)
{
    if (READ_BIT(ADC1->SR, SR_EOC))
    {
        if (Adc_CurrentAsyncState == ADC_ASYNC_STATE_SCAN_GROUP)
        {
            if (Adc_AsyncTotal > 0 && Adc_AsyncBuf != 0)
            {
                /* ---- Scan-group async mode ---- */
                if (Adc_AsyncIndex < Adc_AsyncTotal)
                {
                    Adc_AsyncBuf[Adc_AsyncIndex] = (uint16)(ADC1->DR & 0xFFFFU);
                    Adc_AsyncIndex++;
                }
                if (Adc_AsyncIndex >= Adc_AsyncTotal)
                {
                    // CLEAR_BIT(ADC1->CR1, CR1_EOCIE);
                    // ADC1->SR = 0;
                    if (Adc_AsyncCallback != NULL)
                    {
                        Adc_AsyncCallback(Adc_AsyncBuf, Adc_AsyncTotal);
                    }
                }
            }
            SET_BIT(ADC1->CR2, CR2_ADON);
        }
        else if (Adc_CurrentAsyncState == ADC_ASYNC_STATE_SINGLE)
        {
            if (Adc_SingleCallback != NULL)
            {
                /*  Single-channel async mode  */
                uint16 result = (uint16)(ADC1->DR & 0xFFFFU);

                // CLEAR_BIT(ADC1->CR1, CR1_EOCIE);
                ADC1->SR = 0;

                Adc_SingleCallback(result);
            }
        }
        else
        {
            // error or idle
            ADC1->SR = 0;
        }
    }
}
