/**
 * Adc.h
 *
 *  Created on: 4/19/2026
 *  Author    : AbdallahDarwish
 */

#ifndef ADC_H
#define ADC_H

#include "Std_Types.h"

/* ADC Channels */
#define ADC_CHANNEL_0    0U
#define ADC_CHANNEL_1    1U
#define ADC_CHANNEL_2    2U
#define ADC_CHANNEL_3    3U
#define ADC_CHANNEL_4    4U
#define ADC_CHANNEL_5    5U
#define ADC_CHANNEL_6    6U
#define ADC_CHANNEL_7    7U
#define ADC_CHANNEL_8    8U
#define ADC_CHANNEL_9    9U
#define ADC_CHANNEL_10  10U
#define ADC_CHANNEL_11  11U
#define ADC_CHANNEL_12  12U
#define ADC_CHANNEL_13  13U
#define ADC_CHANNEL_14  14U
#define ADC_CHANNEL_15  15U

/* ADC Resolution options */
#define ADC_RES_12BIT    0x00U
#define ADC_RES_10BIT    0x01U
#define ADC_RES_8BIT     0x02U
#define ADC_RES_6BIT     0x03U



/* Callback types for async operations */
typedef void (*AdcMultiChannelCallback)(uint16 *Results, uint8 NumChannels);
typedef void (*AdcSingleChannelCallback)(uint16 Result);

/**
 * @brief  Initialize ADC1: turn on, set resolution, set prescaler.
 * @param  Resolution  ADC_RES_12BIT / 10 / 8 / 6
 */
void Adc_Init(uint8 Resolution);

/**
 * @brief  Start (or restart) ADC conversion.
 *         Clears flags then sets SWSTART.
 */
void Adc_StartConversion(void);

/**
 * @brief  Stop ADC conversion (disables CONT, then ADON).
 */
void Adc_StopConversion(void);

/* ============================================================== */
/*               SINGLE MODE ->  (one channel, one-shot)          */
/* ============================================================== */

/**
 * @brief  Configure ADC for Single-channel, single-conversion mode.
 *         Disables SCAN and CONT.  Sets the channel and sample time.
 * @param  Channel  ADC_CHANNEL_x
 */
void Adc_ConfigSingleChannel_OneShot(uint8 Channel);

/**
 * @brief  (Synchronous) Trigger one conversion and return the result.
 *         Must call Adc_ConfigSingleMode() first.
 * @return Conversion result (12/10/8/6-bit depending on resolution)
 */
uint16 Adc_ReadSingleChannel(void);

/**
 * @brief  (Asynchronous) Trigger one conversion and return result via callback.
 *         
 * @param  Callback  Function called when conversion is ready (from ISR)
 */
void Adc_ReadSingleChannelAsync(AdcSingleChannelCallback Callback);


/* ============================================================== */
/*                     SCAN MODE  (multi-channel, continous)      */
/* ============================================================== */

/**
 * @brief  Configure Scan Mode with Continuous Conversion.
 *         Sets up the regular-channel sequence and sample times.
 * @param  Channels     Array of channel numbers (e.g. ADC_CHANNEL_0)
 * @param  NumChannels  Number of channels in the array (1-16)
 */
void Adc_ConfigScanGroup_Continuous(uint8 *Channels, uint8 NumChannels);


/**
 * @brief  (Synchronous) Read one complete scan group by polling EOC
 *         for each channel.  Blocks until all channels are read.
 * @param  Results       Pointer to a uint16 array (caller-allocated)
 * @param  NumChannels   Number of channels to read
 */
void Adc_ScanChannelGroup(uint16 *Results, uint8 NumChannels);

/**
 * @brief  (Asynchronous) Start a scan-group read using the EOC interrupt.
 *         The provided callback is invoked (from ISR context) once all
 *         channels have been collected.
 * @param  Results       Pointer to a uint16 array (caller-allocated,
 *                       must remain valid until callback fires)
 * @param  NumChannels   Number of channels to read
 * @param  Callback      Function called when all channels are ready
 */
void Adc_ScanChannelGroupAsync(uint16 *Results, uint8 NumChannels,
                            AdcMultiChannelCallback Callback);

#endif /* ADC_H */
