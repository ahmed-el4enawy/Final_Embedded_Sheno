/**
 * Adc_Private.h
 *
 *  Created on: 4/19/2026
 *  Author    : AbdallahDarwish
 */

#ifndef ADC_PRIVATE_H
#define ADC_PRIVATE_H

#include "Std_Types.h"

/* ADC1 register map (STM32F401) */
typedef struct {
    volatile uint32 SR;          /* 0x00 - Status register                 */
    volatile uint32 CR1;         /* 0x04 - Control register 1              */
    volatile uint32 CR2;         /* 0x08 - Control register 2              */
    volatile uint32 SMPR1;       /* 0x0C - Sample time register 1 (ch10-18)*/
    volatile uint32 SMPR2;       /* 0x10 - Sample time register 2 (ch0-9) */
    volatile uint32 JOFR1;       /* 0x14 - Injected channel offset 1      */
    volatile uint32 JOFR2;       /* 0x18 - Injected channel offset 2      */
    volatile uint32 JOFR3;       /* 0x1C - Injected channel offset 3      */
    volatile uint32 JOFR4;       /* 0x20 - Injected channel offset 4      */
    volatile uint32 HTR;         /* 0x24 - Watchdog higher threshold       */
    volatile uint32 LTR;         /* 0x28 - Watchdog lower threshold        */
    volatile uint32 SQR1;        /* 0x2C - Regular sequence register 1     */
    volatile uint32 SQR2;        /* 0x30 - Regular sequence register 2     */
    volatile uint32 SQR3;        /* 0x34 - Regular sequence register 3     */
    volatile uint32 JSQR;        /* 0x38 - Injected sequence register      */
    volatile uint32 JDR1;        /* 0x3C - Injected data register 1        */
    volatile uint32 JDR2;        /* 0x40 - Injected data register 2        */
    volatile uint32 JDR3;        /* 0x44 - Injected data register 3        */
    volatile uint32 JDR4;        /* 0x48 - Injected data register 4        */
    volatile uint32 DR;          /* 0x4C - Regular data register           */
} AdcType;


typedef enum {
    ADC_ASYNC_STATE_IDLE       = 0U,
    ADC_ASYNC_STATE_SINGLE     = 1U,
    ADC_ASYNC_STATE_SCAN_GROUP = 2U
} Adc_AsyncStateType;

#define ADC1_BASE_ADDR          0x40012000UL

/* ADC Common register (shared, only ADC1 on STM32F401) */
#define ADC_CCR     (*(volatile uint32 *)(ADC1_BASE_ADDR + 0x04UL))

/*  SR bit positions  */
#define SR_AWD           0U     /* Analog watchdog flag                   */
#define SR_EOC           1U     /* End of conversion                      */
#define SR_JEOC          2U     /* Injected end of conversion             */
#define SR_JSTRT         3U     /* Injected channel start flag            */
#define SR_STRT          4U     /* Regular channel start flag             */
#define SR_OVR           5U     /* Overrun                                */

/*  CR1 bit positions  */
#define CR1_SCAN         8U     /* Scan mode                             */
#define CR1_JEOCIE       7U     /* Injected EOC interrupt enable         */
#define CR1_EOCIE        5U     /* EOC interrupt enable                  */
#define CR1_RES          24U    /* Resolution (2 bits: 24-25)            */

/*  CR2 bit positions  */
#define CR2_ADON         0U     /* A/D converter ON                       */
#define CR2_CONT         1U     /* Continuous conversion                  */
#define CR2_DMA          8U     /* DMA mode enable                        */
#define CR2_EOCS        10U     /* End of conversion selection            */
#define CR2_SWSTART     30U     /* Start conversion of regular channels   */

/*  Sample time values (3-bit, used in SMPR1/SMPR2)  */
#define SMPR_3_CYCLES    0x00U
#define SMPR_15_CYCLES   0x01U
#define SMPR_28_CYCLES   0x02U
#define SMPR_56_CYCLES   0x03U
#define SMPR_84_CYCLES   0x04U
#define SMPR_112_CYCLES  0x05U
#define SMPR_144_CYCLES  0x06U
#define SMPR_480_CYCLES  0x07U

/*  CCR bit positions  */
#define CCR_ADCPRE      16U     /* ADC prescaler (2 bits: 16-17)          */

#endif /* ADC_PRIVATE_H */
