/*********************************************************************************

Copyright(c) 2013-2017 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.

*********************************************************************************/
/*!
 * @file      crypto_sha.h
 * @brief     Example to demonstrate Crypto driver
 *
 * @details
 *            This is the primary include file for the Crypto example.
 *
 */

/*=============  I N C L U D E S   =============*/

/* Crypto Driver includes */
#include <drivers/crypto/adi_crypto.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* pick up compiler-specific alignment directives */
#include <drivers/general/adi_drivers_general.h>

/*=============  D E F I N E S  =============*/

/* Enable macro to build example in callback mode */
#define CRYPTO_ENABLE_CALLBACK

/* CRYPTO Device number */
#define CRYPTO_DEV_NUM               (0u)

/* SysTick Cycle Count Macros... max 24-bit measure (no wraparound handling) */
#define CYCLES_INIT {                                                     \
    /* enable with internal clock and no interrupts */                    \
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; \
    SysTick->LOAD = 0x00FFFFFF;}                                     /*!< SysTick instruction count macro */
#define CYCLES_CLR     {SysTick->VAL = 0;}                           /*!< SysTick instruction count macro */
#define CYCLES_GET     (0x00ffffff + 1 - SysTick->VAL)               /*!< SysTick instruction count macro */
#define CYCLES_SUSPEND {SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;}  /*!< SysTick instruction count macro */
#define CYCLES_RESUME  {SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;}   /*!< SysTick instruction count macro */

/*****/
