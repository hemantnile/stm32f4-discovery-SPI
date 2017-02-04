/**
 ******************************************************************************
 * @file    inc/main.h 
 * @author  Hemant Nile
 * @version V1
 * @date    04-Feb-2017
 * @brief   Header for main.c module
 
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#include "common.h"

#define PLL_M				8
#define PLL_N				336
#define PLL_P				2
#define PLL_Q				7
#define AHB_PRE			RCC_CFGR_HPRE_DIV1
#define APB1_PRE		RCC_CFGR_PPRE1_DIV4	
#define APB2_PRE		RCC_CFGR_PPRE2_DIV2		

void Error_Handler(void);
void SPI_Tx(uint8_t address, uint8_t data);
uint8_t SPI_Rx(uint8_t address);
void SPI1_TransferError_Callback(void);

#endif /* __MAIN_H */
