/** \internal
 *--------------------------------------------------------------------------------------*
 *                                                                                      *
 *  COPYRIGHT(C) 2023 Shengkong Intelligent Technology Co.   Ltd                        *
 *                                                                                      *
 *--------------------------------------------------------------------------------------*
 * Description: key head  file                                                          *
 *--------------------------------------------------------------------------------------*
 * History:                                                                             *
 *  <Date>   <Author>  <Description>                                                    *
 *--------------------------------------------------------------------------------------*
 * 20230925   lum    Initial version.                                                   *
 *--------------------------------------------------------------------------------------*/
#ifndef __BSP_KEY_H__
#define	__BSP_KEY_H__


#include "stm32f10x.h"



#define KEY1_GPIO_PORT    	GPIOB			              
#define KEY1_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define KEY1_GPIO_PIN		GPIO_Pin_12			        


#define KEY2_GPIO_PORT    	GPIOB			              
#define KEY2_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define KEY2_GPIO_PIN		GPIO_Pin_13		        


#define KEY1_GET_STATUS()     GPIO_ReadInputDataBit(KEY1_GPIO_PORT,KEY1_GPIO_PIN)
#define KEY2_GET_STATUS()     GPIO_ReadInputDataBit(KEY2_GPIO_PORT,KEY2_GPIO_PIN)






void bsp_key_init(void);
uint8_t bsp_key_scan(void);

#endif

