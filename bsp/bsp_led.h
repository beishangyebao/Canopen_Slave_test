/** \internal
 *--------------------------------------------------------------------------------------*
 *                                                                                      *
 *  COPYRIGHT(C) 2023 Shengkong Intelligent Technology Co.   Ltd                        *
 *                                                                                      *
 *--------------------------------------------------------------------------------------*
 * Description: led head  file                                                          *
 *--------------------------------------------------------------------------------------*
 * History:                                                                             *
 *  <Date>   <Author>  <Description>                                                    *
 *--------------------------------------------------------------------------------------*
 * 20230925   lum    Initial version.                                                   *
 *--------------------------------------------------------------------------------------*/
#ifndef __BSP_LED_H__
#define	__BSP_LED_H__


#include "stm32f10x.h"



#define LED1_GPIO_PORT    	GPIOB			              
#define LED1_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define LED1_GPIO_PIN		    GPIO_Pin_14			        


#define LED2_GPIO_PORT    	GPIOB			              
#define LED2_GPIO_CLK 	    RCC_APB2Periph_GPIOB		
#define LED2_GPIO_PIN		    GPIO_Pin_15		        




/* 定义控制IO的宏 */
#define LED1_TOGGLE		 {LED1_GPIO_PORT->ODR^=LED1_GPIO_PIN;}
#define LED1_OFF		   {LED1_GPIO_PORT->BSRR=LED1_GPIO_PIN;}
#define LED1_ON			   {LED1_GPIO_PORT->BRR=LED1_GPIO_PIN;}

#define LED2_TOGGLE		 {LED2_GPIO_PORT->ODR^=LED2_GPIO_PIN;}
#define LED2_OFF		   {LED2_GPIO_PORT->BSRR=LED2_GPIO_PIN;}
#define LED2_ON			   {LED2_GPIO_PORT->BRR=LED2_GPIO_PIN;}





void bsp_led_init(void);

#endif /* __LED_H */
