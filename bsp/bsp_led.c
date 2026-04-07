#include "bsp_led.h"   


void bsp_led_init()
{		

		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd( LED1_GPIO_CLK, ENABLE);
	    RCC_APB2PeriphClockCmd( LED2_GPIO_CLK, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

		GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;
		GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);	
		
		GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;
		GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
}

/*********************************************END OF FILE**********************/
