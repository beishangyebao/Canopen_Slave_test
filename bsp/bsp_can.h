/** \internal
 *--------------------------------------------------------------------------------------*
 *                                                                                      *
 *  COPYRIGHT(C) 2023 Shengkong Intelligent Technology Co.   Ltd                        *
 *                                                                                      *
 *--------------------------------------------------------------------------------------*
 * Description: can head  file                                                          *
 *--------------------------------------------------------------------------------------*
 * History:                                                                             *
 *  <Date>   <Author>  <Description>                                                    *
 *--------------------------------------------------------------------------------------*
 * 20230925   lum    Initial version.                                                   *
 *--------------------------------------------------------------------------------------*/
#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "stm32f10x.h"
#include "canfestival.h"



void bsp_can_init(void);
uint8_t canSend(CAN_PORT notused, Message *message);

#endif
