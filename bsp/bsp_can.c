#include "bsp_can.h"

#include "TestSlave.h"
#include "can_app.h"
#include "canfestival.h"
#include "generator.h"
#include "stm32f10x.h"

#include <string.h>

/* CAN1 底层初始化与中断，同时处理 CanFestival 报文和 G4 反馈。 */

#define ID_G4_FEEDBACK 0x011u

extern CO_Data TestSlave_Data;

volatile uint32_t g_last_g4_heartbeat_time;

extern volatile uint8_t g_g4_fault_status;

/* 使能 GPIO/AFIO 时钟后，配置 PA11/PA12 为 CAN 收发引脚。 */
static void gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* 将 CAN RX FIFO0 中断接入系统。 */
static void nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* 接收工程使用标准帧，过滤器保持较宽松配置。 */
static void can_filter_config(void)
{
    CAN_FilterInitTypeDef CAN_FilterInitStructure;

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}

/* 按当前总线时序初始化 CAN1，并打开接收中断。 */
void bsp_can_init(void)
{
    CAN_InitTypeDef CAN_InitStructure;

    gpio_config();
    nvic_config();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = ENABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 = CAN_BS1_6tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
    CAN_InitStructure.CAN_Prescaler = 3;

    CAN_Init(CAN1, &CAN_InitStructure);
    can_filter_config();
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}

/* 先解析 G4 专用反馈帧，其余报文再交给 CanFestival。 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CanRxMsg message;
    Message Rx_Message;

    CAN_Receive(CAN1, CAN_FIFO0, &message);

    if (message.StdId == ID_G4_FEEDBACK) {
        uint8_t status_byte;
        uint8_t mode;
        int32_t act_main;

        /* 每收到一帧有效反馈，都先刷新应用层的通信监督状态。 */
        CAN_App_OnG4FeedbackReceived();

        /* 厂商状态字节的 bit0 作为 G4 设备故障总标志。 */
        status_byte = message.Data[0];
        g_g4_fault_status = (status_byte & 0x01u) ? 1u : 0u;

        /*
         * 当前从站只使用 mode 和主反馈值 act_main。
         * 力矩控制不在这一层闭环，因此 Byte2~3 的实际力矩字段不参与本地控制链。
         */
        mode = message.Data[1];
        act_main = (int32_t)(message.Data[4] |
                             (message.Data[5] << 8) |
                             (message.Data[6] << 16) |
                             (message.Data[7] << 24));

        Generator_OnG4Feedback(mode, act_main);
        return;
    }

    Rx_Message.cob_id = message.StdId;
    Rx_Message.rtr = (message.RTR == CAN_RTR_DATA) ? 0 : 1;
    Rx_Message.len = message.DLC;
    memcpy(Rx_Message.data, message.Data, message.DLC);

    canDispatch(&TestSlave_Data, &Rx_Message);
}

/* 提供给 CanFestival 和 G4 驱动辅助函数共用的阻塞式发送封装。 */
uint8_t canSend(CAN_PORT notused, Message *message)
{
    uint32_t i = 0xFFFFFFu;
    CanTxMsg TxMessage;
    uint8_t TransmitMailbox;

    (void)notused;

    TxMessage.DLC = message->len;
    memcpy(TxMessage.Data, message->data, message->len);
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.StdId = message->cob_id;
    /* CanFestival 的 RTR 表示值与 STM32 库不同，这里做一次转换。 */
    TxMessage.RTR = (message->rtr == CAN_RTR_DATA) ? 0 : 2;

    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);
    /* 轮询等待发送完成，因为上层期望拿到同步发送结果。 */
    while ((CAN_TransmitStatus(CAN1, TransmitMailbox) != CANTXOK) && --i) {
    }

    return (i != 0u) ? 0u : 1u;
}
