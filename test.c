#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
#define LED1_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_14)  // LED1亮
#define LED1_OFF GPIO_SetBits(GPIOC, GPIO_Pin_14)    // LED1灭
#define LED2_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_15)  // LED2亮
#define LED2_OFF GPIO_SetBits(GPIOC, GPIO_Pin_15)    // LED2灭

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
// 定义状态枚举
typedef enum {
  STATE_SPO2,    // 血氧
  STATE_ECG,     // 心电
  STATE_TEMP     // 体温
} SystemState;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
// 当前系统状态
SystemState currentState;

static __IO u32 s_iTimDelayCnt = 250;  // 250ms的延时计数
static bool isDelayDone;

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//按键初始化函数
void Key_Init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  // Key1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  // Key2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  // Key3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void ScanKey(u8* state) {
  if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) == 0) {
    *state = STATE_SPO2;
  }
  else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == 0) {
    *state = STATE_ECG;
  }
  else if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 0) {
    *state = STATE_TEMP;
  }
}

// SysTick 初始化函数
void SysTick_Init(void) {
  SystemCoreClockUpdate();  //更新系统时钟
  //配置SysTick的重装值为10000，即10ms中断一次
  if (SysTick_Config(SystemCoreClock / 1000) != 0) {
    //错误处理
    while (1);
  }
}

void SysTick_Handler(void) {
  //每次SysTick中断时，执行此函数
  if (s_iTimDelayCnt > 0) {
    s_iTimDelayCnt--;
  }
  else {
    isDelayDone = !isDelayDone;
    s_iTimDelayCnt = 250;
  }
}

// LED 初始化函数
void LED_Init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  GPIO_ResetBits(GPIOC, GPIO_Pin_14);
  GPIO_ResetBits(GPIOC, GPIO_Pin_15);
}

void FlickerLED(u8 state) {
  switch(state)
  {
    case STATE_SPO2:
      if(isDelayDone) {
      LED1_OFF;
      LED2_ON;
      }
      else {
      LED1_OFF;
      LED2_OFF;
      }
    break;
    case STATE_ECG:
      if(isDelayDone) {
      LED1_ON;
      LED2_ON;
      }
      else {
      LED1_OFF;
      LED2_OFF;
      }
    break;
    case STATE_TEMP:
      if(isDelayDone) {
      LED1_ON;
      LED2_OFF;
      }
      else {
      LED1_OFF;
      LED2_OFF;
      }
    break;
    default:

    break;
  }
}

// USART1 初始化函数
void USART1_Init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

  // 配置 PA9 为 USART1 TX
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置 PA10 为 USART1 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  // 浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置 USART1 参数
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // 使能 USART1
  USART_Cmd(USART1, ENABLE);
}

// 重定向 fputc 函数
int fputc(int ch, FILE *f) {
  // 等待直到 USART1 的数据寄存器空闲
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  // 将字符发送到 USART1
  USART_SendData(USART1, (uint8_t) ch);
  // 等待发送完成
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  return ch;
}

int main(void) {
  SysTick_Init();
  USART1_Init();
  LED_Init();
  Key_Init();

  while(1) {
    ScanKey(&currentState);

    switch(currentState) {
      case STATE_SPO2:
        FlickerLED(currentState);
        printf("Switched to SPO2 module.\r\n");
        break;
      case STATE_ECG:
        FlickerLED(currentState);
        printf("Switched to ECG module.\r\n");
        break;
      case STATE_TEMP:
        FlickerLED(currentState);
        printf("Switched to TEMP module.\r\n");
        break;
      default:
        printf("Unknown state!\r\n");
        break;
    }
  }
}
