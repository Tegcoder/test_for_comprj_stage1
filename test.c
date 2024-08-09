#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>

/*********************************************************************************************************
*                                              宏定义
*********************************************************************************************************/
// LED1和LED2外部3V3上拉，低电平点亮，高电平熄灭
#define LED1_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_14)
#define LED1_OFF GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define LED2_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_15)
#define LED2_OFF GPIO_SetBits(GPIOC, GPIO_Pin_15)

#define BAUD_RATE  115200
#define SYSTICK_RELOAD_VALUE SystemCoreClock/1000  // SysTick重装载值，系统时钟每毫秒触发一次中断
#define DELAY_MS  250  // 定义宏用于SysTick延时时间计数

// 定义宏用于读取特定引脚的电平状态并进行检查
#define CHECK_SPO2_KEY()  (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) == 0)
#define CHECK_ECG_KEY()   (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == 0)
#define CHECK_TEMP_KEY()  (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 0)
/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
// 系统状态枚举
typedef enum {
  STATE_IDLE,  // 默认待机状态
  STATE_SPO2,
  STATE_ECG,
  STATE_TEMP
}EnumSystemState;

// 布尔类型结构体定义
typedef struct {
  bool isDelayDone;
}StructFlagTypeDef;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
EnumSystemState currentState;
StructFlagTypeDef flagType;

static __IO u32 s_iTimDelayCnt = DELAY_MS;

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//按键初始化函数
void InitKey(void) {
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
  if(CHECK_SPO2_KEY()) {
    *state = STATE_SPO2;
  }
  else if(CHECK_ECG_KEY()) {
    *state = STATE_ECG;
  }
  else if(CHECK_TEMP_KEY()) {
    *state = STATE_TEMP;
  }
}

// SysTick 初始化函数
void InitSysTick(void) {
  SystemCoreClockUpdate();  //更新系统时钟
  if (SysTick_Config(SYSTICK_RELOAD_VALUE) != 0) {
    // 如果配置失败，进入死循环处理错误
    while (1);
  }
}

void SysTick_Handler(void) {
  if (s_iTimDelayCnt > 0) {
    s_iTimDelayCnt--;
  }
  else {
    flagType.isDelayDone = !flagType.isDelayDone;
    s_iTimDelayCnt = DELAY_MS;
  }
}

// LED 初始化函数
void InitLED(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_SetBits(GPIOC, GPIO_Pin_14);
  GPIO_SetBits(GPIOC, GPIO_Pin_15);
}

void FlickerLED(u8 state) {
  switch(state)
  {
    case STATE_SPO2:
      if(flagType.isDelayDone) {
      LED1_OFF;
      LED2_ON;
      }
      else {
      LED1_OFF;
      LED2_OFF;
      }
    break;
    case STATE_ECG:
      if(flagType.isDelayDone) {
      LED1_ON;
      LED2_ON;
      }
      else {
      LED1_OFF;
      LED2_OFF;
      }
    break;
    case STATE_TEMP:
      if(flagType.isDelayDone) {
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
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置 PA10 为 USART1 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // 配置 USART1 参数
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = BAUD_RATE;
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
  USART_SendData(USART1, (u8) ch);
  // 等待发送完成
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
  return ch;
}

int main(void) {
  InitSysTick();
  USART1_Init();

  InitLED();
  InitKey();

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
