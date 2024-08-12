#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>

/*********************************************************************************************************
*                                              �궨��
*********************************************************************************************************/
// LED1��LED2�ⲿ3V3�������͵�ƽ�������ߵ�ƽϨ��
#define LED1_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_14)
#define LED1_OFF GPIO_SetBits(GPIOC, GPIO_Pin_14)
#define LED2_ON  GPIO_ResetBits(GPIOC, GPIO_Pin_15)
#define LED2_OFF GPIO_SetBits(GPIOC, GPIO_Pin_15)

#define BAUD_RATE  115200
#define SYSTICK_RELOAD_VALUE SystemCoreClock/1000  // SysTick��װ��ֵ��ϵͳʱ��ÿ���봥��һ���ж�
#define DELAY_MS  250  // ���������SysTick��ʱʱ�����

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/
// ϵͳ״̬ö��
typedef enum {
  STATE_IDLE,  // Ĭ�ϴ���״̬
  STATE_SPO2,
  STATE_ECG,
  STATE_TEMP
}EnumSystemState;

// �������ͽṹ�嶨��
typedef struct {
  bool isDelayDone;
}StructFlagTypeDef;

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
EnumSystemState currentState;
StructFlagTypeDef flagType;

static __IO u32 s_iTimDelayCnt = DELAY_MS;

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
//EXTI��ʼ������
void InitEXTI(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource11);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2);

  EXTI_InitStructure.EXTI_Line = EXTI_Line11 | EXTI_Line12 | EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // PC11��PC12
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; // PD2
  NVIC_Init(&NVIC_InitStructure);
}

void EXTI15_10_IRQHandler(void) {
  if(EXTI_GetFlagStatus(EXTI_Line11) != RESET) {
    // ����PC11�ж�
    currentState = STATE_SPO2;
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
  if(EXTI_GetFlagStatus(EXTI_Line12) != RESET) {
    // ����PC12�ж�
    currentState = STATE_ECG;
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
}

void EXTI2_IRQHandler(void) {
  if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
    // ����PD2�ж�
    currentState = STATE_TEMP;
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}

//������ʼ������
void InitKey(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;  // Key1 Key2
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;  // Key3
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

// SysTick ��ʼ������
void InitSysTick(void) {
  SystemCoreClockUpdate();  //����ϵͳʱ��
  if (SysTick_Config(SYSTICK_RELOAD_VALUE) != 0) {
    // �������ʧ�ܣ�������ѭ���������
    while (1){};
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

// LED ��ʼ������
void InitLED(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15; // PC14��PC15
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_SetBits(GPIOC, GPIO_Pin_14 | GPIO_Pin_15);  // Ĭ�ϸߵ�ƽ
}

void FlickerLED(const EnumSystemState state) {
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

// USART1 ��ʼ������
void USART1_Init(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

  // ���� PA9 Ϊ USART1 TX
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // ���� PA10 Ϊ USART1 RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // ���� USART1 ����
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = BAUD_RATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);

  // ʹ�� USART1
  USART_Cmd(USART1, ENABLE);
}

// �ض��� fputc ����
int fputc(int ch, FILE *f) {
  // �ȴ�ֱ�� USART1 �����ݼĴ�������
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {

  };
  // ���ַ����͵� USART1
  USART_SendData(USART1, (u8) ch);
  // �ȴ��������
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {

  };
  return ch;
}

int main(void) {
  InitSysTick();
  USART1_Init();
  InitEXTI();

  InitLED();
  InitKey();

  while(1) {
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
