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

// ��������ڶ�ȡ�ض����ŵĵ�ƽ״̬�����м��
#define CHECK_SPO2_KEY()  (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) == 0)
#define CHECK_ECG_KEY()   (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == 0)
#define CHECK_TEMP_KEY()  (GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_2) == 0)
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
//������ʼ������
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

// SysTick ��ʼ������
void InitSysTick(void) {
  SystemCoreClockUpdate();  //����ϵͳʱ��
  if (SysTick_Config(SYSTICK_RELOAD_VALUE) != 0) {
    // �������ʧ�ܣ�������ѭ���������
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

// LED ��ʼ������
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
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  // ���ַ����͵� USART1
  USART_SendData(USART1, (u8) ch);
  // �ȴ��������
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
