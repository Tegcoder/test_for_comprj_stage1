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
#define DELAY_250_MS  250  // ���������SysTick��ʱʱ�����

// ����ARR��PSCֵ
#define SPO2_PSC   71
#define SPO2_ARR   499
#define ECG_PSC    71
#define ECG_ARR    99
#define TEMP_PSC   71
#define TEMP_ARR   7999

/*********************************************************************************************************
*                                              ö�ٽṹ�嶨��
*********************************************************************************************************/
// ϵͳ״̬ö��
typedef enum {
  STATE_IDLE = 0, // Ĭ�ϴ���״̬
  STATE_SPO2,
  STATE_ECG,
  STATE_TEMP,
  STATE_MAX // ����ֵ�����ڱ�ʾ״̬�����������ֵ
} EnumSystemState;

// �������ͽṹ�嶨��
typedef struct {
  bool isDelayDone;
  bool s_i500usFlag;    //��500us��־λ��ֵ����ΪFALSE
  bool s_i2msFlag;      //��2ms��־λ��ֵ����ΪFALSE
  bool s_i1secFlag;     //��1s��־λ��ֵ����ΪFALSE
} StructFlagTypeDef;

/*********************************************************************************************************
*                                              �ڲ�����
*********************************************************************************************************/
EnumSystemState currentState = STATE_MAX;
EnumSystemState lastState = STATE_IDLE; // ʹ�ø���ֵ��ʾδ��ʼ��״̬
StructFlagTypeDef flagType;

static __IO u32 s_iSysTickDelayCnt = DELAY_250_MS;
static u16 s_arrADC1Data;   //���ADCת���������

/*********************************************************************************************************
*                                              �ڲ�����ʵ��
*********************************************************************************************************/
//TIM3��ʱ����ʼ������
void InitTimer3(void) {
  TIM_DeInit(TIM3);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

switch(currentState) {
  case STATE_SPO2:
    TIM_TimeBaseInitStructure.TIM_Prescaler = SPO2_PSC; // 72MHz / (71 + 1) = 1MHz����ʱ��ʱ��Ƶ��Ϊ 1MHz
    TIM_TimeBaseInitStructure.TIM_Period =    SPO2_ARR; // 1MHz / (499 + 1) = 2kHz��500��s����ÿ 500��s ����һ��
    printf("Configured STATE_SPO2: Prescaler %d, Auto-reload %d\n", SPO2_PSC, SPO2_ARR);
    break;

  case STATE_ECG:
    TIM_TimeBaseInitStructure.TIM_Prescaler = ECG_PSC; // 72MHz / (71 + 1) = 1MHz����ʱ��ʱ��Ƶ��Ϊ 1MHz
    TIM_TimeBaseInitStructure.TIM_Period =    ECG_ARR; // 1MHz / (9999 + 1) = 100Hz��10ms����ÿ 10ms ����һ��
    printf("Configured STATE_ECG: Prescaler %d, Auto-reload %d\n", ECG_PSC, ECG_ARR);
    break;

  case STATE_TEMP:
    TIM_TimeBaseInitStructure.TIM_Prescaler = TEMP_PSC; // 72MHz / (71 + 1) = 1MHz����ʱ��ʱ��Ƶ��Ϊ 1MHz
    TIM_TimeBaseInitStructure.TIM_Period =    TEMP_ARR; // 1MHz / (9999 + 1) = 100Hz��10ms����ÿ 10ms ����һ��
    printf("Configured STATE_TEMP: Prescaler %d, Auto-reload %d\n", TEMP_PSC, TEMP_ARR);
    break;

  default:
    printf("Unspecified state!TIM3 configuration failure.\n");
    break;
}

  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
}

//TIM2��ʱ����ʼ������
void InitTimer2(void) {
  TIM_DeInit(TIM2);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //����TIM2�ṩ������100us��ʱ
  TIM_TimeBaseInitStructure.TIM_Prescaler = 71;
  TIM_TimeBaseInitStructure.TIM_Period =    99;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // ʹ�ܸ����ж�Դ

  TIM_Cmd(TIM2, ENABLE);

  printf("TIM2 config success.\n");
}

void TIM2_IRQHandler(void) {
    static u8 s_iCnt500us = 0; // 500us������
    static u8 s_iCnt2ms = 0;   // 2ms������
    static u16 s_iCnt1sec = 0; // 1s������

    // �������жϱ�־λ
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // ����жϱ�־λ

        s_iCnt500us++;
        s_iCnt2ms++;
        s_iCnt1sec++;

        // 500us��־λ
        if (s_iCnt500us >= 5) {  // 100us һ�Σ�500us ��Ҫ 5 ��
            flagType.s_i500usFlag = true;
            s_iCnt500us = 0; // ���ü�����
        }

        // 2ms��־λ
        if (s_iCnt2ms >= 20) {  // 100us һ�Σ�2ms ��Ҫ 20 ��
            flagType.s_i2msFlag = true;
            s_iCnt2ms = 0; // ���ü�����
        }

        // 1s��־λ
        if (s_iCnt1sec >= 10000) {  // 100us һ�Σ�1s ��Ҫ 10000 ��
            flagType.s_i1secFlag = true;
            s_iCnt1sec = 0; // ���ü�����
        }
    }
}

//ͨ��ADC����
void InitADC(void) {
  ADC_DeInit(ADC1);

  // ʹ��ADC���ʱ��
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); // ����ADCʱ�ӷ�Ƶ��ADCCLK=PCLK2/6=12MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // ʹ��ADC1��ʱ��

  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef  ADC_InitStructure;

  // ����GPIO��ADC
  switch (currentState) {
    case STATE_SPO2:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ʹ��GPIOA��ʱ��

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; // ʹ���������
      ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5); //���ò���ʱ��Ϊ239.5������

      printf("Configured STATE_SPO2: ADC_Channel_5, GPIOA_Pin_5, using software trigger\n");
      break;

    case STATE_ECG:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // ʹ��GPIOC��ʱ��

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOC, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO; // ʹ��TIM3����
      ADC_ExternalTrigConvCmd(ADC1, ENABLE); // ʹ���ⲿ�¼�����ADCת��
      ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5); //���ò���ʱ��Ϊ239.5������

      printf("Configured STATE_ECG: ADC_Channel_14, GPIOC_Pin_4, using TIM3 trigger\n");
      break;

    case STATE_TEMP:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // ʹ��GPIOA��ʱ��

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO; // ʹ��TIM3����
      ADC_ExternalTrigConvCmd(ADC1, ENABLE); // ʹ���ⲿ�¼�����ADCת��
      ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5); //���ò���ʱ��Ϊ239.5������

      printf("Configured STATE_TEMP: ADC_Channel_6, GPIOA_Pin_6, using TIM3 trigger\n");
      break;

    default:
      printf("Unspecified state!ADC configuration failure.\n");
      break;
  }

  // ADCͨ�����ò���
  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel       = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_DMACmd(ADC1, ENABLE); // ʹ��DMA
  ADC_Cmd(ADC1, ENABLE); // ʹ��ADC

  ADC_ResetCalibration(ADC1); // ����ADC��λУ׼
  while (ADC_GetResetCalibrationStatus(ADC1)); // �ȴ���λУ׼���
  ADC_StartCalibration(ADC1); // ����ADCУ׼
  while (ADC_GetCalibrationStatus(ADC1)); // �ȴ�У׼���
}

//DMA1 ��ʼ������
void InitDMA(void) {
  DMA_DeInit(DMA1_Channel1);

  DMA_InitTypeDef DMA_InitStructure;  //DMA_InitStructure���ڴ��DMA�Ĳ���

  //ʹ��RCC���ʱ��
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //ʹ��DMA1��ʱ��

  //����DMA1_Channel1
  DMA_DeInit(DMA1_Channel1);  //��DMA1_CH1�Ĵ�������ΪĬ��ֵ
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);           //���������ַ
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&s_arrADC1Data;        //���ô洢����ַ
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;           //����Ϊ���赽�洢��ģʽ
  DMA_InitStructure.DMA_BufferSize         = 1;                               //����Ҫ�������������Ŀ
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //��������Ϊ�ǵ���ģʽ
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //���ô洢��Ϊ����ģʽ
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //�����������ݳ���Ϊ����
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;     //���ô洢�����ݳ���Ϊ����
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //����Ϊѭ��ģʽ
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;             //����Ϊ�е����ȼ�
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;                 //��ֹ�洢�����洢������
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //���ݲ�����ʼ��DMA1_Channel1

  DMA_Cmd(DMA1_Channel1, ENABLE); //ʹ��DMA1_Channel1

  printf("DMA config success.\n");
}

//EXTI��ʼ������
void InitEXTI(void) {
  EXTI_DeInit();

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

  printf("EXTI config success.\n");
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

  printf("Key config success.\n");
}

// SysTick ��ʼ������
void InitSysTick(void) {
  SystemCoreClockUpdate();  //����ϵͳʱ��
  if (SysTick_Config(SYSTICK_RELOAD_VALUE) != 0) {
    // �������ʧ�ܣ�������ѭ���������
    while (1){};
  }
  printf("SysTick config success.\n");
}

void SysTick_Handler(void) {
  if (s_iSysTickDelayCnt > 0) {
    s_iSysTickDelayCnt--;
  }
  else {
    flagType.isDelayDone = !flagType.isDelayDone;
    s_iSysTickDelayCnt = DELAY_250_MS;
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

void FlickerLED(void) {
  switch(currentState)
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
  USART_DeInit(USART1);

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

  printf("USART1 config success.\n");
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

// ״̬�л����ú���
void ConfigureForSpO2(void) {
}

void ConfigureForECG(void) {
}

void ConfigureForTemperature(void) {
}

// �����ʼ������
void PeripheralInit(void) {
  USART1_Init();   // ��ʼ��USART1�����ڴ���ͨ��

  InitSysTick();   // ��ʼ��ϵͳ�δ�ʱ������������ϵͳ����

  InitEXTI();      // ��ʼ���ⲿ�жϣ����ڰ������ⲿ�¼��Ĳ���

  InitTimer3();    // ��ʼ��TIM3������ADCģ��Ĵ�������

  InitTimer2();    // ��ʼ��TIM2������������ʱ����

  InitADC();       // ��ʼ��ADC������ģ���źŵĲ�����ת��

  InitDMA();       // ��ʼ��DMA������������ݴ���Ч��

  InitLED();       // ��ʼ��LED������ָʾ�ƵĿ���

  InitKey();       // ��ʼ�������������û�����ļ��
}

const char* GetStateName(EnumSystemState state) {
  switch (state) {
    case STATE_SPO2: return "STATE_SPO2";
    case STATE_ECG: return "STATE_ECG";
    case STATE_TEMP: return "STATE_TEMP";
    case STATE_IDLE: return "STATE_IDLE";
    default: return "UNKNOWN_STATE";
  }
}

// ����״̬�л�
void HandleStateChange(void) {
  if (currentState != lastState) {
    switch (currentState) {
      case STATE_SPO2:
        PeripheralInit();
        ConfigureForSpO2();
        break;
      case STATE_ECG:
        PeripheralInit();
        ConfigureForECG();
        break;
      case STATE_TEMP:
        PeripheralInit();
        ConfigureForTemperature();
        break;
      default:
        PeripheralInit();
        break;
    }
    printf("State changed from %s to %s\n", GetStateName(lastState), GetStateName(currentState));
    lastState = currentState;
   }
}

// ״̬��������ʵ��
void HandleSpO2(void) {
  FlickerLED();
}

void HandleECG(void) {
  FlickerLED();
}

void HandleTemperature(void) {
  FlickerLED();
}

// ִ�е�ǰ״̬������
void ExecuteCurrentState(void) {
  switch (currentState) {
    case STATE_SPO2:
      HandleSpO2();
      break;
    case STATE_ECG:
      HandleECG();
      break;
    case STATE_TEMP:
      HandleTemperature();
      break;
    default:
      break;
  }
}

// ����
void Test(void) {
    // ��ӡDMA�����ADC����
    printf("ADC: %d\r\n", s_arrADC1Data);

    // ���� 1s ��־λ
    if (flagType.s_i1secFlag) {
        printf("1secflag is ready!\n");
        flagType.s_i1secFlag = false;
    }

    // ���� 500us ��־λ
    if (flagType.s_i500usFlag) {
        printf("500usflag is ready!\n");
        flagType.s_i500usFlag = false;
    }

    // ���� 2ms ��־λ
    if (flagType.s_i2msFlag) {
        printf("2msflag is ready!\n");
        flagType.s_i2msFlag = false;
    }
}

// ִ��������̨����
void PerformBackgroundTasks(void) {
  // ����ADC��DMA�Ĳ���
  Test();
  // ִ������������������̨����
}

// ״̬��������
void StateMachine(void) {
  while (1) {
    HandleStateChange();    // ����״̬�л�
    ExecuteCurrentState();  // ִ�е�ǰ״̬������
    PerformBackgroundTasks();  // ִ��������̨����
  }
}

// ������
int main(void) {
  // ϵͳ��ʼ��
  SystemInit();

  StateMachine();

  return 0;
}
