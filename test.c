/*********************************************************************************************************
* 当前版本：4.0.0
* 作    者：Tegco
* 完成日期：2024年08月20日
* 整机实现说明：三个按键 Key1、Key2 和 Key3，它们分别对应血氧、心电和体温的采集功能，目前血氧和心电只提供基础的波
* 形显示，体温只提供S1采样并计算出电阻值，不提供C1C2校准。
**********************************************************************************************************/

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
#define DELAY_250_MS  250  // 定义宏用于SysTick延时时间计数
#define DAC_DEFAULT_VALUE 1500  // DAC输出的默认数字值（12位 4095），对应的模拟电压基于参考电压(3V3)进行转换

// TIM2配置，用于一般任务的定时功能
#define TIMER2_TASK_PSC    71   // TIM2预分频值，将72MHz的系统时钟分频为1MHz时钟频率（72MHz/72）
#define TIMER2_TASK_ARR    99   // TIM2重装载值，产生10kHz的定时中断信号（1MHz/100），对应的中断周期为100μs

// TIM4配置，用于DAC1触发
#define TIMER4_DAC1_PSC    71   // TIM4预分频值，将72MHz的系统时钟分频为100kHz时钟频率（72MHz/72）
#define TIMER4_DAC1_ARR    7999 // TIM4重装载值，产生12.5Hz的触发信号（1MHz/8000），对应的触发周期为8ms

// TIM3配置，用于ADC1触发，具体用于SPO2、ECG、TEMP等模块
#define TIMER3_SPO2_PSC    71   // TIM3预分频值，将72MHz的系统时钟分频为1MHz时钟频率（72MHz/72）
#define TIMER3_SPO2_ARR    49   // TIM3重装载值，产生20kHz的ADC触发信号（1MHz/50）,对应的触发周期为50μs

#define TIMER3_ECG_PSC     71   // TIM3预分频值，将72MHz的系统时钟分频为1MHz时钟频率（72MHz/72）
#define TIMER3_ECG_ARR     99   // TIM3重装载值，产生10kHz的ADC触发信号（1MHz/100）,对应的触发周期为100μs

#define TIMER3_TEMP_PSC    71   // TIM3预分频值，将72MHz的系统时钟分频为1MHz时钟频率（72MHz/72）
#define TIMER3_TEMP_ARR    7999 // TIM3重装载值，产生125Hz的ADC触发信号（1MHz/8000）,对应的触发周期为8ms

/*********************************************************************************************************
*                                              枚举结构体定义
*********************************************************************************************************/
// 系统状态枚举
typedef enum {
  STATE_IDLE = 0, // 默认待机状态
  STATE_SPO2,
  STATE_ECG,
  STATE_TEMP,
  STATE_MAX // 辅助值，用于表示状态的数量或结束值
} EnumSystemState;

// 布尔类型结构体定义
typedef struct {
  bool isDelayDone;
  bool s_i500usFlag;
  bool s_i2msFlag;
  bool s_i1secFlag;
} StructFlagTypeDef;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
EnumSystemState currentState = STATE_MAX;
EnumSystemState lastState = STATE_IDLE; // 使用辅助值表示未初始化状态
StructFlagTypeDef flagType;

static __IO u32 s_iSysTickDelayCnt = DELAY_250_MS;
u16 s_iADC1Data;   //存放ADC转换结果数据

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//TIM4定时器初始化函数
void InitTimer4ForDAC1(void) {
  TIM_DeInit(TIM4);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER4_DAC1_PSC;
  TIM_TimeBaseInitStructure.TIM_Period =    TIMER4_DAC1_ARR;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);
  TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);

  TIM_Cmd(TIM4, ENABLE);
}

//TIM3定时器初始化函数
void InitTimer3ForADC1(void) {

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //使能TIM3的时钟
  TIM_DeInit(TIM3);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

switch(currentState) {
  case STATE_SPO2:
    TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER3_SPO2_PSC;
    TIM_TimeBaseInitStructure.TIM_Period =    TIMER3_SPO2_ARR;
    printf("Configured STATE_SPO2: Prescaler %d, Auto-reload %d\n", TIMER3_SPO2_PSC, TIMER3_SPO2_ARR);
    break;

  case STATE_ECG:
    TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER3_ECG_PSC;
    TIM_TimeBaseInitStructure.TIM_Period    = TIMER3_ECG_ARR;
    printf("Configured STATE_ECG: Prescaler %d, Auto-reload %d\n", TIMER3_ECG_PSC, TIMER3_ECG_ARR);
    break;

  case STATE_TEMP:
    TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER3_TEMP_PSC;
    TIM_TimeBaseInitStructure.TIM_Period =    TIMER3_TEMP_ARR;
    printf("Configured STATE_TEMP: Prescaler %d, Auto-reload %d\n", TIMER3_TEMP_PSC, TIMER3_TEMP_ARR);
    break;

  default:
    printf("Unspecified state!TIM3 configuration failure.\n");
    break;
  }

  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up; //设置向上计数模式
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);               //根据参数初始化定时器
  TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);          //选择更新事件为触发输入
  TIM_Cmd(TIM3, ENABLE);  //使能定时器
}

//TIM2定时器初始化函数
void InitTimer2ForTask(void) {
  TIM_DeInit(TIM2);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //配置TIM2提供基础的100us延时
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIMER2_TASK_PSC;
  TIM_TimeBaseInitStructure.TIM_Period =    TIMER2_TASK_ARR;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // 使能更新中断源

  TIM_Cmd(TIM2, ENABLE);

  printf("TIM2 config success.\n");
}

void TIM2_IRQHandler(void) {
    static u8 s_iCnt500us = 0; // 500us计数器
    static u8 s_iCnt2ms = 0;   // 2ms计数器
    static u16 s_iCnt1sec = 0; // 1s计数器

    // 检查更新中断标志位
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)!= RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update); // 清除中断标志位

        s_iCnt500us++;
        s_iCnt2ms++;
        s_iCnt1sec++;

        // 500us标志位
        if (s_iCnt500us >= 5) {  // 100us 一次，500us 需要 5 次
            flagType.s_i500usFlag = true;
            s_iCnt500us = 0; // 重置计数器
        }

        // 2ms标志位
        if (s_iCnt2ms >= 20) {  // 100us 一次，2ms 需要 20 次
            flagType.s_i2msFlag = true;
            s_iCnt2ms = 0; // 重置计数器
        }

        // 1s标志位
        if (s_iCnt1sec >= 10000) {  // 100us 一次，1s 需要 10000 次
            flagType.s_i1secFlag = true;
            s_iCnt1sec = 0; // 重置计数器
        }
    }
}

//DAC配置
void InitDAC(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  DAC_DeInit();

  GPIO_InitTypeDef GPIO_InitStructure;
  DAC_InitTypeDef DAC_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // 防止寄生干扰

  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T4_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None; // 关闭波形生成

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_SetChannel1Data(DAC_Align_12b_R, DAC_DEFAULT_VALUE); // DAC默认输出值
  DAC_Cmd(DAC_Channel_1, ENABLE);
}

//ADC配置
void InitADC(void) {
  // 使能ADC相关时钟
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 设置ADC时钟分频，ADCCLK=PCLK2/6=12MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1  , ENABLE);
  ADC_DeInit(ADC1);

  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef  ADC_InitStructure;

  // 配置GPIO和ADC
  switch (currentState) {
    case STATE_SPO2:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; // 使用软件触发
      ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5); // 设置采样时间为239.5个周期

      printf("Configured STATE_SPO2: ADC_Channel_5, GPIOA_Pin_5, using software trigger\n");
      break;

    case STATE_ECG:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC , ENABLE);

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOC, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO;  // 使用TIM3触发
      ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期

      printf("Configured STATE_ECG: ADC_Channel_14, GPIOC_Pin_4, using TIM3 trigger\n");
      break;

    case STATE_TEMP:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO; // 使用TIM3触发
      ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5); // 设置采样时间为239.5个周期

      printf("Configured STATE_TEMP: ADC_Channel_6, GPIOA_Pin_6, using TIM3 trigger\n");
      break;

    default:
      printf("Unspecified state!ADC configuration failure.\n");
      break;
  }

  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;  //设置为独立模式
  ADC_InitStructure.ADC_ScanConvMode       = ENABLE;                //使能扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;               //禁止连续转换模式
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;   //设置为右对齐
  ADC_InitStructure.ADC_NbrOfChannel       = 1; //设置ADC的通道数目

  ADC_Init(ADC1, &ADC_InitStructure);
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);      //使用外部事件启动ADC转换
  ADC_Cmd(ADC1, ENABLE);                      //使能ADC1
  ADC_DMACmd(ADC1, ENABLE);                   //使能ADC1的DMA
  ADC_ResetCalibration(ADC1);                 //启动ADC复位校准，即将RSTCAL赋值为1
  while(ADC_GetResetCalibrationStatus(ADC1)); //读取并判断RSTCAL，RSTCAL为0跳出while语句
  ADC_StartCalibration(ADC1);                 //启动ADC校准，即将CAL赋值为1
  while(ADC_GetCalibrationStatus(ADC1));      //读取并判断CAL，CAL为0跳出while语句
}

//DMA1 初始化函数
void InitDMA1Ch1ForADC1(void) {
  //使能RCC相关时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //使能DMA1的时钟
  DMA_DeInit(DMA1_Channel1);

  DMA_InitTypeDef DMA_InitStructure;  //DMA_InitStructure用于存放DMA的参数

  //配置DMA1_Channel1
  DMA_DeInit(DMA1_Channel1);  //将DMA1_CH1寄存器设置为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);           //设置外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&s_iADC1Data;        //设置存储器地址
  DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;           //设置为外设到存储器模式
  DMA_InitStructure.DMA_BufferSize         = 1;                               //设置要传输的数据项数目
  DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //设置外设为非递增模式
  DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //设置存储器为递增模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //设置外设数据长度为半字
  DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;     //设置存储器数据长度为半字
  DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //设置为循环模式
  DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;             //设置为中等优先级
  DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;                 //禁止存储器到存储器访问
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);  //根据参数初始化DMA1_Channel1

  DMA_Cmd(DMA1_Channel1, ENABLE); //使能DMA1_Channel1

  printf("DMA config success.\n");
}

//EXTI初始化函数
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

  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // PC11和PC12
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
    // 处理PC11中断
    currentState = STATE_SPO2;
    EXTI_ClearITPendingBit(EXTI_Line11);
  }
  if(EXTI_GetFlagStatus(EXTI_Line12) != RESET) {
    // 处理PC12中断
    currentState = STATE_ECG;
    EXTI_ClearITPendingBit(EXTI_Line12);
  }
}

void EXTI2_IRQHandler(void) {
  if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
    // 处理PD2中断
    currentState = STATE_TEMP;
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
}

//按键初始化函数
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

// SysTick 初始化函数
void InitSysTick(void) {
  SystemCoreClockUpdate();  //更新系统时钟
  if (SysTick_Config(SYSTICK_RELOAD_VALUE) != 0) {
    // 如果配置失败，进入死循环处理错误
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

// LED 初始化函数
void InitLED(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15; // PC14和PC15
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_SetBits(GPIOC, GPIO_Pin_14 | GPIO_Pin_15);  // 默认高电平
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

// USART1 初始化函数
void USART1_Init(void) {
  USART_DeInit(USART1);

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

  printf("USART1 config success.\n");
}

// 重定向 fputc 函数
int fputc(int ch, FILE *f) {
  // 等待直到 USART1 的数据寄存器空闲
  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {

  };
  // 将字符发送到 USART1
  USART_SendData(USART1, (u8) ch);
  // 等待发送完成
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {

  };
  return ch;
}

///////////////////////////////////////// SPO2_CONFIG //////////////////////////////////////////////
void SPO2_ConfigRedCS(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
}

void SPO2_ConfigIRCS(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOA, GPIO_Pin_3);
}

///////////////////////////////////////// ECG_CONFIG //////////////////////////////////////////////
void ECG_ConfigZERO(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef  GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;

  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOC, GPIO_Pin_13);            //ECG_ZERO默认设置为低电平
}

///////////////////////////////////////// TEMP_CONFIG //////////////////////////////////////////////
void TEMP_ConfigPAPB(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  // 禁用JTAG，保留SWD，释放PB3和PB4引脚
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_ResetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_4);
}

void TEMP_ConfigSENS(void) {
  // 使能GPIOC时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  // 配置PC8和PC9为推挽输出
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  // 设置PC8为高电平，PC9为低电平
  GPIO_SetBits(GPIOC, GPIO_Pin_8);
  GPIO_ResetBits(GPIOC, GPIO_Pin_9);
}

void TEMP_ConfigSENSOFF(void) {
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// 状态切换配置函数
void ConfigureForSpO2(void) {
  SPO2_ConfigRedCS();
  SPO2_ConfigIRCS();
}

void ConfigureForECG(void) {
  ECG_ConfigZERO();
}

void ConfigureForTemperature(void) {
  TEMP_ConfigPAPB();
  TEMP_ConfigSENS();
}

// 外设初始化函数
void PeripheralInit(void) {
  USART1_Init();   // 初始化USART1，用于串口通信，用于调试信息的输出和数据的串行传输

  InitSysTick();   // 初始化系统滴答定时器(SysTick)，用于生成系统心跳，并提供周期性中断

  InitEXTI();      // 初始化外部中断(EXTI)，用于按键等外部事件的捕获，响应外部引脚的电平变化

  InitTimer4ForDAC1();  // 初始化TIM4，用于DAC1的触发配置，生成定时信号控制DAC输出的更新

  InitTimer3ForADC1();    // 初始化TIM3，用于ADC模块的触发配置，生成定时信号控制ADC的采样

  InitTimer2ForTask();    // 初始化TIM2，用于其他定时功能，提供系统中其他任务所需的定时中断

  InitDMA1Ch1ForADC1();   // 初始化DMA1通道1，用于提高ADC数据的传输效率，将ADC数据直接传输到内存

  InitDAC();  // 初始化DAC，用于将数字信号转换为模拟信号，控制DAC的输出

  InitADC();  // 初始化ADC，用于模拟信号的采样和转换，将模拟输入信号转换为数字信号

  InitLED();  // 初始化LED，用于指示灯的控制，设置GPIO引脚用于控制LED的开关状态

  InitKey();  // 初始化按键，用于用户输入的检测，设置GPIO引脚用于按键输入的读取
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

// 处理状态切换
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

///////////////////////////////////////// SPO2_HANDLE //////////////////////////////////////////////
void SPO2Wave(void) {
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  printf("s_iADC1Data = %d\r\n", s_iADC1Data);
}

///////////////////////////////////////// ECG_HANDLE ///////////////////////////////////////////////
void ECGWave(void) {
  printf("s_iADC1Data = %d\r\n", s_iADC1Data);
}

///////////////////////////////////////// TEMP_HANDLE //////////////////////////////////////////////
void CalcTVal(void) {
  float VoltVal;
  float R;
  
  VoltVal = (s_iADC1Data * 3.3) / 4095;
  printf("VoltVal_With_Gain = %f\r\n", VoltVal);
  VoltVal = (float)(s_iADC1Data * 33 * 100) / (4095* 10 * 125);
  printf("VoltVal_Witout_Gain = %f\r\n", VoltVal);
  R = (VoltVal * 147 / 10) / (5 - VoltVal);
  printf("R = %f\r\n", R);
}

// 状态处理函数的实现
void HandleSpO2(void) {
  if(flagType.s_i2msFlag) {
    SPO2Wave();
    flagType.s_i2msFlag = false;
  }
}

void HandleECG(void) {
  if(flagType.s_i2msFlag) {
    ECGWave();
    flagType.s_i2msFlag = false;
  }
}

void HandleTemperature(void) {
  if(flagType.s_i1secFlag) {
    CalcTVal();
    flagType.s_i1secFlag = false;
  }
}

// 执行当前状态的任务
void ExecuteCurrentState(void) {
  switch (currentState) {
    case STATE_SPO2:
      FlickerLED();
      HandleSpO2();
      break;
    case STATE_ECG:
      FlickerLED();
      HandleECG();
      break;
    case STATE_TEMP:
      FlickerLED();
      HandleTemperature();
      break;
    default:
      break;
  }
}

// 测试
void Test(void) {
  u16 dacValue = DAC->DHR12R1;
  u16 adcValue = ADC1->DR;
  u16 dmaValue = s_iADC1Data;

  // 打印DAC通道1输出值
  printf("DAC Value High Byte: %02X, Low Byte: %02X\r\n", (dacValue >> 8) & 0xFF, dacValue & 0xFF);

  // 打印ADC转换结果值
  printf("ADC Value High Byte: %02X, Low Byte: %02X\r\n", (adcValue >> 8) & 0xFF, adcValue & 0xFF);

  // 打印DMA传输的ADC数据
  printf("DMA Value High Byte: %02X, Low Byte: %02X\r\n", (dmaValue >> 8) & 0xFF, dmaValue & 0xFF);

  // 打印读取到的DAC数据
  printf("Current DAC Channel 1 Output Value (Decimal): %d\r\n", DAC->DHR12R1);
  printf("Current DAC Channel 1 Output Value (Hex): 0x%X\r\n", DAC->DHR12R1);

  // 打印ADC转换结果寄存器的值
  printf("Current ADC Conversion Value (Decimal): %d\r\n", ADC1->DR);
  printf("Current ADC Conversion Value (Hex): 0x%X\r\n", ADC1->DR);

  // 打印DMA传输的ADC数据
  printf("DMA Transferred ADC Data (Decimal): %d\r\n", s_iADC1Data);
  printf("DMA Transferred ADC Data (Hex): 0x%X\r\n", s_iADC1Data);
}

// 执行其他后台任务
void PerformBackgroundTasks(void) {
  // 进行ADC和DMA的测试
  if(flagType.s_i1secFlag) {
    Test();
    flagType.s_i1secFlag = false;
  }
  // 执行其他周期性任务或后台任务
}

// 状态机处理函数
void StateMachine(void) {
  while (1) {
    HandleStateChange();    // 处理状态切换
    ExecuteCurrentState();  // 执行当前状态的任务
//    PerformBackgroundTasks();  // 执行其他后台任务
  }
}

// 主函数
int main(void) {
  // 系统初始化
  SystemInit();

  StateMachine();

  return 0;
}
