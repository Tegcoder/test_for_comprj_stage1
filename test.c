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

// 定义ARR和PSC值
#define SPO2_PSC   71
#define SPO2_ARR   499
#define ECG_PSC    71
#define ECG_ARR    99
#define TEMP_PSC   71
#define TEMP_ARR   7999

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
  bool s_i500usFlag;    //将500us标志位的值设置为FALSE
  bool s_i2msFlag;      //将2ms标志位的值设置为FALSE
  bool s_i1secFlag;     //将1s标志位的值设置为FALSE
} StructFlagTypeDef;

/*********************************************************************************************************
*                                              内部变量
*********************************************************************************************************/
EnumSystemState currentState = STATE_MAX;
EnumSystemState lastState = STATE_IDLE; // 使用辅助值表示未初始化状态
StructFlagTypeDef flagType;

static __IO u32 s_iSysTickDelayCnt = DELAY_250_MS;
static u16 s_arrADC1Data;   //存放ADC转换结果数据

/*********************************************************************************************************
*                                              内部函数实现
*********************************************************************************************************/
//TIM3定时器初始化函数
void InitTimer3(void) {
  TIM_DeInit(TIM3);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

switch(currentState) {
  case STATE_SPO2:
    TIM_TimeBaseInitStructure.TIM_Prescaler = SPO2_PSC; // 72MHz / (71 + 1) = 1MHz，定时器时钟频率为 1MHz
    TIM_TimeBaseInitStructure.TIM_Period =    SPO2_ARR; // 1MHz / (499 + 1) = 2kHz（500μs），每 500μs 触发一次
    printf("Configured STATE_SPO2: Prescaler %d, Auto-reload %d\n", SPO2_PSC, SPO2_ARR);
    break;

  case STATE_ECG:
    TIM_TimeBaseInitStructure.TIM_Prescaler = ECG_PSC; // 72MHz / (71 + 1) = 1MHz，定时器时钟频率为 1MHz
    TIM_TimeBaseInitStructure.TIM_Period =    ECG_ARR; // 1MHz / (9999 + 1) = 100Hz（10ms），每 10ms 触发一次
    printf("Configured STATE_ECG: Prescaler %d, Auto-reload %d\n", ECG_PSC, ECG_ARR);
    break;

  case STATE_TEMP:
    TIM_TimeBaseInitStructure.TIM_Prescaler = TEMP_PSC; // 72MHz / (71 + 1) = 1MHz，定时器时钟频率为 1MHz
    TIM_TimeBaseInitStructure.TIM_Period =    TEMP_ARR; // 1MHz / (9999 + 1) = 100Hz（10ms），每 10ms 触发一次
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

//TIM2定时器初始化函数
void InitTimer2(void) {
  TIM_DeInit(TIM2);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  //配置TIM2提供基础的100us延时
  TIM_TimeBaseInitStructure.TIM_Prescaler = 71;
  TIM_TimeBaseInitStructure.TIM_Period =    99;
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

//通用ADC配置
void InitADC(void) {
  ADC_DeInit(ADC1);

  // 使能ADC相关时钟
  RCC_ADCCLKConfig(RCC_PCLK2_Div6); // 设置ADC时钟分频，ADCCLK=PCLK2/6=12MHz
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);  // 使能ADC1的时钟

  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef  ADC_InitStructure;

  // 配置GPIO和ADC
  switch (currentState) {
    case STATE_SPO2:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOA的时钟

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None; // 使用软件触发
      ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期

      printf("Configured STATE_SPO2: ADC_Channel_5, GPIOA_Pin_5, using software trigger\n");
      break;

    case STATE_ECG:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); // 使能GPIOC的时钟

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOC, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO; // 使用TIM3触发
      ADC_ExternalTrigConvCmd(ADC1, ENABLE); // 使用外部事件启动ADC转换
      ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期

      printf("Configured STATE_ECG: ADC_Channel_14, GPIOC_Pin_4, using TIM3 trigger\n");
      break;

    case STATE_TEMP:
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOA的时钟

      GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6;
      GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
      GPIO_Init(GPIOA, &GPIO_InitStructure);

      ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_T3_TRGO; // 使用TIM3触发
      ADC_ExternalTrigConvCmd(ADC1, ENABLE); // 使用外部事件启动ADC转换
      ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5); //设置采样时间为239.5个周期

      printf("Configured STATE_TEMP: ADC_Channel_6, GPIOA_Pin_6, using TIM3 trigger\n");
      break;

    default:
      printf("Unspecified state!ADC configuration failure.\n");
      break;
  }

  // ADC通用配置部分
  ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel       = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  ADC_DMACmd(ADC1, ENABLE); // 使能DMA
  ADC_Cmd(ADC1, ENABLE); // 使能ADC

  ADC_ResetCalibration(ADC1); // 启动ADC复位校准
  while (ADC_GetResetCalibrationStatus(ADC1)); // 等待复位校准完成
  ADC_StartCalibration(ADC1); // 启动ADC校准
  while (ADC_GetCalibrationStatus(ADC1)); // 等待校准完成
}

//DMA1 初始化函数
void InitDMA(void) {
  DMA_DeInit(DMA1_Channel1);

  DMA_InitTypeDef DMA_InitStructure;  //DMA_InitStructure用于存放DMA的参数

  //使能RCC相关时钟
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  //使能DMA1的时钟

  //配置DMA1_Channel1
  DMA_DeInit(DMA1_Channel1);  //将DMA1_CH1寄存器设置为默认值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);           //设置外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)&s_arrADC1Data;        //设置存储器地址
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

// 状态切换配置函数
void ConfigureForSpO2(void) {
}

void ConfigureForECG(void) {
}

void ConfigureForTemperature(void) {
}

// 外设初始化函数
void PeripheralInit(void) {
  USART1_Init();   // 初始化USART1，用于串口通信

  InitSysTick();   // 初始化系统滴答定时器，用于生成系统心跳

  InitEXTI();      // 初始化外部中断，用于按键等外部事件的捕获

  InitTimer3();    // 初始化TIM3，用于ADC模块的触发配置

  InitTimer2();    // 初始化TIM2，用于其他定时功能

  InitADC();       // 初始化ADC，用于模拟信号的采样和转换

  InitDMA();       // 初始化DMA，用于提高数据传输效率

  InitLED();       // 初始化LED，用于指示灯的控制

  InitKey();       // 初始化按键，用于用户输入的检测
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

// 状态处理函数的实现
void HandleSpO2(void) {
  FlickerLED();
}

void HandleECG(void) {
  FlickerLED();
}

void HandleTemperature(void) {
  FlickerLED();
}

// 执行当前状态的任务
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

// 测试
void Test(void) {
    // 打印DMA传输的ADC数据
    printf("ADC: %d\r\n", s_arrADC1Data);

    // 测试 1s 标志位
    if (flagType.s_i1secFlag) {
        printf("1secflag is ready!\n");
        flagType.s_i1secFlag = false;
    }

    // 测试 500us 标志位
    if (flagType.s_i500usFlag) {
        printf("500usflag is ready!\n");
        flagType.s_i500usFlag = false;
    }

    // 测试 2ms 标志位
    if (flagType.s_i2msFlag) {
        printf("2msflag is ready!\n");
        flagType.s_i2msFlag = false;
    }
}

// 执行其他后台任务
void PerformBackgroundTasks(void) {
  // 进行ADC和DMA的测试
  Test();
  // 执行其他周期性任务或后台任务
}

// 状态机处理函数
void StateMachine(void) {
  while (1) {
    HandleStateChange();    // 处理状态切换
    ExecuteCurrentState();  // 执行当前状态的任务
    PerformBackgroundTasks();  // 执行其他后台任务
  }
}

// 主函数
int main(void) {
  // 系统初始化
  SystemInit();

  StateMachine();

  return 0;
}
