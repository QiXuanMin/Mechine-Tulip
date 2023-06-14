#include "ws2812b.h"
#include "delay.h"

/* PB6复用功能TIM4_CH1*/
#define WSPIN   GPIO_PIN_6
#define WSPORT	GPIOB
#define WS_LOW  PBout(6)=0
#define WS_HIGH PBout(6)=1
/*TIM+DMA输出*/
#define BIT_1                   61u
#define BIT_0                   28u

#define PIXEL_MAX 7
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
uint8_t rBuffer[PIXEL_MAX] = {0};
uint8_t gBuffer[PIXEL_MAX] = {0};
uint8_t bBuffer[PIXEL_MAX] = {0};
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
typedef struct
{
    const uint16_t head[3];              //先发送3个0等待dma稳定
    uint16_t data[24 * PIXEL_MAX];       //真正的数据
    const uint16_t tail;                 //最后发送一个0，保证dma结束后，pwm输出低
} frame_buf_ST;

frame_buf_ST frame = { .head[0] = 0,
                       .head[1] = 0,
                       .head[2] = 0,
                       .tail    = 0,
                     };





//因为用到的引脚是PB6，对应的复用定时器是TIM4CH1
TIM_HandleTypeDef 	TIM4_Handler;      	//定时器句柄 
TIM_OC_InitTypeDef 	TIM4_CH1Handler;	//定时器4通道1句柄
DMA_HandleTypeDef  	TIM4_DMA_Handler;	//定时器4的DMA句柄


void DMA_Init()
{
  /* DMA时钟使能 */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA中断初始化 */
  /* DMA1CH1中断服务函数配置 */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  if(timHandle->Instance==TIM4)
  {
    /*TIM4 GPIO Configuration    
    PB6     ------> TIM4_CH1 
    */
	__HAL_RCC_GPIOB_CLK_ENABLE();           	//开启GPIOB时钟
    GPIO_InitStruct.Pin = WSPIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(WSPORT, &GPIO_InitStruct);
  }

}

void TIM4_Init()
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  TIM4_Handler.Instance = TIM4;
  TIM4_Handler.Init.Prescaler = 0;
  TIM4_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM4_Handler.Init.Period = 89;
  TIM4_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&TIM4_Handler);//
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&TIM4_Handler, &sClockSourceConfig);//
	HAL_TIM_PWM_Init(&TIM4_Handler);//
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&TIM4_Handler, &sMasterConfig);//
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &sConfigOC, TIM_CHANNEL_1);//
  HAL_TIM_MspPostInit(&TIM4_Handler);

}

//TIM和DMA的关联初始化
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM4)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();

    /* Peripheral DMA init*/
    TIM4_DMA_Handler.Instance = DMA1_Channel1;
    TIM4_DMA_Handler.Init.Direction = DMA_MEMORY_TO_PERIPH;
    TIM4_DMA_Handler.Init.PeriphInc = DMA_PINC_DISABLE;
    TIM4_DMA_Handler.Init.MemInc = DMA_MINC_ENABLE;
    TIM4_DMA_Handler.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    TIM4_DMA_Handler.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    TIM4_DMA_Handler.Init.Mode = DMA_NORMAL;
    TIM4_DMA_Handler.Init.Priority = DMA_PRIORITY_MEDIUM;
		HAL_DMA_Init(&TIM4_DMA_Handler);
    /* Several peripheral DMA handle pointers point to the same DMA handle.
     Be aware that there is only one channel to perform all the requested DMAs. */
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_CC1],TIM4_DMA_Handler);
    __HAL_LINKDMA(tim_baseHandle,hdma[TIM_DMA_ID_TRIGGER],TIM4_DMA_Handler);

  }
}

//TIM和DMA的关联反初始化
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
  if(tim_baseHandle->Instance==TIM4)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM3_CLK_DISABLE();

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_CC1]);
    HAL_DMA_DeInit(tim_baseHandle->hdma[TIM_DMA_ID_TRIGGER]);
  }
}



//WS2812B初始显示
void ws2812b_show_init(void)
{
    int8_t i, j;

    for(i = 0; i < PIXEL_MAX; i++)
    {
		rBuffer[i]=0xff;
		gBuffer[i]=0xff;
		bBuffer[i]=0xff;
        for(j = 0; j < 8; j++)
        {
            frame.data[24 * i + j]     = (gBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
            frame.data[24 * i + j + 8]   = (rBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
            frame.data[24 * i + j + 16]  = (bBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
        }
    }
    HAL_TIM_PWM_Start_DMA(&TIM4_Handler, TIM_CHANNEL_1, (uint32_t *)&frame, 3 + 24 * PIXEL_MAX + 1);
}


//WS2812B初始化
void ws2812b_init()
{
	//DMA初始化
	DMA_Init();
	//PWM初始化关联
	TIM4_Init();
	//延时
	//delay_ms(100);
	//点亮WS2812B
	ws2812b_show_rainbow();
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}


/**设置全部WS2812B的颜色
输入8位的GRB值*/
void  setAllPixelColor(uint8_t g, uint8_t r, uint8_t b)
{
    uint8_t i = 0;
    for(i = 0; i < PIXEL_MAX; i++)
    {
        gBuffer[i] = 0;
        rBuffer[i] = 0;
        bBuffer[i] = 0;
    }
    for(i = 0; i < PIXEL_MAX; i++)
    {
        gBuffer[i] = g;
        rBuffer[i] = r;
        bBuffer[i] = b;
    }
}

/**设置全部WS2812B的颜色
输入32位的GRB值*/
void  SetAllPixelColor(uint32_t c)
{
    uint8_t i = 0;
    for(i = 0; i < PIXEL_MAX; i++)
    {
        gBuffer[i] = (uint8_t)(c >> 16);
        rBuffer[i] = (uint8_t)(c >> 8);
        bBuffer[i] = (uint8_t)c;
    }
}

/**设置第N个WS2812B的颜色
输入8位的GRB值*/
void setPixelColor(uint16_t n, uint8_t g, uint8_t r, uint8_t b)
{
    if(n < PIXEL_MAX)
    {
        gBuffer[n] = g;
        rBuffer[n] = r;
        bBuffer[n] = b;
    }
}

/**设置第N个WS2812B的颜色
输入32位的GRB值*/
void SetPixelColor(uint16_t n, uint32_t c)
{
    if(n < PIXEL_MAX)
    {
        gBuffer[n] = (uint8_t)(c >> 16);
        rBuffer[n] = (uint8_t)(c >> 8);
        bBuffer[n] = (uint8_t)c;
    }

}

/**输入8位的GRB值
输出32位的GRB值*/
uint32_t Color(uint8_t g, uint8_t r, uint8_t b)
{
    return ((uint32_t)g << 16) | ((uint32_t)r <<  8) | b;
}

/**彩虹色显示
红橙黄绿青蓝紫*/
void ws2812b_show_rainbow()
{
	//开始的时候开启DMA中断
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
    int8_t i, j;
	SetPixelColor(0,Crimson);
	SetPixelColor(1,Orange);
	SetPixelColor(2,Yellow);
	SetPixelColor(3,Lime);
	SetPixelColor(4,DarkCyan);
	SetPixelColor(5,RoyalBlue);
	SetPixelColor(6,Violet);

    for(i = 0; i < PIXEL_MAX; i++)
    {
        for(j = 0; j < 8; j++)
        {
            frame.data[24 * i + j]     = (gBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
            frame.data[24 * i + j + 8]   = (rBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
            frame.data[24 * i + j + 16]  = (bBuffer[i] & (0x80 >> j)) ? BIT_1 : BIT_0;
        }
    }
    HAL_TIM_PWM_Start_DMA(&TIM4_Handler, TIM_CHANNEL_1, (uint32_t *)&frame, 3 + 24 * PIXEL_MAX + 1);
	//在结束的时候关闭DMA中断
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}






















/** 过去的程序，尝试失败，因为GPIO的速度限制不能纳秒级翻转
void ws2812b_init()
{
//相关的GPIO初始化
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           	//开启GPIOB时钟
    GPIO_Initure.Pin=GPIO_PIN_6;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          	//上拉
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;    	 	//高速
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);	//默认初始化后灯灭
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

//通过定时器的形式模拟波长
//TIM4 PWM部分初始化 
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
    TIM4_Handler.Instance=TIM4;         													//定时器4
    TIM4_Handler.Init.Prescaler=0;       													//定时器分频
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;										//向上计数模式
    TIM4_Handler.Init.Period=90-1;          												//自动重装载值
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;									//分频因子
	TIM4_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;					//使能自动重载
    HAL_TIM_PWM_Init(&TIM4_Handler);       													//初始化PWM
    
    TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; 												//模式选择PWM1
    TIM4_CH1Handler.Pulse=0;            													//设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
    TIM4_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH;											//输出比较极性为低 
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);				//配置TIM4通道1
	
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);											//开启PWM通道1

//DMA1初始化
	__HAL_RCC_DMA1_CLK_ENABLE();															//DMA1时钟使能
	__HAL_LINKDMA(&TIM4_Handler,hdma[TIM_DMA_ID_CC1],TIM4_DMA_Handler);						//将DMA与TIM4联系起来(发送DMA)

    TIM4_DMA_Handler.Instance=DMA1_Channel1;						  						//通道选择//
    TIM4_DMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             						//存储器到外设//
    TIM4_DMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 						//外设非增量模式//
    TIM4_DMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     						//存储器增量模式//
    TIM4_DMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;    					//外设数据长度:8位
    TIM4_DMA_Handler.Init.MemDataAlignment=DMA_PDATAALIGN_HALFWORD;       					//存储器数据长度:8位
    TIM4_DMA_Handler.Init.Mode=DMA_NORMAL;                          						//外设普通模式//
    TIM4_DMA_Handler.Init.Priority=DMA_PRIORITY_HIGH;               						//高优先级//
    
    HAL_DMA_DeInit(&TIM4_DMA_Handler);   
    HAL_DMA_Init(&TIM4_DMA_Handler);
}




void ws2812b_reset()
{
      WS_LOW;                        //DI置为0后，延时50us以上，实现帧复位
      delay_us(65);
}

void ws2812_write0()
{
      WS_HIGH;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();//83.4NS
      WS_LOW;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
}

void ws2812_write1()
{
      WS_HIGH;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
	__nop();__nop();
      WS_LOW;
	__nop();__nop();__nop();__nop();__nop();__nop();__nop();__nop();
}


void WS_Write_Byte(uint8 byte)
{
      uint8 i;
      for(i=0;i<8;i++)
      {
            if(byte & 0x80)
            {
                  ws2812_write1();
            }
            else
            {
                  ws2812_write0();
            }
            byte <<= 1;
      }
}

void WS_Write_24Bits(uint8_t green,uint8_t red,uint8_t blue)
{
	WS_Write_Byte(green);
	WS_Write_Byte(red);
	WS_Write_Byte(blue);
}

void ws2812_rgb_test()
{
      uint8 i;
      ws2812b_reset();
      for(i=0;i<1;i++)
      {
            WS_Write_24Bits(0xff, 0xff, 0xff);
      }
      ws2812b_reset();
}

*/
