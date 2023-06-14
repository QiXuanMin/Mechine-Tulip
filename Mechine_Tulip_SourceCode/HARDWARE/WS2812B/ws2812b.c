#include "ws2812b.h"
#include "delay.h"

/* PB6���ù���TIM4_CH1*/
#define WSPIN   GPIO_PIN_6
#define WSPORT	GPIOB
#define WS_LOW  PBout(6)=0
#define WS_HIGH PBout(6)=1
/*TIM+DMA���*/
#define BIT_1                   61u
#define BIT_0                   28u

#define PIXEL_MAX 7
/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
uint8_t rBuffer[PIXEL_MAX] = {0};
uint8_t gBuffer[PIXEL_MAX] = {0};
uint8_t bBuffer[PIXEL_MAX] = {0};
/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
typedef struct
{
    const uint16_t head[3];              //�ȷ���3��0�ȴ�dma�ȶ�
    uint16_t data[24 * PIXEL_MAX];       //����������
    const uint16_t tail;                 //�����һ��0����֤dma������pwm�����
} frame_buf_ST;

frame_buf_ST frame = { .head[0] = 0,
                       .head[1] = 0,
                       .head[2] = 0,
                       .tail    = 0,
                     };





//��Ϊ�õ���������PB6����Ӧ�ĸ��ö�ʱ����TIM4CH1
TIM_HandleTypeDef 	TIM4_Handler;      	//��ʱ����� 
TIM_OC_InitTypeDef 	TIM4_CH1Handler;	//��ʱ��4ͨ��1���
DMA_HandleTypeDef  	TIM4_DMA_Handler;	//��ʱ��4��DMA���


void DMA_Init()
{
  /* DMAʱ��ʹ�� */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA�жϳ�ʼ�� */
  /* DMA1CH1�жϷ��������� */
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
	__HAL_RCC_GPIOB_CLK_ENABLE();           	//����GPIOBʱ��
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

//TIM��DMA�Ĺ�����ʼ��
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

//TIM��DMA�Ĺ�������ʼ��
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



//WS2812B��ʼ��ʾ
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


//WS2812B��ʼ��
void ws2812b_init()
{
	//DMA��ʼ��
	DMA_Init();
	//PWM��ʼ������
	TIM4_Init();
	//��ʱ
	//delay_ms(100);
	//����WS2812B
	ws2812b_show_rainbow();
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}


/**����ȫ��WS2812B����ɫ
����8λ��GRBֵ*/
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

/**����ȫ��WS2812B����ɫ
����32λ��GRBֵ*/
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

/**���õ�N��WS2812B����ɫ
����8λ��GRBֵ*/
void setPixelColor(uint16_t n, uint8_t g, uint8_t r, uint8_t b)
{
    if(n < PIXEL_MAX)
    {
        gBuffer[n] = g;
        rBuffer[n] = r;
        bBuffer[n] = b;
    }
}

/**���õ�N��WS2812B����ɫ
����32λ��GRBֵ*/
void SetPixelColor(uint16_t n, uint32_t c)
{
    if(n < PIXEL_MAX)
    {
        gBuffer[n] = (uint8_t)(c >> 16);
        rBuffer[n] = (uint8_t)(c >> 8);
        bBuffer[n] = (uint8_t)c;
    }

}

/**����8λ��GRBֵ
���32λ��GRBֵ*/
uint32_t Color(uint8_t g, uint8_t r, uint8_t b)
{
    return ((uint32_t)g << 16) | ((uint32_t)r <<  8) | b;
}

/**�ʺ�ɫ��ʾ
��Ȼ���������*/
void ws2812b_show_rainbow()
{
	//��ʼ��ʱ����DMA�ж�
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
	//�ڽ�����ʱ��ر�DMA�ж�
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
}






















/** ��ȥ�ĳ��򣬳���ʧ�ܣ���ΪGPIO���ٶ����Ʋ������뼶��ת
void ws2812b_init()
{
//��ص�GPIO��ʼ��
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_GPIOB_CLK_ENABLE();           	//����GPIOBʱ��
    GPIO_Initure.Pin=GPIO_PIN_6;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;  	//�������
    GPIO_Initure.Pull=GPIO_PULLUP;          	//����
    GPIO_Initure.Speed=GPIO_SPEED_HIGH;    	 	//����
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);	//Ĭ�ϳ�ʼ�������
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);

//ͨ����ʱ������ʽģ�Ⲩ��
//TIM4 PWM���ֳ�ʼ�� 
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
    TIM4_Handler.Instance=TIM4;         													//��ʱ��4
    TIM4_Handler.Init.Prescaler=0;       													//��ʱ����Ƶ
    TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;										//���ϼ���ģʽ
    TIM4_Handler.Init.Period=90-1;          												//�Զ���װ��ֵ
    TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;									//��Ƶ����
	TIM4_Handler.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;					//ʹ���Զ�����
    HAL_TIM_PWM_Init(&TIM4_Handler);       													//��ʼ��PWM
    
    TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; 												//ģʽѡ��PWM1
    TIM4_CH1Handler.Pulse=0;            													//���ñȽ�ֵ,��ֵ����ȷ��ռ�ձȣ�Ĭ�ϱȽ�ֵΪ�Զ���װ��ֵ��һ��,��ռ�ձ�Ϊ50%
    TIM4_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH;											//����Ƚϼ���Ϊ�� 
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);				//����TIM4ͨ��1
	
    HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);											//����PWMͨ��1

//DMA1��ʼ��
	__HAL_RCC_DMA1_CLK_ENABLE();															//DMA1ʱ��ʹ��
	__HAL_LINKDMA(&TIM4_Handler,hdma[TIM_DMA_ID_CC1],TIM4_DMA_Handler);						//��DMA��TIM4��ϵ����(����DMA)

    TIM4_DMA_Handler.Instance=DMA1_Channel1;						  						//ͨ��ѡ��//
    TIM4_DMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             						//�洢��������//
    TIM4_DMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 						//���������ģʽ//
    TIM4_DMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     						//�洢������ģʽ//
    TIM4_DMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_HALFWORD;    					//�������ݳ���:8λ
    TIM4_DMA_Handler.Init.MemDataAlignment=DMA_PDATAALIGN_HALFWORD;       					//�洢�����ݳ���:8λ
    TIM4_DMA_Handler.Init.Mode=DMA_NORMAL;                          						//������ͨģʽ//
    TIM4_DMA_Handler.Init.Priority=DMA_PRIORITY_HIGH;               						//�����ȼ�//
    
    HAL_DMA_DeInit(&TIM4_DMA_Handler);   
    HAL_DMA_Init(&TIM4_DMA_Handler);
}




void ws2812b_reset()
{
      WS_LOW;                        //DI��Ϊ0����ʱ50us���ϣ�ʵ��֡��λ
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
