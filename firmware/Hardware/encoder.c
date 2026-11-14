#include "stm32f10x.h"                  // Device header
#include "encoder.h"

/*
     ENCODER1   --------- ENCODER2
*/

// volatile: the EXTI handlers below update these behind the main loop's back
static volatile struct ENCODER Encoder1={0,0,0,0,0,{0},0};
static volatile struct ENCODER Encoder2={0,0,0,0,0,{0},0};
void Encoder_Init()
{
	Encoder_Gpio_Init();
	Encoder_Count_FIFO_Init();

	// Start from the real pin levels so the first edge is decoded correctly
	Encoder1.A = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
	Encoder1.B = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
	Encoder2.A = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
	Encoder2.B = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
}

int32_t Encoder1_Count_Get()
{
	return Encoder1.Avg_PulseCount;
}

int32_t Encoder2_Count_Get()
{
	return  Encoder2.Avg_PulseCount;
}

// Cumulative counts since boot (32-bit reads are atomic on the Cortex-M3)
int32_t Encoder1_Total_Get()
{
	return Encoder1.Total;
}

int32_t Encoder2_Total_Get()
{
	return Encoder2.Total;
}

void Encoder_Updata()
{
	Encoder1_Updata();
	Encoder2_Updata();
}
/**************************************************/

void Encoder1_Updata()
{
	Encoder1_Count_FIFO_Updata(Encoder1.PulseCount);
	Encoder1_AvgPulseCount_Updata();
	Encoder1.PulseCount = 0;
}

void Encoder2_Updata()
{
	Encoder2_Count_FIFO_Updata(Encoder2.PulseCount);	
	Encoder2_AvgPulseCount_Updata();	
	Encoder2.PulseCount = 0;
}

void Encoder_Gpio_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	// EC1 (PC0/PC1) and EC2 (PC2/PC3) are decoded in software on every edge.
	// EC3/EC4 (PC6..PC9) are unused: no EXTI, so a floating header can never
	// raise an interrupt. Pull-ups keep unplugged inputs quiet instead of
	// floating (an unplugged EC1/EC2 cable cannot flood the CPU with edges).
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|
								  GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource2);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);

	// Lines 0..3, both edges
	EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

// EXTI 5..9 are never enabled. This handler only guarantees that a stray
// pending bit can't land in the startup file's endless Default_Handler.
void EXTI9_5_IRQHandler()
{
	EXTI_ClearITPendingBit(EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9);
}

/************* ENCODER 1 ***********************/
void EXTI0_IRQHandler() 
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		// 外部中断0触发，电机正向运动
		Encoder1.A = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		if (Encoder1.A == Encoder1.B) 
		{
			Encoder1.PulseCount--;
			Encoder1.Total--;
		} 
		else
		{
			Encoder1.PulseCount++;
			Encoder1.Total++;
		}
	}
}

void EXTI1_IRQHandler() 
{
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) 
	{
		// 外部中断1触发，电机反向运动
		Encoder1.B = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1);
		EXTI_ClearITPendingBit(EXTI_Line1);
		if (Encoder1.A == Encoder1.B) 
		{
			Encoder1.PulseCount++;
			Encoder1.Total++;
		} 
		else
		{
			Encoder1.PulseCount--;
			Encoder1.Total--;
		}
	}
}

/********************************************************/

/************* ENCODER 2 ***********************/
void EXTI2_IRQHandler() 
{
	if (EXTI_GetITStatus(EXTI_Line2) != RESET)
	{
		// 外部中断0触发，电机正向运动
		Encoder2.A = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2);
		EXTI_ClearITPendingBit(EXTI_Line2);
		if (Encoder2.A == Encoder2.B) 
		{
			Encoder2.PulseCount--;
			Encoder2.Total--;
		} 
		else
		{
			Encoder2.PulseCount++;
			Encoder2.Total++;
		}
	}
}

void EXTI3_IRQHandler() 
{
	if (EXTI_GetITStatus(EXTI_Line3) != RESET) 
	{
		// 外部中断1触发，电机反向运动
		Encoder2.B = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
		EXTI_ClearITPendingBit(EXTI_Line3);
		//Encoder2_Detect_Direction();
		if (Encoder2.A == Encoder2.B) 
		{
			Encoder2.PulseCount++;
			Encoder2.Total++;
		} 
		else
		{
			Encoder2.PulseCount--;
			Encoder2.Total--;
		}
		
	}
}


/********************************************************/
void Encoder_Count_FIFO_Init()
{
	for (uint8_t i = 0; i < COUNT_SIZE; i++)
	{
		Encoder1.Count_FIFO[i] = 0;
		Encoder2.Count_FIFO[i] = 0;
	}
}

void Encoder1_Count_FIFO_Updata(int32_t value)
{
	if (Encoder1.pFIFO == COUNT_SIZE)
	{
		for (uint8_t i = 0; i < COUNT_SIZE - 1; i++)
		{
			Encoder1.Count_FIFO[i] = Encoder1.Count_FIFO[i+1];
		}
		Encoder1.pFIFO--;
	}

	Encoder1.Count_FIFO[Encoder1.pFIFO] = value;
	Encoder1.pFIFO++;
}

void Encoder2_Count_FIFO_Updata(int32_t value)
{
	if (Encoder2.pFIFO == COUNT_SIZE)
	{
		for (uint8_t i = 0; i < COUNT_SIZE - 1; i++)
		{
			Encoder2.Count_FIFO[i] = Encoder2.Count_FIFO[i+1];
		}
		Encoder2.pFIFO--;
	}

	Encoder2.Count_FIFO[Encoder2.pFIFO] = value;
	Encoder2.pFIFO++;
}

void Encoder1_AvgPulseCount_Updata()
{
	uint8_t i = 0; 
	int32_t sum = 0;
	
	for(i = 0; i < COUNT_SIZE; i++)
	{
		sum += Encoder1.Count_FIFO[i];
	}
	 
	Encoder1.Avg_PulseCount = sum >> 2;
	
}

void Encoder2_AvgPulseCount_Updata()
{
	uint8_t i = 0; 
	int32_t sum = 0;
	
	for(i = 0; i < COUNT_SIZE; i++)
	{
		sum += Encoder2.Count_FIFO[i];
	}
	 
	Encoder2.Avg_PulseCount = sum >> 2;
}
