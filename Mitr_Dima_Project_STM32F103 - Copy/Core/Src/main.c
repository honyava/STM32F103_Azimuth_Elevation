/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "string.h"
#include "usbd_cdc_if.h"
#include "lcd.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_SIZE   512
#define FFT_SIZE       256 
#define NUMBER_IS_2_POW_K(x)   ((!((x)&((x)-1)))&&((x)>1))  // x is pow(2, k), k=1,2, ...
#define FT_DIRECT        -1    // Direct transform.
#define FT_INVERSE        1    // Inverse transform.
#define MENU_NUMB   9
#define DEGREE  57.29578       // Convert from radian to degrees

unsigned char Inversion[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

unsigned char Menu[MENU_NUMB][15] = {
 " Test MMC Card\0",
 " Test Audio   \0",
 " Test RTC     \0",
 " Test NRF     \0",
 " Test USB     \0",
 " Test Acceler.\0",
 " Test Port    \0",
 " Test UEXT    \0",
 " Test Joystick\0"
 // "              \0",
 // "              \0",
 // "www.olimex.com\0"
};

#define KEY_NONE    0
#define KEY_CENTER  5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
float phi[FFT_SIZE]={0,};
float Ampl[FFT_SIZE]={0,};
float Re[FFT_SIZE]={0};	
float Im[FFT_SIZE]={0};

float max1_9 = 0;
float after_max1_9 = 0;
float max1_10 = 0;
float after_max1_10 = 0;
float max2_9_10 = 0;
float after_max2_9_10 = 0;

float Ampl1_9 = 0;
float Ampl1_10 = 0;
float Ampl2_9_10 = 0;
float phi1_9 = 0;
float phi1_10 = 0;
float phi1_9_test = 0;
float phi1_10_test = 0;
float phi1_10_el = 0;
float phi1_9_el = 0;
float phi2_9_10 = 0;
float phi_end = 0;
float phi_end_elevation = 0;

uint8_t trans_str[15] = {'\0'};
uint8_t trans_str2[20] = {'\0'};
uint32_t ADC_MEASURE [ADC_BUF_SIZE];
float ADC1_9[ADC_BUF_SIZE/2]={0,},ADC2_9[ADC_BUF_SIZE/2]={0,},ADC1_10[((ADC_BUF_SIZE+1)/2)]={0,}/*,ADC2_10[((ADC_BUF_SIZE+1)/2)]={0,}*/;

volatile uint8_t flag = 0;
uint8_t counter = 0;
uint8_t flag_TIM1 = 0;
uint16_t flag_TIM1_2 = 0;
uint16_t i = 0;
uint16_t k = 0;
uint16_t j = 0;
uint16_t t = 0;
float sum1_9 = 0;
float sum2_9 = 0;
float sum1_10 = 0;
float sum2_10 = 0;
float sum_phi1_9 = 0;
float average1_9 = 0;
float average2_9 = 0;
float average1_10 = 0;
float average2_10 = 0;

unsigned char JoyPos=0;
unsigned char MenuPos=0;
unsigned char test_state=0;
unsigned char offset, menu_flag=0;

uint8_t buffer[4];
char str2[4] = "TEST";
extern uint8_t flag_usb;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void TIM1_Init(void) // for Button
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 36;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	NVIC_EnableIRQ (TIM1_UP_IRQn);
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}
void DMA_Start1(void)
{
  DMA_TypeDef *DMA1_s;
	DMA_Channel_TypeDef *DMA_Can1;
	DMA_Can1 = (void*)DMA1_Channel1_BASE;
	DMA1_s = (void*)DMA1_BASE;
  DMA_Can1->CPAR=ADC1_BASE+0x4C;
  DMA_Can1->CMAR=(uint32_t)&ADC_MEASURE;
	DMA_Can1->CNDTR=ADC_BUF_SIZE;
	DMA_Can1->CCR = DMA_CCR_CIRC | DMA_CCR_PSIZE_0| DMA_CCR_MSIZE_0| DMA_CCR_PSIZE_1| DMA_CCR_MSIZE_1| DMA_CCR_PL_1|DMA_CCR_MINC|DMA_CCR_TCIE|DMA_CCR_TEIE ;
	DMA_Can1->CCR = DMA_CCR_CIRC | DMA_CCR_PSIZE_0| DMA_CCR_MSIZE_0| DMA_CCR_PSIZE_1| DMA_CCR_MSIZE_1| DMA_CCR_PL_1|DMA_CCR_MINC|DMA_CCR_TCIE|DMA_CCR_TEIE|ENABLE;
  ADC1->CR2 |= ADC_CR2_DMA;
	
}

void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
  HAL_TIM_Base_Stop(&htim3);
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);
  flag = 1;
  /* USER CODE END DMA1_Channel1_IRQn 1 */
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
        ;
    }

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1) //check if the interrupt comes from TIM1
	{
		flag_TIM1 = 1;
		flag_TIM1_2++;
	}	
}

void COM_float_write(float f)
{
      uint8_t *ptr;  
      char i;
      ptr = (uint8_t *)&f;  
      for (i=0;i<4;i++) CDC_Transmit_FS(ptr+i, 1);
}


bool FFT(float *Rdat, float *Idat, int N, int LogN, int Ft_Flag)
{
  // parameters error check:
  if((Rdat == NULL) || (Idat == NULL))                  return false;
  if((N > 16384) || (N < 1))                            return false;
  if(!NUMBER_IS_2_POW_K(N))                             return false;
  if((LogN < 2) || (LogN > 14))                         return false;
  if((Ft_Flag != FT_DIRECT) && (Ft_Flag != FT_INVERSE)) return false;

  register int  i, j, n, k, io, ie, in, nn;
  float         ru, iu, rtp, itp, rtq, itq, rw, iw, sr;
  
  static const float Rcoef[14] =
  {  -1.0000000000000000F,  0.0000000000000000F,  0.7071067811865475F,
      0.9238795325112867F,  0.9807852804032304F,  0.9951847266721969F,
      0.9987954562051724F,  0.9996988186962042F,  0.9999247018391445F,
      0.9999811752826011F,  0.9999952938095761F,  0.9999988234517018F,
      0.9999997058628822F,  0.9999999264657178F
  };
  static const float Icoef[14] =
  {   0.0000000000000000F, -1.0000000000000000F, -0.7071067811865474F,
     -0.3826834323650897F, -0.1950903220161282F, -0.0980171403295606F,
     -0.0490676743274180F, -0.0245412285229122F, -0.0122715382857199F,
     -0.0061358846491544F, -0.0030679567629659F, -0.0015339801862847F,
     -0.0007669903187427F, -0.0003834951875714F
  };
  
  nn = N >> 1;
  ie = N;
  for(n=1; n<=LogN; n++)
  {
    rw = Rcoef[LogN - n];
    iw = Icoef[LogN - n];
    if(Ft_Flag == FT_INVERSE) iw = -iw;
    in = ie >> 1;
    ru = 1.0F;
    iu = 0.0F;
    for(j=0; j<in; j++)
    {
      for(i=j; i<N; i+=ie)
      {
        io       = i + in;
        rtp      = Rdat[i]  + Rdat[io];
        itp      = Idat[i]  + Idat[io];
        rtq      = Rdat[i]  - Rdat[io];
        itq      = Idat[i]  - Idat[io];
        Rdat[io] = rtq * ru - itq * iu;
        Idat[io] = itq * ru + rtq * iu;
        Rdat[i]  = rtp;
        Idat[i]  = itp;
      }

      sr = ru;
      ru = ru * rw - iu * iw;
      iu = iu * rw + sr * iw;
    }

    ie >>= 1;
  }

  for(j=i=1; i<N; i++)
  {
    if(i < j)
    {
      io       = i - 1;
      in       = j - 1;
      rtp      = Rdat[in];
      itp      = Idat[in];
      Rdat[in] = Rdat[io];
      Idat[in] = Idat[io];
      Rdat[io] = rtp;
      Idat[io] = itp;
    }

    k = nn;

    while(k < j)
    {
      j   = j - k;
      k >>= 1;
    }

    j = j + k;
  }

  if(Ft_Flag == FT_DIRECT) return true;

  rw = 1.0F / N;

  for(i=0; i<N; i++)
  {
    Rdat[i] *= rw;
    Idat[i] *= rw;
  }

  return true;
}
///////////////////////////////////////////////////////////////////////////////

void InitJoystick(void) 
{
  // Set variables asociates
  JoyPos = 0;
  MenuPos = 1;
  // Init

		 // BUTTON CENTER as input
  GPIO_InitStruct.Pin =  GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
}
unsigned char GetJoystickPosition (void)
{

  if((HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)) == GPIO_PIN_SET) return KEY_CENTER;

  return KEY_NONE;

}

void UpdateMenu(unsigned char pos)
{

  Inversion[pos-1] = 1;

  if(menu_flag==0) 
	{
		if(pos==9) { offset=3; menu_flag=1; }
    if(pos==8) { offset=2; menu_flag=1; }
    if(pos==7) { offset=1; }
    if(pos==1) { offset=0; }
  }
  else 
	{
    if(pos==9) { offset=3; }
		if(pos==8) { offset=2; }
    if(pos==7) { offset=1; }
    if(pos==1) { offset=0;  menu_flag=0; }
  }

  LCDClear();
  LCDStr ( 0, Menu[0+offset], Inversion[0+offset]);
  LCDStr ( 1, Menu[1+offset], Inversion[1+offset]);
  LCDStr ( 2, Menu[2+offset], Inversion[2+offset]);
  LCDStr ( 3, Menu[3+offset], Inversion[3+offset]);
  LCDStr ( 4, Menu[4+offset], Inversion[4+offset]);
  LCDStr ( 5, Menu[5+offset], Inversion[5+offset]);
  LCDUpdate();

  Inversion[pos-1] = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
	TIM1_Init();
  MX_SPI1_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start(&hadc2);
	DMA_Start1();
	LCDInit();
	LCDContrast(0x50);
	LCDUpdate();
	LCDClear();
  InitJoystick();
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(flag == 1)
		{
      k = 0;
            
		  while(i < ADC_BUF_SIZE)
			{
        if(i % 2 == 1)
        {
					ADC1_9[i/2] = (ADC_MEASURE[i] & 0xFFFF)*0.80586;// convert in mV (3300/4095)   sin channel
					ADC2_9[i/2] = ((ADC_MEASURE[i] & 0xFFFF0000) >> 16)*0.80586; // reference channel	
					sum1_9 += ADC1_9[i/2];
					sum2_9 += ADC2_9[i/2];
					if(i == FFT_SIZE*2 - 1)
					{
						average1_9=sum1_9/FFT_SIZE;
						average2_9=sum2_9/FFT_SIZE;
					}	
					
        }
        else
        {
					ADC1_10[k] = (ADC_MEASURE[i]&0xFFFF)*0.80586; // cos channel
					sum1_10 += ADC1_10[k];
					k++;
					if(k == FFT_SIZE)
					{
						average1_10 = sum1_10/FFT_SIZE;
					}	
        }
				i++;
				
				if(i == ADC_BUF_SIZE)
				{	
					///////////////////////////////////////////////// NEW_CHANEL(reference channel)
					for(j = 0; j < FFT_SIZE; j++)
					{
						Re[j] = ADC2_9[j] - average2_9; // without const component
						Im[j] = 0.0;									
					}

					FFT(Re, Im, FFT_SIZE, 8, FT_DIRECT); 
					
					for (j = 0; j < FFT_SIZE; j++)
					{
					  Ampl[j] = sqrt(Im[j]*Im[j]+Re[j]*Re[j])/(FFT_SIZE);
						phi[j] = atan2(Im[j],Re[j]);					
					}
					max2_9_10 = Ampl[0];
					after_max2_9_10 = Ampl[0];		
					for (j = 0; j < FFT_SIZE; j++)
					{
					  if(Ampl[j] > max2_9_10)
            {
              max2_9_10 = Ampl[j]; // search max value
							phi2_9_10 = phi[j];
							t = j;
							if(Ampl[t-1] > Ampl[t+1]) // search the second of max value
							{
								after_max2_9_10 = Ampl[t-1];
							}
							else
							{
								after_max2_9_10 = Ampl[t+1];
							}
							Ampl2_9_10 = 2*(max2_9_10 + after_max2_9_10);
						}
					}
//////////////////////////////////////////////////////// NEW_CHANEL(sin)					
					for(j = 0; j < FFT_SIZE; j++)
					{
						Re[j]=ADC1_9[j] - average1_9; // without const component
						Im[j] = 0.0;
						Ampl[j] = 0.0;
						phi[j] = 0.0;
					}
					FFT(Re, Im, FFT_SIZE, 8, FT_DIRECT); 					
					for (j = 0; j < FFT_SIZE; j++)
					{
					  Ampl[j] = sqrt(Im[j]*Im[j]+Re[j]*Re[j])/(FFT_SIZE);
						phi[j] = atan2(Im[j],Re[j]);
					}
					max1_9 =  Ampl[t];
					after_max1_9 = 0.0;	
					phi1_9 = phi[t];

					if(Ampl[t-1] > Ampl[t+1])
					{
						after_max1_9 = Ampl[t-1];
					}
					else
					{
						after_max1_9 = Ampl[t+1];
					}
					Ampl1_9 = 2*(max1_9 + after_max1_9); // Ampl for sin channel
	
//////////////////////////////////////////////////////// NEW_CHANEL(cos)
					for(j = 0; j < FFT_SIZE; j++)
					{
						Re[j] = ADC1_10[j]-average1_10; // without const component
						Im[j] = 0.0;									
					}

					FFT(Re, Im, FFT_SIZE, 8, FT_DIRECT); 
					
					for (j = 0; j < FFT_SIZE; j++)
					{
					  Ampl[j] = sqrt(Im[j]*Im[j]+Re[j]*Re[j])/(FFT_SIZE);
						phi[j] = atan2(Im[j],Re[j]);
						
					}
					max1_10 = Ampl[t];
					after_max1_10 = 0.0;
					phi1_10 = phi[t];

					if(Ampl[t-1] > Ampl[t+1])
					{
						after_max1_10 = Ampl[t-1];
					}
					else
					{
						after_max1_10 = Ampl[t+1];
					}
					Ampl1_10 = 2*(max1_10 + after_max1_10); // Ampl for cos channel
						
					phi1_9 = phi1_9*DEGREE; // convert from radian in degrees
					phi1_10 = phi1_10*DEGREE;
					phi2_9_10 = phi2_9_10*DEGREE;
					phi1_9_test = phi1_9;
					phi1_10_test = phi1_10;							
					phi1_9_test = (phi2_9_10 - phi1_9_test)/2;
					phi1_10_test = (phi2_9_10 - phi1_10_test)/2;
					
					if(phi1_9_test >= 0) // calculate angle of sin channel
					{
						phi1_9 = (phi2_9_10 - phi1_9)/2;
					}
					else
					{
						phi1_9 = (phi2_9_10 - phi1_9)/2 + 180;
					}
					
					if (phi1_9 <= 45 && phi1_9 >= 0)
					{
						phi1_9 = phi1_9*90/45 ;
					}
					else if (phi1_9 <= 90 && phi1_9 > 45)
					{
						phi1_9 = phi1_9*90/45;
					}
					else if (phi1_9 <= 135 && phi1_9 > 90)
					{
						phi1_9 = (phi1_9/135 - 2/3)*(-270);
					}
					else if (phi1_9 <= 180 && phi1_9 > 135)
					{
						phi1_9 = (phi1_9/135 - 2/3)*(-270);
					}		
					
					if(phi1_10_test >= 0) // calculate angle of cos channel
					{
						phi1_10 = (phi2_9_10 - phi1_10)/2;
					}
					else
					{
						phi1_10 = (phi2_9_10 - phi1_10)/2 + 180;
					}
					
					if (phi1_10 <= 45 && phi1_10 >= 0)
					{
						phi1_10 = phi1_10*90/45 ;
					}
					else if (phi1_10 <= 90 && phi1_10 > 45)
					{
						phi1_10 = phi1_10*90/45;
					}
					else if (phi1_10 <= 135 && phi1_10 > 90)
					{
						phi1_10 = (phi1_10/135 - 2/3)*(-270);
					}
					else if (phi1_10 <= 180 && phi1_10 > 135)
					{
						phi1_10 = (phi1_10/135 - 2/3)*(-270);
					}
					if(phi1_9 <= 0 && phi1_10 <= 0)
					{						
						phi_end = (atan2(Ampl1_9,Ampl1_10)*DEGREE);
						if (phi_end <= 45) phi_end += 315;
						else phi_end -= 45;
						phi_end_elevation = (atan2(Ampl1_9,Ampl1_10)*DEGREE);
					}
					else if(phi1_9 < 0 && phi1_10 >= 0)
					{
						phi_end = (atan2(Ampl1_9,-Ampl1_10)*DEGREE) - 45;
						phi_end_elevation = (atan2(Ampl1_9,-Ampl1_10)*DEGREE);
					}
					else if(phi1_9 > 0 && phi1_10 >= 0)
					{
						phi_end_elevation = (atan2(-Ampl1_9,-Ampl1_10)*DEGREE);
						phi_end = (atan2(-Ampl1_9,-Ampl1_10)*DEGREE);
					  phi_end += 315;
					}
					else if(phi1_9 >= 0 && phi1_10 < 0)
					{
					  phi_end = (atan2(-Ampl1_9,Ampl1_10)*DEGREE) + 315;
						phi_end_elevation = (atan2(-Ampl1_9,Ampl1_10)*DEGREE);
					}						
				}								
			}
			JoyPos = GetJoystickPosition();
			if((strncmp ((char*)buffer, (char*)str2,4) == 0) && (flag_usb == 1))
			{	
				COM_float_write(Ampl1_9);
				COM_float_write(Ampl1_10);
				COM_float_write(Ampl2_9_10);
				HAL_Delay(2);
				COM_float_write(phi1_9);
				COM_float_write(phi1_10);
				COM_float_write(phi2_9_10);
				HAL_Delay(2);
				COM_float_write(phi_end);
				COM_float_write(phi_end_elevation);					
				flag_usb = 0;

			}	
			if(MenuPos == 1 && flag_TIM1_2 > 50)
			{
				LCDContrast(0x50);

				JoyPos = GetJoystickPosition();
				if(JoyPos == KEY_CENTER && flag_TIM1 == 1 ) // checking of pressing on the button
				{
					if(counter == 3) counter = 0;
					counter++;
					flag_TIM1 = 0;
					
				}
				if(counter == 0 | counter == 1)
				{
					LCDClear();
					sprintf(trans_str2, "Ampl: %.3f", Ampl1_9);
					LCDStr ( 0, trans_str2, 0 );
					sprintf(trans_str2, "phi:  %.3f",phi1_9);
					LCDStr ( 1, trans_str2, 0 );
					sprintf(trans_str2, "Amp2: %.3f", Ampl1_10);
					LCDStr ( 2, trans_str2, 0 );
					sprintf(trans_str2, "phi:  %.3f",phi1_10);
					LCDStr ( 3, trans_str2, 0 );
					sprintf(trans_str2, "Amp3: %.3f", Ampl2_9_10);
					LCDStr ( 4, trans_str2, 0 );
					sprintf(trans_str2, "phi:  %.3f",phi2_9_10);
					LCDStr ( 5, trans_str2, 0 );
					LCDUpdate();

				}
				else if(counter == 2)
				{
				  LCDClear();
					LCDStr ( 0, "    Azimuth    ", 0 );
					sprintf(trans_str2, "phi_sc: %.3f",phi_end);
					LCDStr ( 1, trans_str2, 0 );
					sprintf(trans_str2, "Amp1: %.3f", Ampl1_9);
					LCDStr ( 2, trans_str2, 0 );
					sprintf(trans_str2, "Amp2: %.3f", Ampl1_10);
					LCDStr ( 3, trans_str2, 0 );
					LCDUpdate();
				}
				else if(counter == 3)
				{
				  LCDClear();				
					LCDStr ( 0, "   Elevation   ", 0 );
					sprintf(trans_str2, "phi_sc: %.3f",phi_end_elevation);
					LCDStr ( 1, trans_str2, 0 );
					sprintf(trans_str2, "Amp1: %.3f", Ampl1_9);
					LCDStr ( 2, trans_str2, 0 );
					sprintf(trans_str2, "Amp2: %.3f", Ampl1_10);
					LCDStr ( 3, trans_str2, 0 );
					LCDUpdate();					
				}
				flag_TIM1_2 = 0;
			}
			flag = 0;
			HAL_TIM_Base_Start(&htim3);
		}
			i = 0;
			j = 0;
			phi1_9 = 0;
			phi1_10 = 0;
			phi2_9_10 = 0;
			sum1_9 = 0;
			sum2_10 = 0;
			sum1_10 = 0;
			sum2_9 = 0;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
	
	
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
	
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;   //// Fd = 4kHz
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC7 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	

}
//////////////////////////////////////

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
