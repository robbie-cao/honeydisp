/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2017 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

#include "st_logo1.h"
#include "lcd.h"
#include "logo.h"
#include "icon.h"

#include "voc.h"
#include "sensair.h"
#include "pm25.h"
#include "hih6130.h"

#include "comm.h"

//extern FontDef_t Font_7x10;
//extern FontDef_t Font_11x18;
//extern FontDef_t Font_16x26;

/* Pin definitions */
#define LCD_RST_PORT      GPIOC
#define LCD_RST_PIN       GPIO_PIN_6
#define LCD_RST_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define LCD_RST_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

#define LCD_RST_SET			HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_SET)
#define LCD_RST_RESET			HAL_GPIO_WritePin(LCD_RST_PORT, LCD_RST_PIN, GPIO_PIN_RESET)


#define LCD_BKL_PORT      GPIOC
#define LCD_BKL_PIN       GPIO_PIN_5
#define LCD_BKL_CLK_ENABLE()           __HAL_RCC_GPIOC_CLK_ENABLE()
#define LCD_BKL_CLK_DISABLE()          __HAL_RCC_GPIOC_CLK_DISABLE()

#define LCD_BKL_SET			HAL_GPIO_WritePin(LCD_BKL_PORT, LCD_BKL_PIN, GPIO_PIN_SET)
#define LCD_BKL_RESET			HAL_GPIO_WritePin(LCD_BKL_PORT, LCD_BKL_PIN, GPIO_PIN_RESET)



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

SRAM_HandleTypeDef hsram1;

uint32_t tim3_count = 0;
uint8_t sensor_current = 0xFF;
uint8_t sensor_next = 0;

uint8_t one_byte = 'X';
uint8_t recv_comm_buf[COMM_RECV_BUF_MAX];
uint8_t send_comm_buf[256];
uint8_t recv_comm_idx;
uint8_t start_rcv_timer;
uint8_t rcv_tim_delay;
uint8_t comm_rcv_flag;

float g_humidity = 0.0, g_temperature = 0.0;
uint16_t g_co2 = 500, g_voc = 0, g_pm25 = 50;

typedef struct LCD_Screen
{
   uint8_t* cur_icon;
   union value
   {
       float temp_val;
       uint16_t other_val;
   }sensor;
   uint8_t cur_index;
};

struct LCD_Screen screen[4];



/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM3_Init(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void Test_PM25(void)
{
  PM25_StopAutoSend();
  PM25_StartMeasurement();
  HAL_Delay(100);

  while (1) {
    uint16_t pm25, pm10;
    PM25_Read(&pm25, &pm10);
    HAL_Delay(1000);
  }
}

void Test_BigFont(void)
{
  POINT_COLOR=WHITE;
  LCD_ShowString(10,40,320,32,32,"Honeywell IAQ");
  while (1) {
    POINT_COLOR=WHITE;
    LCD_ShowString(10,80,320,96,96,"0123456789");
    HAL_Delay(1000);
    POINT_COLOR=RED;
    LCD_ShowString(10,80,320,96,96,"9876543210");
    HAL_Delay(1000);
  }
}

void Test_SensorDataInAll(void)
{
  while (1) {
    float h, t;
    uint16_t co2, voc;
    char str[32];

    memset(str, 0, sizeof(str));
    Get_VocData(&co2, &voc);
    Get_HumiTemp(&h, &t);
    S8_Read(&co2);


    POINT_COLOR=WHITE;
    LCD_ShowString(10,40,320,32,32,"Honeywell IAQ");
    LCD_ShowString(10,80,320,32,32,"VOC:");
    LCD_ShowString(10,120,320,32,32,"CO2:");
    LCD_ShowString(10,160,320,32,32,"HUM:");
    LCD_ShowString(10,200,320,32,32,"TEM:");

    LCD_Fill(320,80,479,240,BLACK);
    sprintf(str, "%dppm", voc);
    LCD_ShowString(320,80,320,32,32,str);
    sprintf(str, "%dppm", co2);
    LCD_ShowString(320,120,320,32,32,str);
    sprintf(str, "%.1f%%", h);
    LCD_ShowString(320,160,320,32,32,str);
    sprintf(str, "%.1f", t);
    LCD_ShowString(320,200,320,32,32,str);

    HAL_Delay(2000);
  }

}

void Test_SensorDataOneByOne(void)
{
  while (1) {
    float h, t;
    uint16_t co2, voc;
    char str[32];

    memset(str, 0, sizeof(str));
    Get_VocData(&co2, &voc);
    Get_HumiTemp(&h, &t);
    S8_Read(&co2);


    static int ccc = 0;
    int k = 0;
    k = ccc % 4;
    ccc += 1;

    POINT_COLOR=WHITE;
    LCD_ShowString(10,40,320,32,32,"Honeywell IAQ");
    LCD_Fill(10,80,479,120,BLACK);

    switch (k) {
      case 0:
        sprintf(str, "%dppb", voc);
        LCD_ShowString(10,80,320,32,32,"VOC:");
        LCD_ShowString(320,80,320,32,32,str);
        break;
      case 1:
        sprintf(str, "%dppm", co2);
        LCD_ShowString(10,80,320,32,32,"CO2:");
        LCD_ShowString(320,80,320,32,32,str);
        break;
      case 2:
        sprintf(str, "%.1f%%", h);
        LCD_ShowString(10,80,320,32,32,"HUM:");
        LCD_ShowString(320,80,320,32,32,str);
        break;
      case 3:
        sprintf(str, "%.1f", t);
        LCD_ShowString(10,80,320,32,32,"TEM:");
        LCD_ShowString(320,80,320,32,32,str);
        break;
      default:
        break;
    }

    HAL_Delay(2000);
  }
}

void Test_Display(void)
{
  uint8_t x=0;
  uint8_t k;

  while(1)
  {
    POINT_COLOR=RED;
    LCD_Switch_Off();

    switch(x)
    {
      case 0: LCD_Clear(WHITE);  break;
      case 1: LCD_Clear(BLACK);  break;
      case 2: LCD_Clear(GBLUE);  break;
      case 3: LCD_Clear(BRED);   break;
      case 4: LCD_Clear(MAGENTA);break;
      case 5: LCD_Clear(GREEN);  break;
      case 6: LCD_Clear(CYAN);   break;
      case 7: LCD_Clear(YELLOW); break;
      case 8: LCD_Clear(BRRED);  break;
      case 9: LCD_Clear(GRAY);   break;
      case 10:LCD_Clear(LGRAY);  break;
      case 11:LCD_Clear(BROWN);  break;
    }

    x++;
    if(x==12)x=0;


    LCD_ShowString(10,40,320,32,32,"Honeywell IAQ");
    POINT_COLOR=BLUE;
    LCD_ShowString(10,80,320,24,24,"FSMC-LCD TEST");
    POINT_COLOR=WHITE;
    LCD_ShowString(10,110,320,16,16,"Simon Gu");
    POINT_COLOR=GREEN;
    LCD_ShowString(10,150,300,12,12,"2017-6-28");

    LCD_DrawBitmap(240, 160, (uint16_t *)ST_LOGO_1);

    for(k=0; k<70; k++)
    {
      POINT_COLOR=RED;
      LCD_Draw_Circle(120,240,10+k);
      POINT_COLOR=BLUE;
      LCD_Draw_Circle(360,80,10+k);
    }
    LCD_Switch_On();
    LCD_Scroll_On(LEFT);
    HAL_Delay(1000);
  }
}

void Test_LogoAndFonts(void)
{
  int k = 0;
  while (1) {
    POINT_COLOR=WHITE;
    //          LCD_Switch_Off();

    LCD_ShowImage(30, 124, 420, 72, (uint8_t*)logo);
    HAL_Delay(2000);
    LCD_Clear(BLACK);

#if 1
    POINT_COLOR=WHITE;
    for(k=0;k<4;k++)
    {
      LCD_ShowDigit(48+k*96,64,0x32+k,192,1);
    }
    HAL_Delay(2000);
    LCD_Scroll_On(LEFT);

    LCD_Clear(BLACK);
    POINT_COLOR=RED;
    for(k=0;k<4;k++)
    {
      LCD_ShowDigit(48+k*96,64,0x36+k,192,1);
    }
    HAL_Delay(2000);
    LCD_Scroll_On(LEFT);

#endif
    //              LCD_Switch_On();


    LCD_Clear(BLACK);
    LCD_ShowImage(30, 124, 420, 72, (uint8_t*)logo);
    LCD_MaskImage(30,124,420,72, BLACK);
    HAL_Delay(2000);
  }
}

void IAQ_Init(void)
{
   /* Screen[0] For temp */
    screen[0].cur_icon =(uint8_t*)icon_temp;
    screen[0].sensor.temp_val= (float)35.9;
    screen[0].cur_index = INDEX_0;

   /* Screen[1] For Humidity */
    screen[1].cur_icon =(uint8_t*)icon_hum;
    screen[1].sensor.other_val= 71;
    screen[1].cur_index = INDEX_1;

   /* Screen[2] For CO2*/
    screen[2].cur_icon =(uint8_t*)icon_co2;
    screen[2].sensor.other_val= 420;
    screen[2].cur_index = INDEX_2;

  /* Screen[3] For TVOC*/
    screen[3].cur_icon =(uint8_t*)icon_tvoc;
    screen[3].sensor.other_val= 106;
    screen[3].cur_index = INDEX_3;

  /* Screen[4] For PM25*/
    screen[4].cur_icon =(uint8_t*)icon_pm25;
    screen[4].sensor.other_val= 68;
    screen[4].cur_index = INDEX_4;

}


/* USER CODE BEGIN 5 */
/* StartDefaultTask function */
void Test_LogoFontsAndData(void)
{
  uint8_t x=0;
  uint8_t k=0;
  float curval;
  uint16_t myval;
  uint8_t bit_width;
  char buf[4]={0};

  LCD_Clear(BLACK);
  /* Infinite loop */
  for(;;)
  {
          printf("LCD Task is Running Now...\r\n");
          POINT_COLOR=WHITE;
//          LCD_Switch_Off();

          LCD_ShowImage(LOGO_XPOS, LOGO_YPOS, LOGO_WIDTH, LOGO_HEIGHT, (uint8_t*)logo);
          HAL_Delay(2000);
          LCD_Clear(BLACK);


          for(k=0;k<5;k++)
          {
            POINT_COLOR=WHITE;
            memset(buf, 0, sizeof(buf));
            LCD_ShowImage(ICON_SENSOR_XPOS, ICON_SENSOR_YPOS,
                          ICON_SENSOR_WIDTH, ICON_SENSOR_HEIGHT, (uint8_t*)screen[k].cur_icon);
            LCD_ShowSlide(screen[k].cur_index);
             if(k==0)
             {
                POINT_COLOR=RED;
                curval=screen[k].sensor.temp_val;
                sprintf(buf,"%3.1f",curval);
                if(curval<0) //Negative value
                {
                   LCD_ShowChar(DIGIT_XPOS, DIGIT_YPOS, '-', 32, 1);
                }
                else if(curval>=0 && curval<10)
                {
                   bit_width=2;
                }else if(curval>=10 && curval<100)
                {
                   bit_width=3;
                }
                LCD_ShowDigtStr(buf, 1, bit_width);
             }
             else
             {
                myval=screen[k].sensor.other_val;
                sprintf(buf, "%d", myval);
                if(myval<0) //Negative value
                {
                   LCD_ShowChar(DIGIT_XPOS, DIGIT_YPOS, '-', 32, 1);
                }
                else if(myval>=0 && myval<10)
                {
                   bit_width=2;
                }else if(myval>=10 && myval<100)
                {
                   bit_width=2;
                }
                else if(myval>=100 && myval<1000)
                {
                   bit_width=3;
                }
                LCD_ShowDigtStr(buf, 0, bit_width);

             }

             HAL_Delay(2000);
             LCD_Clear(BLACK);
          }


#if 0
                POINT_COLOR=WHITE;

                for(k=0; k<2;k++)
                {
                    LCD_ShowDigit(DOT_XPOS+k*DIGIT_WIDTH,DIGIT_YPOS,0x33+k,DIGIT_HEIGHT,1);
                }
                LCD_ShowDot();
                LCD_ShowDigit(DOT_XPOS+DIGIT_XPOS+2*DIGIT_WIDTH,DIGIT_YPOS,0x35,DIGIT_HEIGHT,1);
                LCD_ShowSlide(INDEX_0);

                LCD_ShowImage(ICON_SENSOR_XPOS, ICON_SENSOR_YPOS,
                              ICON_SENSOR_WIDTH, ICON_SENSOR_HEIGHT, (uint8_t*)icon_temp);



                 HAL_Delay(2000);
  //               LCD_Scroll_On(LEFT);
  //              HAL_Delay(2000);

                LCD_Clear(BLACK);
                POINT_COLOR=RED;

                for(k=0;k<4;k++)
                {
                   LCD_ShowDigit(DIGIT_XPOS+k*DIGIT_WIDTH,DIGIT_YPOS,0x36+k,DIGIT_HEIGHT,1);

                }
                LCD_ShowImage(ICON_SENSOR_XPOS, ICON_SENSOR_YPOS,
                              ICON_SENSOR_WIDTH, ICON_SENSOR_HEIGHT, (uint8_t*)icon_hum);
                LCD_ShowSlide(INDEX_1);

                 HAL_Delay(2000);
  //               LCD_Scroll_On(RIGHT);
  //               HAL_Delay(2000);

#endif
 //              LCD_Switch_On();


               LCD_Clear(BLACK);
               LCD_ShowImage(LOGO_XPOS, LOGO_YPOS, LOGO_WIDTH, LOGO_HEIGHT, (uint8_t*)logo);
               LCD_MaskImage(LOGO_XPOS,LOGO_YPOS,LOGO_WIDTH,LOGO_HEIGHT, BLACK);
               HAL_Delay(2000);

   }


}

void Test_SensorAutoDisp(void)
{
  while (1) {
    float h, t;
    uint16_t co2, voc;
    char str[32];

    if (sensor_current == sensor_next) {
      continue ;
    }

    memset(str, 0, sizeof(str));
    Get_VocData(&co2, &voc);
    Get_HumiTemp(&h, &t);
    S8_Read(&co2);

    POINT_COLOR=WHITE;

    switch (sensor_next) {
    case 0:
      sprintf(str, "%d", voc);
      LCD_Fill(10,80,479,80+40,BLACK);
      LCD_ShowString(10,80,320,32,32,"VOC(ppb)");
      LCD_Fill(10,80+40,479,80+40+96,BLACK);
      LCD_ShowString(10,120,320,96,96, str);

      LCD_Fill(30,80+40+100,479,80+40+100+40,BLACK);
      LCD_Draw_Circle(50,80+40+100+20,15);
      LCD_Draw_Circle(90,80+40+100+20,10);
      LCD_Draw_Circle(130,80+40+100+20,10);
      LCD_Draw_Circle(170,80+40+100+20,10);
      break;
    case 1:
      sprintf(str, "%d", co2);
      LCD_Fill(10,80,479,80+40,BLACK);
      LCD_ShowString(10,80,320,32,32,"CO2(ppm)");
      LCD_Fill(10,80+40,479,80+40+96,BLACK);
      LCD_ShowString(10,120,320,96,96,str);

      LCD_Fill(30,80+40+100,479,80+40+100+40,BLACK);
      LCD_Draw_Circle(50,80+40+100+20,10);
      LCD_Draw_Circle(90,80+40+100+20,15);
      LCD_Draw_Circle(130,80+40+100+20,10);
      LCD_Draw_Circle(170,80+40+100+20,10);
      break;
    case 2:
      sprintf(str, "%d", (int)h);
      LCD_Fill(10,80,479,80+40,BLACK);
      LCD_ShowString(10,80,320,32,32,"Humidity(%)");
      LCD_Fill(10,80+40,479,80+40+96,BLACK);
      LCD_ShowString(10,120,320,96,96,str);

      LCD_Fill(30,80+40+100,479,80+40+100+40,BLACK);
      LCD_Draw_Circle(50,80+40+100+20,10);
      LCD_Draw_Circle(90,80+40+100+20,10);
      LCD_Draw_Circle(130,80+40+100+20,15);
      LCD_Draw_Circle(170,80+40+100+20,10);
      break;
    case 3:
      sprintf(str, "%d", (int)t);
      LCD_Fill(10,80,479,80+40,BLACK);
      LCD_ShowString(10,80,320,32,32,"Temperature(C)");
      LCD_Fill(10,80+40,479,80+40+96,BLACK);
      LCD_ShowString(10,120,320,96,96,str);

      LCD_Fill(30,80+40+100,479,80+40+100+40,BLACK);
      LCD_Draw_Circle(50,80+40+100+20,10);
      LCD_Draw_Circle(90,80+40+100+20,10);
      LCD_Draw_Circle(130,80+40+100+20,10);
      LCD_Draw_Circle(170,80+40+100+20,15);
      break;
    default:
      break;
    }
    sensor_current = sensor_next;
  }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_FMC_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();

  /* Force reset */
  LCD_RST_RESET;
  LCD_Delay(20000);
  LCD_RST_SET;

  /* Delay for RST response */
  LCD_Delay(20000);

  /* Software reset */
  LCD_WR_REG(SOFT_RESET);
  LCD_Delay(50000);

  LCD_Init();
  LCD_BKL_SET;

  /* USER CODE BEGIN 2 */

  printf("HON Connected Air Stat...\r\n");

  LCD_Clear(BLACK);
  POINT_COLOR=WHITE;

  /*##-2- Start the TIM Base generation in interrupt mode ####################*/
  /* Start Channel1 */
  if(HAL_TIM_Base_Start_IT(&htim3) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }

  /* TEST CODE BEGIN */
  printf("Starting...\r\n");
  HAL_UART_Receive_IT(&huart3, &one_byte, 1);

  Comm_Init();
  while (0) {
    if(comm_rcv_flag)
    {
      Comm_Process();
      comm_rcv_flag = 0;
      Comm_Response();
    }
    if (sensor_current == sensor_next) {
      continue ;
    }
    uint16_t tmp;
    Get_VocData(&tmp, &g_voc);
    Get_HumiTemp(&g_humidity, &g_temperature);
    sensor_current = sensor_next;
  }

#if 0
  Test_Display();
#endif

#if 0
  Test_LogoAndFonts();
#endif

#if 1
  IAQ_Init();
  Test_LogoFontsAndData();
#endif

#if 0
  Test_PM25();
#endif

#if 0
  Test_BigFont();
#endif

#if 0
  Test_SensorDataInAll();
#endif

#if 0
  Test_SensorDataOneByOne();
#endif

#if 1
  Test_SensorAutoDisp();
#endif

  /* TEST CODE END */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
  }
  /* USER CODE END WHILE */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
  /*##-1- Configure the TIM peripheral #######################################*/
  /* -----------------------------------------------------------------------
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1),
    since APB1 prescaler is different from 1.
      TIM3CLK = 2 * PCLK1
      PCLK1 = HCLK / 4
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
    To get TIM3 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    Prescaler = ((SystemCoreClock /2) /10 KHz) - 1

    Note:
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to update SystemCoreClock
     variable value. Otherwise, any configuration based on this variable will be incorrect.
     This variable is updated in three ways:
      1) by calling CMSIS function SystemCoreClockUpdate()
      2) by calling HAL API function HAL_RCC_GetSysClockFreq()
      3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
  ----------------------------------------------------------------------- */

  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uint32_t uwPrescalerValue = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;

//  TIM_ClockConfigTypeDef sClockSourceConfig;
//  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Period = 10000 - 1;
  htim3.Init.Prescaler = uwPrescalerValue;
  htim3.Init.ClockDivision = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /* Initialize recv_comm_buf */
    for(int i=0;i<COMM_RECV_BUF_MAX;i++)
    {
        recv_comm_buf[i] = 0;
    }
    recv_comm_idx = 0;
    start_rcv_timer = 0;
    rcv_tim_delay = 0;
    comm_rcv_flag = 0;
}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing;
  FMC_NORSRAM_TimingTypeDef ExtTiming;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_ENABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 70;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 3; //was 17;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */
  ExtTiming.AddressSetupTime = 15;
  ExtTiming.AddressHoldTime = 15;
  ExtTiming.DataSetupTime = 70;
  ExtTiming.BusTurnAroundDuration = 15;
  ExtTiming.CLKDivision = 16;
  ExtTiming.DataLatency = 3; //was 17;
  ExtTiming.AccessMode = FMC_ACCESS_MODE_A;

  if (HAL_SRAM_Init(&hsram1, &Timing, &ExtTiming) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Pinout Configuration
*/
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();


  /* Configure the LCD RST pin */
  GPIO_InitStruct.Pin = LCD_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(LCD_RST_PORT, &GPIO_InitStruct);

  /* Configure the LCD BackLight pin */
  GPIO_InitStruct.Pin = LCD_BKL_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;

  HAL_GPIO_Init(LCD_BKL_PORT, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  printf(".");
  tim3_count++;
  if (tim3_count >= 3) {
    sensor_next += 1;
    if (sensor_next >= 4) {
      sensor_next = 0;
    }
    tim3_count = 0;
  }
}

/**
  * @brief  Tx Transfer completed callback
  * @param  UartHandle: UART handle.
  * @note   This example shows a simple way to report end of IT Tx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/

}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete*/
//  printf("%c", one_byte);
  if(!comm_rcv_flag)
  {
    recv_comm_buf[recv_comm_idx++] = one_byte;
    if(recv_comm_idx == COMM_RECV_BUF_MAX)
    {
      recv_comm_idx = 0;
    }
  }
  start_rcv_timer = 1;
  rcv_tim_delay = 0;
  HAL_UART_Receive_IT(&huart3, &one_byte, 1);

}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
 void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  /* Turn LED3 on: Transfer error in reception/transmission process */
  //BSP_LED_On(LED3);
}

void Timer_1MS_ISR(void)
{
  /* UART end of receive check */
  if(start_rcv_timer)
  {
    rcv_tim_delay++;
    if(rcv_tim_delay >= REC_TIM_DELAY)
    {
      start_rcv_timer = 0;
      comm_rcv_flag = 1;
    }
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
