/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : M2006/C610 motor control via CAN - auto start low speed
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "ps2.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t TIM_COUNT[2];
static uint32_t led1_tick = 0;
static uint32_t motor_tick = 0;
static uint32_t step_cnt = 0;
static uint8_t  pul_state = 0;
static ps2_state_t ps2;
static float    speed_scale = 1.0f;
static uint8_t  prev_btn2 = 0xFF;      /* for edge detection */
static uint8_t  prev_btn1 = 0xFF;
static float    balance = 1.07f;        /* left motor scale (left faster → <1.0) */
static int16_t  sm_left  = 0;          /* smoothed motor current */
static int16_t  sm_right = 0;
static uint8_t  cal_ok = 0;            /* center calibrated flag */
static uint8_t  clx, cly, crx, cry;    /* calibrated centers */
static uint32_t ps2_ok_cnt = 0;        /* successful reads */
static uint32_t ps2_fail_cnt = 0;      /* failed reads */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  my_can_filter_init_recv_all(&hcan1);
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);


  /* Stepper motor init */
  MX_MOTO_GPIO_Init();

  MX_TIM3_Init();
  /* TIM3 started/stopped by PS2 UP/DOWN buttons */

  /* PS2 controller init */
  ps2_init();

  /*
   * One-shot: change motor ID from 1 to 2.
   * Disconnect one motor, flash, power-cycle, then comment this line out.
   */
 // c610_set_motor_id(&hcan1, 2);
  /* USER CODE END 2 */

  while (1)
  {
    /* LED1 slow blink = loop running */
    if(HAL_GetTick() - led1_tick > 1000)
    {
      led1_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    /* PS2 control: read & drive 2 M2006 motors every 10ms */
    if(HAL_GetTick() - motor_tick >= 10)
    {
      motor_tick = HAL_GetTick();

      if (ps2_read(&ps2)) {
        ps2_ok_cnt++;
        /* --- auto-calibrate center on first valid frame --- */
        if (!cal_ok) {
          cal_ok = 1;
          clx = ps2.joy_lx;
          cly = ps2.joy_ly;
          crx = ps2.joy_rx;
          cry = ps2.joy_ry;
        }

        /* --- speed adjust buttons (edge-triggered) --- */
        uint8_t edge1 = ~ps2.btn1 & prev_btn1;
        uint8_t edge2 = ~ps2.btn2 & prev_btn2;
        if (edge2 & PS2_L1) {
          speed_scale *= 0.75f;
          if (speed_scale < 0.1f) speed_scale = 0.1f;
        }
        if (edge2 & PS2_R1) {
          speed_scale *= 1.333f;
          if (speed_scale > 3.0f) speed_scale = 3.0f;
        }
        prev_btn1 = ps2.btn1;
        prev_btn2 = ps2.btn2;

        /* --- dead zone around calibrated center --- */
        int16_t ly = (int16_t)ps2.joy_ly - cly;
        int16_t rx = (int16_t)ps2.joy_rx - crx;
        if (ly > -35 && ly < 35) ly = 0;
        if (rx > -35 && rx < 35) rx = 0;

        /* --- differential drive mixing --- */
        int16_t forward = (int16_t)(-ly * 10 * speed_scale);
        int16_t turn    = (int16_t)( rx * 8 * speed_scale);
        int16_t tgt_left  = forward + turn;
        int16_t tgt_right = forward - turn;
			
    

        if (tgt_left  >  4000) tgt_left  =  2500;
        if (tgt_left  < -4000) tgt_left  = -2500;
        if (tgt_right >  4000) tgt_right =  2500;
        if (tgt_right < -4000) tgt_right = -2500;

        /* --- acceleration ramp: move smoothed value toward target --- */
        #define RAMP_STEP  400   /* current change per 10ms cycle */
        if (sm_left < tgt_left) {
          sm_left += RAMP_STEP;
          if (sm_left > tgt_left) sm_left = tgt_left;
        } else if (sm_left > tgt_left) {
          sm_left -= RAMP_STEP;
          if (sm_left < tgt_left) sm_left = tgt_left;
        }
        if (sm_right < tgt_right) {
          sm_right += RAMP_STEP;
          if (sm_right > tgt_right) sm_right = tgt_right;
        } else if (sm_right > tgt_right) {
          sm_right -= RAMP_STEP;
          if (sm_right < tgt_right) sm_right = tgt_right;
        }

        /* M1=left (balance scaled), M3=right (inverted). M2/M4 unused */
        set_moto_current(&hcan1, (int16_t)(sm_left * balance), sm_left,
                         -sm_right, sm_right);

        /* --- Stepper motor: PS2 D-pad UP/DOWN --- */
        if (!(ps2.btn1 & PS2_UP)) {
          GPIOB->BSRR = MOTO_DIR_Pin;
          HAL_TIM_Base_Start_IT(&htim3);
        } else if (!(ps2.btn1 & PS2_DOWN)) {
          GPIOB->BSRR = (uint32_t)MOTO_DIR_Pin << 16;
          HAL_TIM_Base_Start_IT(&htim3);
        } else {
          HAL_TIM_Base_Stop_IT(&htim3);
        }
      } else {
        /* controller disconnected -> stop all motors immediately */
        ps2_fail_cnt++;
        sm_left  = 0;
        sm_right = 0;
        set_moto_current(&hcan1, 0, 0, 0, 0);
        HAL_TIM_Base_Stop_IT(&htim3);
      }
    }
  }
}

/** System Clock Configuration */
void SystemClock_Config(void)
{
  /* Enable FPU (must be done before any float operation) */
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  if (htim->Instance == TIM3) {
    pul_state = !pul_state;
    if (pul_state)
      GPIOB->BSRR = GPIO_PIN_0;                  // PB0 HIGH
    else
      GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;  // PB0 LOW
    if (++step_cnt >= 8000) { // LED1每2秒闪一次
      step_cnt = 0;
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
  }
}

void _Error_Handler(char * file, int line)
{
  while(1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
