/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : M2006/C610 + PS2 + stepper + dual servo control
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
static uint8_t  prev_btn1 = 0xFF;       /* PS2 buttons: 0=pressed */
static uint8_t  prev_btn2 = 0xFF;
static float    balance = 1.07f;        /* left motor scale */
static int16_t  sm_left  = 0;
static int16_t  sm_right = 0;
static uint32_t ps2_ok_cnt = 0;
static uint32_t ps2_fail_cnt = 0;

static uint8_t  cal_ok = 0;
static uint8_t  cly, crx;

/* Nathan-style PS2 reconnect */
static uint8_t  ps2_connected = 1;
static uint32_t ps2_reinit_tick = 0;

/* stepper acceleration */
static uint16_t stepper_period = 8999;
/* USER CODE END PV */
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

  MX_MOTO_GPIO_Init();
  MX_TIM3_Init();
  MX_SERVO_GPIO_Init();
  MX_TIM4_Init();
  ps2_init();
  /* USER CODE END 2 */

  while (1)
  {
    if(HAL_GetTick() - led1_tick > 1000)
    {
      led1_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    if(HAL_GetTick() - motor_tick >= 10)
    {
      motor_tick = HAL_GetTick();

      if (ps2_read(&ps2)) {
        ps2_connected = 1;
        ps2_ok_cnt++;

        if (!cal_ok) {
          cal_ok = 1;
          cly = ps2.joy_ly;
          crx = ps2.joy_rx;
        }

        /* ---- speed: L1=×0.75, R1=×1.33 (edge-triggered, 0=pressed) ---- */
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

        /* ---- dead zone ---- */
        int16_t ly = (int16_t)ps2.joy_ly - cly;
        int16_t rx = (int16_t)ps2.joy_rx - crx;
        if (ly > -35 && ly < 35) ly = 0;
        if (rx > -35 && rx < 35) rx = 0;

        /* ---- differential drive: L-stick Y=fwd, R-stick X=turn ---- */
        int16_t forward = (int16_t)(-ly * 10 * speed_scale);
        int16_t turn    = (int16_t)( rx * 8  * speed_scale);
        int16_t tgt_left  = forward + turn;
        int16_t tgt_right = forward - turn;

        if (tgt_left  >  4000) tgt_left  =  4000;
        if (tgt_left  < -4000) tgt_left  = -4000;
        if (tgt_right >  4000) tgt_right =  4000;
        if (tgt_right < -4000) tgt_right = -4000;

        if (sm_left < tgt_left) {
          sm_left += 200;
          if (sm_left > tgt_left) sm_left = tgt_left;
        } else if (sm_left > tgt_left) {
          sm_left -= 200;
          if (sm_left < tgt_left) sm_left = tgt_left;
        }
        if (sm_right < tgt_right) {
          sm_right += 200;
          if (sm_right > tgt_right) sm_right = tgt_right;
        } else if (sm_right > tgt_right) {
          sm_right -= 200;
          if (sm_right < tgt_right) sm_right = tgt_right;
        }

        set_moto_current(&hcan1, (int16_t)(sm_left * balance), sm_left,
                         -sm_right, sm_right);

        /* ---- stepper: D-pad UP/DOWN with acceleration ramp ---- */
        if (!(ps2.btn1 & PS2_UP)) {
          GPIOB->BSRR = MOTO_DIR_Pin;
          if (!(htim3.Instance->CR1 & TIM_CR1_CEN)) {
            stepper_period = 8999;
            htim3.Instance->ARR = stepper_period;
            HAL_TIM_Base_Start_IT(&htim3);
          }
        } else if (!(ps2.btn1 & PS2_DOWN)) {
          GPIOB->BSRR = (uint32_t)MOTO_DIR_Pin << 16;
          if (!(htim3.Instance->CR1 & TIM_CR1_CEN)) {
            stepper_period = 8999;
            htim3.Instance->ARR = stepper_period;
            HAL_TIM_Base_Start_IT(&htim3);
          }
        } else {
          HAL_TIM_Base_Stop_IT(&htim3);
          stepper_period = 8999;
        }

        if ((htim3.Instance->CR1 & TIM_CR1_CEN) && stepper_period > 2249) {
          stepper_period -= 100;
          if (stepper_period < 2249) stepper_period = 2249;
          htim3.Instance->ARR = stepper_period;
        }

        /*
         * 360° continuous-rotation servos: 1500=STOP, 500=max CW, 2500=max CCW
         * Hold=fast spin-up, Release=ultra-slow coast to stop
         */
        #define S_GO  8   /* spin-up speed */
        #define S_STOP 6   /* coast-to-stop */

        /* MG996R (PC0, 360°): D-pad LEFT/RIGHT, release=coast to stop */
        if (!(ps2.btn1 & PS2_LEFT)) {
          if (servo0_pulse > 500) servo0_pulse -= S_GO;
        } else if (!(ps2.btn1 & PS2_RIGHT)) {
          if (servo0_pulse < 2500) servo0_pulse += S_GO;
        } else {
          if (servo0_pulse < 1500) servo0_pulse += S_STOP;
          else if (servo0_pulse > 1500) servo0_pulse -= S_STOP;
        }

        /* MG996R #2 (PC1, 360°): L2/R2, release=coast to stop */
        if (!(ps2.btn2 & PS2_L2)) {
          if (servo1_pulse > 500) servo1_pulse -= S_GO;
        } else if (!(ps2.btn2 & PS2_R2)) {
          if (servo1_pulse < 2500) servo1_pulse += S_GO;
        } else {
          if (servo1_pulse < 1500) servo1_pulse += S_STOP;
          else if (servo1_pulse > 1500) servo1_pulse -= S_STOP;
        }
      } else {
        ps2_fail_cnt++;
        if (ps2_connected) {
          ps2_connected = 0;
          sm_left  = 0;
          sm_right = 0;
          set_moto_current(&hcan1, 0, 0, 0, 0);
          HAL_TIM_Base_Stop_IT(&htim3);
          stepper_period = 8999;
        }
        if (HAL_GetTick() - ps2_reinit_tick > 200) {
          ps2_reinit_tick = HAL_GetTick();
          ps2_init();
        }
      }
    }
  }
}

/** System Clock Configuration */
void SystemClock_Config(void)
{
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
    _Error_Handler(__FILE__, __LINE__);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    _Error_Handler(__FILE__, __LINE__);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ---- timer callbacks ---- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  if (htim->Instance == TIM3) {
    pul_state = !pul_state;
    if (pul_state)
      GPIOB->BSRR = GPIO_PIN_0;
    else
      GPIOB->BSRR = (uint32_t)GPIO_PIN_0 << 16;
    if (++step_cnt >= 24000) {
      step_cnt = 0;
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
  }
  if (htim->Instance == TIM4) {
    /* start of 20ms servo frame: both pins HIGH */
    GPIOC->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    /* update OC compare values for this frame */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, servo0_pulse);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, servo1_pulse);
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* end of servo pulse: pull the matching pin LOW */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    GPIOC->BSRR = (uint32_t)GPIO_PIN_0 << 16;  /* PC0 LOW */
  else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    GPIOC->BSRR = (uint32_t)GPIO_PIN_1 << 16;  /* PC1 LOW */
}

void _Error_Handler(char * file, int line)
{
  while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
