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
static int8_t   stepper_dir = 0;  /* -1=DOWN, 0=STOP, 1=UP */
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
  MX_EMAG_GPIO_Init();
  MX_TIM3_Init();
  MX_SERVO_GPIO_Init();
  MX_TIM4_Init();
  ps2_init();
  /* USER CODE END 2 */

  static uint32_t emag_test_tick = 0;
  static uint8_t  emag_test_state = 0;

  while (1)
  {
    if(HAL_GetTick() - led1_tick > 1000)
    {
      led1_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }

    /* ---- 测试: 每2秒切换电磁铁 (验证GPIO和硬件是否正常) ---- */
    if(HAL_GetTick() - emag_test_tick > 2000)
    {
      emag_test_tick = HAL_GetTick();
      emag_test_state = !emag_test_state;
      if (emag_test_state) {
        GPIOD->BSRR = EMAG1_IN1_Pin | EMAG2_IN3_Pin
                    | ((uint32_t)(EMAG1_IN2_Pin | EMAG2_IN4_Pin) << 16);
      } else {
        GPIOD->BSRR = (uint32_t)(EMAG1_IN1_Pin | EMAG1_IN2_Pin
                               | EMAG2_IN3_Pin | EMAG2_IN4_Pin) << 16;
      }
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
        int16_t forward = (int16_t)(-ly * 15 * speed_scale);
        int16_t turn    = (int16_t)( rx * 12 * speed_scale);
        int16_t tgt_left  = forward + turn;
        int16_t tgt_right = forward - turn;

        /* ---- slope boost: if motors are loaded (high current, low speed)
           while user commands movement, auto-ramp extra torque ---- */
        {
          static int16_t boost_l = 0, boost_r = 0;
          int16_t spd_l = (moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm) / 2;
          int16_t cur_l = (moto_chassis[0].real_current + moto_chassis[1].real_current) / 2;
          int16_t spd_r = (moto_chassis[2].speed_rpm + moto_chassis[3].speed_rpm) / 2;
          int16_t cur_r = (moto_chassis[2].real_current + moto_chassis[3].real_current) / 2;

          /* left side: climbing when target > 300, speed < 500rpm, current > 1A */
          if (abs(tgt_left) > 300 && abs(spd_l) < 500 && abs(cur_l) > 1000) {
            if (boost_l < 5000) boost_l += 150;
          } else {
            if (boost_l > 0) boost_l -= 100;
          }

          /* right side */
          if (abs(tgt_right) > 300 && abs(spd_r) < 500 && abs(cur_r) > 1000) {
            if (boost_r < 5000) boost_r += 150;
          } else {
            if (boost_r > 0) boost_r -= 100;
          }

          if (tgt_left  > 0) tgt_left  += boost_l;
          else              tgt_left  -= boost_l;
          if (tgt_right > 0) tgt_right += boost_r;
          else              tgt_right -= boost_r;
        }

        if (tgt_left  >  8000) tgt_left  =  8000;
        if (tgt_left  < -8000) tgt_left  = -8000;
        if (tgt_right >  8000) tgt_right =  8000;
        if (tgt_right < -8000) tgt_right = -8000;

        if (sm_left < tgt_left) {
          sm_left += 500;
          if (sm_left > tgt_left) sm_left = tgt_left;
        } else if (sm_left > tgt_left) {
          sm_left -= 500;
          if (sm_left < tgt_left) sm_left = tgt_left;
        }
        if (sm_right < tgt_right) {
          sm_right += 500;
          if (sm_right > tgt_right) sm_right = tgt_right;
        } else if (sm_right > tgt_right) {
          sm_right -= 500;
          if (sm_right < tgt_right) sm_right = tgt_right;
        }

        set_moto_current(&hcan1, (int16_t)(sm_left * balance), sm_left,
                         -sm_right, sm_right);

        /* ---- stepper: D-pad UP/DOWN with acceleration ramp ---- */
        {
          int8_t desired = 0;
          if (!(ps2.btn1 & PS2_UP))
            desired = 1;
          else if (!(ps2.btn1 & PS2_DOWN))
            desired = -1;

          if (desired != stepper_dir) {
            /* direction change or start/stop — full reset before switch */
            HAL_TIM_Base_Stop_IT(&htim3);
            htim3.Instance->CNT = 0;
            stepper_period = 8999;
            htim3.Instance->ARR = stepper_period;
            pul_state = 0;
            GPIOB->BSRR = (uint32_t)MOTO_PUL_Pin << 16;  /* PUL LOW */

            if (desired == 1) {
              GPIOB->BSRR = MOTO_DIR_Pin;              /* DIR HIGH */
              HAL_TIM_Base_Start_IT(&htim3);
            } else if (desired == -1) {
              GPIOB->BSRR = (uint32_t)MOTO_DIR_Pin << 16; /* DIR LOW */
              HAL_TIM_Base_Start_IT(&htim3);
            }
            stepper_dir = desired;
          }

          /* acceleration ramp — safe ARR update via ARPE */
          if (stepper_dir != 0 && stepper_period > 1687) {
            stepper_period -= 133;
            if (stepper_period < 1687) stepper_period = 1687;
            htim3.Instance->ARR = stepper_period;
          }
        }

        /*
         * Both servos: 360° continuous-rotation, instant stop on release
         *   1500us=STOP,  500us=max CW,  2500us=max CCW
         * Servo0 (PC0): D-pad LEFT/RIGHT
         * Servo1 (PC1): L2/R2
         */
        #define S_360_STEP 5

        /* ---- servo0: 360°, instant stop ---- */
        if (!(ps2.btn1 & PS2_LEFT)) {
          if (servo0_pulse > 500) servo0_pulse -= S_360_STEP;
        } else if (!(ps2.btn1 & PS2_RIGHT)) {
          if (servo0_pulse < 2500) servo0_pulse += S_360_STEP;
        } else {
          servo0_pulse = 1500;
        }

        /* ---- servo1: 360°, instant stop ---- */
        if (!(ps2.btn2 & PS2_L2)) {
          if (servo1_pulse > 500) servo1_pulse -= S_360_STEP;
        } else if (!(ps2.btn2 & PS2_R2)) {
          if (servo1_pulse < 2500) servo1_pulse += S_360_STEP;
        } else {
          servo1_pulse = 1500;
        }

        /* ---- electromagnets: CIRCLE = both ON, release = both OFF ---- */
        if (!(ps2.btn2 & PS2_CIR)) {
          GPIOD->BSRR = EMAG1_IN1_Pin | EMAG2_IN3_Pin
                      | ((uint32_t)(EMAG1_IN2_Pin | EMAG2_IN4_Pin) << 16);
        } else {
          GPIOD->BSRR = (uint32_t)(EMAG1_IN1_Pin | EMAG1_IN2_Pin
                                 | EMAG2_IN3_Pin | EMAG2_IN4_Pin) << 16;
        }

        /* PS2 bit-bang disables IRQs for ~900us. If TIM4 interrupts were
           delayed, the PWM pin state may be wrong. Fix both cases:
           - pin HIGH when CNT already passed CCR (OC match missed)
           - pin LOW  when CNT still before CCR (period start missed)
           Either way: clear flags + restart clean frame */
        {
          uint32_t cnt = TIM4->CNT;
          uint32_t ccr1 = TIM4->CCR1;
          uint32_t odr  = GPIOC->ODR;
          int bad = 0;

          if (cnt < ccr1) {
            if (!(odr & GPIO_PIN_0)) bad = 1;  /* should be HIGH but is LOW */
          } else {
            if (odr & GPIO_PIN_0) bad = 1;     /* should be LOW but is HIGH */
          }
          if (!bad) {
            uint32_t ccr2 = TIM4->CCR2;
            if (cnt < ccr2) {
              if (!(odr & GPIO_PIN_1)) bad = 1;
            } else {
              if (odr & GPIO_PIN_1) bad = 1;
            }
          }

          if (bad) {
            TIM4->SR    = ~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF);
            TIM4->CNT   = 0;
            TIM4->CCR1  = servo0_pulse;
            TIM4->CCR2  = servo1_pulse;
            GPIOC->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
          }
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
          stepper_dir = 0;
          pul_state = 0;
          GPIOB->BSRR = (uint32_t)MOTO_PUL_Pin << 16;
          GPIOD->BSRR = (uint32_t)(EMAG1_IN1_Pin | EMAG1_IN2_Pin
                                 | EMAG2_IN3_Pin | EMAG2_IN4_Pin) << 16;
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 1, 0);
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
    /* update compare values BEFORE raising pins, so the first
       compare match after CNT rolls to 0 uses the latest pulse width */
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, servo0_pulse);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, servo1_pulse);
    GPIOC->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
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
