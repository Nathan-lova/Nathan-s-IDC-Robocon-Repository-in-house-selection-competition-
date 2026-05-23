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
#include "pid.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

volatile uint16_t dbg_servo180_target = 0;
volatile uint16_t dbg_servo1_pulse = 0;
/* USER CODE BEGIN PV */
#define JOY_DEAD      15
#define JOY_MAX       127.0f
#define WHEEL_MAX_RPM 4000.0f
#define TURN_MAX_RPM  3000.0f
#define SERVO360_STOP_US  1500u
#define SERVO360_SPEED_US 150u

PID_TypeDef motor_pid[2];

/* ================= Real PS2 key mapping ================= */

typedef struct
{
    uint8_t bank;   // 1 = btn1, 2 = btn2
    uint8_t mask;
} ps2_key_t;

#define PS2_BANK_BTN1  1
#define PS2_BANK_BTN2  2

#define K_BTN1(mask)   {PS2_BANK_BTN1, (mask)}
#define K_BTN2(mask)   {PS2_BANK_BTN2, (mask)}


/* 你们实测出来的物理按键映射 */
static ps2_key_t KEY_PHY_UP       = K_BTN1(0x08);
static ps2_key_t KEY_PHY_RIGHT    = K_BTN1(0x10);
static ps2_key_t KEY_PHY_DOWN     = K_BTN1(0x20);
static ps2_key_t KEY_PHY_LEFT     = K_BTN1(0x40);

static ps2_key_t KEY_PHY_L2       = K_BTN1(0x80);  /* confirmed: btn1 255→127 */
static ps2_key_t KEY_PHY_R2       = K_BTN2(0x01);  /* confirmed: btn2 127→126 */
static ps2_key_t KEY_PHY_L1       = K_BTN2(0x02);
static ps2_key_t KEY_PHY_R1       = K_BTN2(0x04);


uint16_t TIM_COUNT[2];
static uint32_t led1_tick = 0;
static uint32_t motor_tick = 0;
static uint32_t step_cnt = 0;
static uint8_t  pul_state = 0;
static int8_t   stepper_dir = 0;  /* -1=DOWN, 0=STOP, 1=UP */
static ps2_state_t ps2;
static float    speed_scale = 1.0f;
static uint8_t  prev_btn2 = 0xFF;       /* edge detection for L1/R1 */
static uint32_t ps2_ok_cnt = 0;
static uint32_t ps2_fail_cnt = 0;

static uint8_t  cal_ok = 0;
static uint8_t  cly, crx;
static uint8_t  relay_on;
static uint8_t  prev_circle_bit = 1;   /* btn2 bit5 idle=1 (not pressed) */
static uint8_t  debounce_left;
static uint8_t  debounce_right;
static uint8_t  debounce_l2;
static uint8_t  debounce_r2;
static uint8_t  debounce_up;
static uint8_t  debounce_down;

/* Nathan-style PS2 reconnect */
static uint8_t  ps2_connected = 1;
static uint32_t ps2_reinit_tick = 0;

/* stepper acceleration */
static uint16_t stepper_period = 10499;  /* ~2000 steps/s start */
/* USER CODE END PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */
static uint8_t ps2_key_pressed(ps2_key_t key)
{
    uint8_t pressed;

    if (key.bank == PS2_BANK_BTN1)
    {
        pressed = (uint8_t)(~ps2.btn1);
    }
    else
    {
        /*
         * 你们的手柄 btn2 在不按任何键时也有 0x80。
         * 所以必须屏蔽 0x80，否则程序会误以为某个键一直被按下。
         */
        pressed = (uint8_t)(~ps2.btn2) & 0x7F;
    }

    return (pressed & key.mask) != 0;
}
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
  servo_drv_init();
  ps2_init();

  /* PID speed control init */
  pid_init(&motor_pid[0]);
  pid_init(&motor_pid[1]);
  motor_pid[0].f_param_init(&motor_pid[0], PID_Speed, 8000, 3000, 20, 10, 500, 0, 2.5f, 0.1f, 0.5f);
  motor_pid[1].f_param_init(&motor_pid[1], PID_Speed, 8000, 3000, 20, 10, 500, 0, 2.5f, 0.1f, 0.5f);
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

    /* 继电器测试代码已注释，用ST-Link看relay_on变量确认按键检测 */

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
				{
						uint8_t btn2_effective;
						uint8_t edge2;

						btn2_effective = ((uint8_t)(~ps2.btn2)) & 0x7F;
						edge2 = btn2_effective & (uint8_t)(~prev_btn2);

						if (edge2 & KEY_PHY_R1.mask)
						{
								speed_scale *= 1.25f;  /* R1: speed up */
								if (speed_scale > 2.0f)
										speed_scale = 2.0f;
						}

						if (edge2 & KEY_PHY_L1.mask)
						{
								speed_scale *= 0.8f;   /* L1: slow down */
								if (speed_scale < 0.3f)
										speed_scale = 0.3f;
						}

						prev_btn2 = btn2_effective;
				}

        /* ---- joystick normalize ---- */
        int16_t ly = (int16_t)ps2.joy_ly - cly;
        int16_t rx = (int16_t)ps2.joy_rx - crx;

        if (ly > -JOY_DEAD && ly < JOY_DEAD) ly = 0;
        if (rx > -JOY_DEAD && rx < JOY_DEAD) rx = 0;

        float fwd = 0.0f, trn = 0.0f;
        if (ly != 0) { fwd = -(float)ly / (JOY_MAX - JOY_DEAD); if (fwd >  1.0f) fwd =  1.0f; if (fwd < -1.0f) fwd = -1.0f; }
        if (rx != 0) { trn =  (float)rx / (JOY_MAX - JOY_DEAD); if (trn >  1.0f) trn =  1.0f; if (trn < -1.0f) trn = -1.0f; }

        float fwd_rpm = fwd * WHEEL_MAX_RPM * speed_scale;
        float trn_rpm = trn * TURN_MAX_RPM  * speed_scale;
        float t_l = fwd_rpm + trn_rpm, t_r = fwd_rpm - trn_rpm;
        if (t_l >  WHEEL_MAX_RPM) t_l =  WHEEL_MAX_RPM; if (t_l < -WHEEL_MAX_RPM) t_l = -WHEEL_MAX_RPM;
        if (t_r >  WHEEL_MAX_RPM) t_r =  WHEEL_MAX_RPM; if (t_r < -WHEEL_MAX_RPM) t_r = -WHEEL_MAX_RPM;
        motor_pid[0].target = t_l;
        motor_pid[1].target = t_r;

        /* ---- PID speed control ---- */
        int16_t cur_l = (int16_t)motor_pid[0].f_cal_pid(&motor_pid[0],
                                         moto_chassis[0].speed_rpm);
        int16_t cur_r = (int16_t)motor_pid[1].f_cal_pid(&motor_pid[1],
                                         -moto_chassis[1].speed_rpm);

        if (cur_l >  8000) cur_l =  8000;
        if (cur_l < -8000) cur_l = -8000;
        if (cur_r >  8000) cur_r =  8000;
        if (cur_r < -8000) cur_r = -8000;

        set_moto_current(&hcan1, cur_l, -cur_r);

        /* ---- stepper: D-pad UP/DOWN (3-frame debounce) ---- */
        {
          uint8_t up_now   = ps2_key_pressed(KEY_PHY_UP);
          uint8_t down_now = ps2_key_pressed(KEY_PHY_DOWN);

          if (up_now   && debounce_up   < 5) debounce_up++;
          else if (!up_now)                  debounce_up = 0;
          if (down_now && debounce_down < 5) debounce_down++;
          else if (!down_now)                debounce_down = 0;

          int8_t desired = 0;
          if (debounce_down > 2)
            desired = 1;
          else if (debounce_up > 2)
            desired = -1;

          if (desired != stepper_dir) {
            /* direction change or start/stop — full reset before switch */
            HAL_TIM_Base_Stop_IT(&htim3);
            htim3.Instance->CNT = 0;
            stepper_period = 10499;
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

          /* accel ramp */
          if (stepper_dir != 0 && stepper_period > 4999) {
            stepper_period -= 250;
            if (stepper_period < 4999) stepper_period = 4999;
            htim3.Instance->ARR = stepper_period;
          }
        }

        /*
         * Servo0 (PC0, D-pad LEFT/RIGHT): 360° continuous rotation
         *   1500us=STOP, 500us=max CW, 2500us=max CCW
         * Servo1 (PC1, L2/R2): 180° angle servo — holds position on release
         *   500us=0°, 1500us=90°, 2500us=180°
         */
        #define SERVO360_DB  5u    /* debounce frames */
        #define S_180_STEP  25

        /* ---- servo0: 360° fixed-speed, stop on release ---- */
        {
          uint8_t left_now  = ps2_key_pressed(KEY_PHY_LEFT);
          uint8_t right_now = ps2_key_pressed(KEY_PHY_RIGHT);

          if (left_now && !right_now) {
            if (debounce_left  < 10) debounce_left++;
            debounce_right = 0;
          } else if (right_now && !left_now) {
            if (debounce_right < 10) debounce_right++;
            debounce_left = 0;
          } else {
            debounce_left = 0;
            debounce_right = 0;
          }

          uint16_t s0;
          if (debounce_left >= SERVO360_DB)
            s0 = SERVO360_STOP_US - SERVO360_SPEED_US;
          else if (debounce_right >= SERVO360_DB)
            s0 = SERVO360_STOP_US + SERVO360_SPEED_US;
          else
            s0 = SERVO360_STOP_US;

          servo_set_pulse(SERVO_CH0, s0);
        }

        /* ---- servo1: 180°, L2=收 R2=放 (3-frame debounce) ---- */
        {
          uint8_t l2_now = ps2_key_pressed(KEY_PHY_L2);
          uint8_t r2_now = ps2_key_pressed(KEY_PHY_R2);

          if (l2_now && debounce_l2 < 5) debounce_l2++;
          else if (!l2_now)             debounce_l2 = 0;
          if (r2_now && debounce_r2 < 5) debounce_r2++;
          else if (!r2_now)             debounce_r2 = 0;

          uint16_t s1 = servo_get_pulse(SERVO_CH1);
          if (debounce_l2 > 2) {
            if (s1 > 1500) s1 -= S_180_STEP;
          } else if (debounce_r2 > 2) {
            if (s1 < 2000) s1 += S_180_STEP;
          }
          servo_set_pulse(SERVO_CH1, s1);
          dbg_servo1_pulse = s1;  /* debug: watch this in Keil */
        }
	

        /* release = keep current position (no snap-back to center) */

        /* ---- 继电器: 按一下CIRCLE=吸合, 再按一下=断开 (toggle) ---- */
        {
          uint8_t circle_bit = (ps2.btn2 & 0x10) ? 1 : 0;  /* confirmed: btn2 127→111 */
          if (prev_circle_bit && !circle_bit) {   /* falling edge = press */
            relay_on = !relay_on;
          }
          prev_circle_bit = circle_bit;

          if (relay_on) {
            GPIOD->BSRR = (uint32_t)GPIO_PIN_12 << 16; /* LOW = 吸合 */
          } else {
            GPIOD->BSRR = GPIO_PIN_12;                  /* HIGH = 断开 */
          }
        }

        /* servo_tim4_glitch_check(); */
        ps2_fail_cnt = 0;  /* reset fail counter on successful read */
      } else {
        ps2_fail_cnt++;
        servo_set_pulse(SERVO_CH0, SERVO360_STOP_US);
        /* 连续10次失败(100ms)才判定真断连, 防止偶尔噪声误触发 */
        if (ps2_connected && ps2_fail_cnt > 10) {
          ps2_connected = 0;
          motor_pid[0].f_pid_reset(&motor_pid[0], 2.5f, 0.1f, 0.5f);
          motor_pid[1].f_pid_reset(&motor_pid[1], 2.5f, 0.1f, 0.5f);
          set_moto_current(&hcan1, 0, 0);
          HAL_TIM_Base_Stop_IT(&htim3);
          stepper_period = 10499;
          stepper_dir = 0;
          pul_state = 0;
          GPIOB->BSRR = (uint32_t)MOTO_PUL_Pin << 16;
          relay_on = 0;
          GPIOD->BSRR = GPIO_PIN_12;                  /* HIGH = 断开 */
        }
        if (!ps2_connected && HAL_GetTick() - ps2_reinit_tick > 200) {
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
    servo_tim4_period_elapsed();
  }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  servo_tim4_oc_match(htim->Channel);
}

void _Error_Handler(char * file, int line)
{
  while(1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
