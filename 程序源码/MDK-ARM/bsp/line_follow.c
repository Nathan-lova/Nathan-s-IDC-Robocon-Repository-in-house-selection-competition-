#include "line_follow.h"
#include "pid.h"
#include "stm32f4xx_hal.h"

#define LF_LOST_TIMEOUT_MS  500

static uint8_t  lf_active;
static uint32_t lf_start_tick;
static uint32_t lf_lost_tick;
static float    lf_last_pos;
static uint8_t  lf_line_lost;

static PID_TypeDef lf_pid;

void lf_start(void)
{
    pid_init(&lf_pid);
    lf_pid.f_param_init(&lf_pid, PID_Position,
                        3000,     /* MaxOutput */
                        200,      /* IntegralLimit */
                        0.05f,    /* DeadBand */
                        10,       /* ControlPeriod */
                        500,      /* Max_Err */
                        0,        /* target = 0 (line centered) */
                        800.0f,   /* kp */
                        30.0f,    /* ki */
                        200.0f);  /* kd */

    lf_active     = 1;
    lf_start_tick = HAL_GetTick();
    lf_line_lost  = 0;
    lf_last_pos   = 0.0f;
}

void lf_stop(void)
{
    lf_active    = 0;
    lf_line_lost = 0;
}

uint8_t lf_is_active(void)
{
    return lf_active;
}

float lf_update(uint8_t bits)
{
    if (!lf_active) return 0.0f;

    /* 15-second auto-stop */
    if (HAL_GetTick() - lf_start_tick > LF_DURATION_MS) {
        lf_stop();
        return 0.0f;
    }

    /* weighted centroid: sensor 0..7, weight = index */
    int sum_w = 0, cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (bits & (1 << i)) {
            sum_w += i;
            cnt++;
        }
    }

    float pos;
    if (cnt == 0) {
        if (!lf_line_lost) {
            lf_line_lost  = 1;
            lf_lost_tick  = HAL_GetTick();
        }
        if (HAL_GetTick() - lf_lost_tick > LF_LOST_TIMEOUT_MS) {
            lf_stop();
            return 0.0f;
        }
        pos = lf_last_pos;
    } else {
        lf_line_lost = 0;
        pos = (float)sum_w / (float)cnt - 3.5f;
        lf_last_pos = pos;
    }

    lf_pid.target = 0.0f;
    float steer = lf_pid.f_cal_pid(&lf_pid, pos);

    if (steer >  3000.0f) steer =  3000.0f;
    if (steer < -3000.0f) steer = -3000.0f;

    return steer;
}
