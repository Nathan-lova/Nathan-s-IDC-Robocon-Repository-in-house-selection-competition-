#include "ps2.h"
#include "stm32f4xx_hal.h"

/* PS2 pins */
#define PS2_CS   GPIO_PIN_6    /* PI6: CS/SEL, output */
#define PS2_SCK  GPIO_PIN_1    /* PA1: clock, output */
#define PS2_MISO GPIO_PIN_2      /* PA2: data from controller, input */
#define PS2_MOSI GPIO_PIN_7    /* PI7: command to controller, output */

#define PS2_CS_PORT   GPIOI
#define PS2_SCK_PORT  GPIOA
#define PS2_MISO_PORT GPIOA
#define PS2_MOSI_PORT GPIOI

/* pin macros */
#define CS_H()   (PS2_CS_PORT->BSRR = PS2_CS)
#define CS_L()   (PS2_CS_PORT->BSRR = (uint32_t)PS2_CS << 16)
#define SCK_H()  (PS2_SCK_PORT->BSRR = PS2_SCK)
#define SCK_L()  (PS2_SCK_PORT->BSRR = (uint32_t)PS2_SCK << 16)
#define MOSI_H() (PS2_MOSI_PORT->BSRR = PS2_MOSI)
#define MOSI_L() (PS2_MOSI_PORT->BSRR = (uint32_t)PS2_MOSI << 16)
#define MISO_R() ((PS2_MISO_PORT->IDR & PS2_MISO) ? 1 : 0)

/* DWT-based microsecond delay (168MHz = 168 cycles/us) */
static void delay_us(u16 n)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (uint32_t)n * 168;
    while ((DWT->CYCCNT - start) < ticks);
}

/*
 * Bit-bang one byte, LSB first (PS2 protocol requirement).
 * CPOL=0 (SCK idle low), CPHA=0:
 *   - set MOSI, then SCK rising  -> PS2 samples  MOSI
 *   - SCK falling -> PS2 outputs MISO, we read it
 * Original Robocon timing: 3+6+3us per bit (proven working).
 */
static u8 ps2_xfer_byte(u8 tx)
{
    u8 rx = 0;
    u8 i;

    for (i = 0; i < 8; i++) {
        if (tx & (1 << i))
            MOSI_H();
        else
            MOSI_L();
        delay_us(3);

        SCK_H();
        delay_us(6);

        SCK_L();
        delay_us(3);

        if (MISO_R())
            rx |= (1 << i);
    }
    return rx;
}

void ps2_init(void)
{
    /* enable DWT cycle counter for delay_us */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();

    /* PA1(SCK): output push-pull */
    GPIOA->MODER   = (GPIOA->MODER   & ~(3u << 2))  | (1u << 2);
    GPIOA->OTYPER  =  GPIOA->OTYPER  & ~(1u << 1);
    GPIOA->PUPDR   =  GPIOA->PUPDR   & ~(3u << 2);
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~(3u << 2)) | (3u << 2);

    /* PA2(MISO): input, pull-up */
    GPIOA->MODER  &= ~(3u << 4);
    GPIOA->PUPDR  = (GPIOA->PUPDR & ~(3u << 4)) | (1u << 4);

    /* PI6(CS): output push-pull */
    GPIOI->MODER   = (GPIOI->MODER   & ~(3u << 12)) | (1u << 12);
    GPIOI->OTYPER  =  GPIOI->OTYPER  & ~(1u << 6);
    GPIOI->PUPDR   =  GPIOI->PUPDR   & ~(3u << 12);
    GPIOI->OSPEEDR = (GPIOI->OSPEEDR & ~(3u << 12)) | (3u << 12);

    /* PI7(MOSI): output push-pull */
    GPIOI->MODER   = (GPIOI->MODER   & ~(3u << 14)) | (1u << 14);
    GPIOI->OTYPER  =  GPIOI->OTYPER  & ~(1u << 7);
    GPIOI->PUPDR   =  GPIOI->PUPDR   & ~(3u << 14);
    GPIOI->OSPEEDR = (GPIOI->OSPEEDR & ~(3u << 14)) | (3u << 14);

    /* idle: CS=HIGH, SCK=LOW, MOSI=HIGH */
    CS_H();
    SCK_L();
    MOSI_H();
}

u8 ps2_read(ps2_state_t *state)
{
    /*
     * PS2 poll command sequence.
     *
     * We send  : 0x01  0x42  0x00  0x00  0x00  0x00  0x00  0x00  0x00
     * We recv  :  -     ID   0x5A   btn1  btn2  R-X   R-Y   L-X   L-Y
     *         rx[0] rx[1] rx[2] rx[3] rx[4] rx[5] rx[6] rx[7] rx[8]
     */
    u8 tx[9] = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    u8 rx[9];
    u8 i;

    /* Block IRQs at priority >=1 so TIM4 (priority 0) keeps running —
       otherwise servo PWM glitches every PS2 poll causing 180° servo jitter. */
    __set_BASEPRI(0x10);
    CS_L();
    delay_us(50);

    for (i = 0; i < 9; i++) {
        rx[i] = ps2_xfer_byte(tx[i]);
    }

    CS_H();
    MOSI_H();
    __set_BASEPRI(0);

    state->id     = rx[1];
    state->btn1   = rx[3];     /* 0 = pressed */
    state->btn2   = rx[4];     /* 0 = pressed */
    state->joy_rx = rx[5];     /* 0=left, 0x80=center, 0xFF=right */
    state->joy_ry = rx[6];     /* 0=up,   0x80=center, 0xFF=down */
    state->joy_lx = rx[7];     /* 0=left, 0x80=center, 0xFF=right */
    state->joy_ly = rx[8];     /* 0=up,   0x80=center, 0xFF=down */

    /* reject all-zero or all-0xFF (noise / disconnected) */
    if (state->btn1 == 0x00 && state->btn2 == 0x00 &&
        state->joy_rx == 0x00 && state->joy_ry == 0x00 &&
        state->joy_lx == 0x00 && state->joy_ly == 0x00)
        return 0;
    if (state->btn1 == 0xFF && state->btn2 == 0xFF &&
        state->joy_rx == 0xFF && state->joy_ry == 0xFF &&
        state->joy_lx == 0xFF && state->joy_ly == 0xFF)
        return 0;

    return 1;
}
