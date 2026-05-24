#include "bsp_ir8.h"

#define IR8_ADDR_8BIT  0x60
#define IR8_REG_SENSOR 0x50

static I2C_HandleTypeDef hi2c2;

void IR8_Init(void)
{
    /* GPIO: PF0=I2C2_SCL, PF1=I2C2_SDA, AF4, open-drain */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOF, &gpio);

    /* I2C2: 100 kHz standard mode */
    __HAL_RCC_I2C2_CLK_ENABLE();

    hi2c2.Instance             = I2C2;
    hi2c2.Init.ClockSpeed      = 100000;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
}

uint8_t IR8_Read(void)
{
    uint8_t bits = 0;
    HAL_I2C_Mem_Read(&hi2c2, IR8_ADDR_8BIT, IR8_REG_SENSOR,
                     I2C_MEMADD_SIZE_8BIT, &bits, 1, 10);
    return bits;
}
