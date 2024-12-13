#include "rst.h"

void rst_setup(void)
{
    /* Setup NRST as GPIO out and pull it high */
	GPIO_InitTypeDef gpio;
 

    gpio.Pin   = ESC_GPIO_Pin_RSTN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Pull  = GPIO_PULLUP;
    HAL_GPIO_Init(ESC_GPIOX_RSTN, &gpio);
    
    rst_high();
}

void rst_low(void)
{    /* Set RSTN line low */
	HAL_GPIO_WritePin(ESC_GPIOX_RSTN, ESC_GPIO_Pin_RSTN, 0);
}

void rst_high(void)
{
    /* Set RSTN line high */
	HAL_GPIO_WritePin(ESC_GPIOX_RSTN, ESC_GPIO_Pin_RSTN, 1);
}

void rst_check_start(void)
{
    /* Setup NRST as GPIO input and pull it high */
    GPIO_InitTypeDef gpio;
 

    gpio.Pin   = ESC_GPIO_Pin_RSTN;
    gpio.Mode  = GPIO_MODE_INPUT;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(ESC_GPIOX_RSTN, &gpio);
}

uint8_t is_esc_reset(void)
{
    /* Check if ESC pulled RSTN line up */ 
    return HAL_GPIO_ReadPin(ESC_GPIOX_RSTN, ESC_GPIO_Pin_RSTN);
}

