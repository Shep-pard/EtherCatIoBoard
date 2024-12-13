#include "spi.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;

void spi_gpio_setup(void)
{




}

void spi_setup(void)
{

}

void spi_select (int8_t board)
{
    // Soft CSM
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0);
}

void spi_unselect (int8_t board)
{
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 1);
}

inline static uint8_t spi_transfer(uint8_t byte)
{
	uint8_t receive = 0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &receive, 1, 50);


    return receive;
}

void spi_write (int8_t board, uint8_t *data, uint8_t size)
{
	HAL_SPI_Transmit(&hspi1, data, size, 100);
}

void spi_read (int8_t board, uint8_t *result, uint8_t size)
{
	HAL_SPI_Receive(&hspi1, result, size, 100);
}


void spi_bidirectionally_transfer (int8_t board, uint8_t *result, uint8_t *data, uint8_t size)
{

	HAL_SPI_TransmitReceive(&hspi1, data, result, size, 50);
}
