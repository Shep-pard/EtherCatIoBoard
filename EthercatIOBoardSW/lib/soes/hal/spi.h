#ifndef SRC_APP_SPI_H_
#define SRC_APP_SPI_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"


#define SCS_LOW                           0
#define SCS_HIGH                          1

#define DUMMY_BYTE 0xFF
#define tout 5000

  #define SPIX_ESC_SCS                    SPI_NSS_Soft
  #define SCS_ACTIVE_POLARITY             SCS_LOW
  // Mode 0 per SDK settings
  #define SPIX_ESC_CPOL                   SPI_CPOL_Low
  #define SPIX_ESC_CPHA                   SPI_CPHA_1Edge


void spi_setup(void);
void spi_select (int8_t board);
void spi_unselect (int8_t board);
void spi_write (int8_t board, uint8_t *data, uint8_t size);
void spi_read (int8_t board, uint8_t *result, uint8_t size);
void spi_bidirectionally_transfer (int8_t board, uint8_t *result, uint8_t *data, uint8_t size);


#endif /* SRC_APP_SPI_H_ */
