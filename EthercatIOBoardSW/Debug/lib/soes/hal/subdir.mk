################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/soes/hal/esc_hw.c \
../lib/soes/hal/esc_irq.c \
../lib/soes/hal/rst.c \
../lib/soes/hal/spi.c 

OBJS += \
./lib/soes/hal/esc_hw.o \
./lib/soes/hal/esc_irq.o \
./lib/soes/hal/rst.o \
./lib/soes/hal/spi.o 

C_DEPS += \
./lib/soes/hal/esc_hw.d \
./lib/soes/hal/esc_irq.d \
./lib/soes/hal/rst.d \
./lib/soes/hal/spi.d 


# Each subdirectory must supply rules for building sources it contributes
lib/soes/hal/%.o lib/soes/hal/%.su lib/soes/hal/%.cyclo: ../lib/soes/hal/%.c lib/soes/hal/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes/include/sys/gcc" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes/include" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes/hal" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes-esi" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-soes-2f-hal

clean-lib-2f-soes-2f-hal:
	-$(RM) ./lib/soes/hal/esc_hw.cyclo ./lib/soes/hal/esc_hw.d ./lib/soes/hal/esc_hw.o ./lib/soes/hal/esc_hw.su ./lib/soes/hal/esc_irq.cyclo ./lib/soes/hal/esc_irq.d ./lib/soes/hal/esc_irq.o ./lib/soes/hal/esc_irq.su ./lib/soes/hal/rst.cyclo ./lib/soes/hal/rst.d ./lib/soes/hal/rst.o ./lib/soes/hal/rst.su ./lib/soes/hal/spi.cyclo ./lib/soes/hal/spi.d ./lib/soes/hal/spi.o ./lib/soes/hal/spi.su

.PHONY: clean-lib-2f-soes-2f-hal

