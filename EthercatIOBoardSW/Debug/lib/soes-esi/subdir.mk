################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/soes-esi/objectlist.c 

OBJS += \
./lib/soes-esi/objectlist.o 

C_DEPS += \
./lib/soes-esi/objectlist.d 


# Each subdirectory must supply rules for building sources it contributes
lib/soes-esi/%.o lib/soes-esi/%.su lib/soes-esi/%.cyclo: ../lib/soes-esi/%.c lib/soes-esi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes/include/sys/gcc" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes/include" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes/hal" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes" -I"C:/Users/Soeren/projects/EtherCatIoBoard/EthercatIOBoardSW/lib/soes-esi" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-soes-2d-esi

clean-lib-2f-soes-2d-esi:
	-$(RM) ./lib/soes-esi/objectlist.cyclo ./lib/soes-esi/objectlist.d ./lib/soes-esi/objectlist.o ./lib/soes-esi/objectlist.su

.PHONY: clean-lib-2f-soes-2d-esi

