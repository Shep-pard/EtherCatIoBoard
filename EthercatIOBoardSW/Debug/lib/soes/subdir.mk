################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../lib/soes/ecat_slv.c \
../lib/soes/esc.c \
../lib/soes/esc_coe.c \
../lib/soes/esc_eep.c \
../lib/soes/esc_eoe.c \
../lib/soes/esc_foe.c 

OBJS += \
./lib/soes/ecat_slv.o \
./lib/soes/esc.o \
./lib/soes/esc_coe.o \
./lib/soes/esc_eep.o \
./lib/soes/esc_eoe.o \
./lib/soes/esc_foe.o 

C_DEPS += \
./lib/soes/ecat_slv.d \
./lib/soes/esc.d \
./lib/soes/esc_coe.d \
./lib/soes/esc_eep.d \
./lib/soes/esc_eoe.d \
./lib/soes/esc_foe.d 


# Each subdirectory must supply rules for building sources it contributes
lib/soes/%.o lib/soes/%.su lib/soes/%.cyclo: ../lib/soes/%.c lib/soes/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Soeren/projects/EthercatIOBoardSW/lib/soes/include/sys/gcc" -I"C:/Users/Soeren/projects/EthercatIOBoardSW/lib/soes/include" -I"C:/Users/Soeren/projects/EthercatIOBoardSW/lib/soes/hal" -I"C:/Users/Soeren/projects/EthercatIOBoardSW/lib/soes" -I"C:/Users/Soeren/projects/EthercatIOBoardSW/lib/soes-esi" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-lib-2f-soes

clean-lib-2f-soes:
	-$(RM) ./lib/soes/ecat_slv.cyclo ./lib/soes/ecat_slv.d ./lib/soes/ecat_slv.o ./lib/soes/ecat_slv.su ./lib/soes/esc.cyclo ./lib/soes/esc.d ./lib/soes/esc.o ./lib/soes/esc.su ./lib/soes/esc_coe.cyclo ./lib/soes/esc_coe.d ./lib/soes/esc_coe.o ./lib/soes/esc_coe.su ./lib/soes/esc_eep.cyclo ./lib/soes/esc_eep.d ./lib/soes/esc_eep.o ./lib/soes/esc_eep.su ./lib/soes/esc_eoe.cyclo ./lib/soes/esc_eoe.d ./lib/soes/esc_eoe.o ./lib/soes/esc_eoe.su ./lib/soes/esc_foe.cyclo ./lib/soes/esc_foe.d ./lib/soes/esc_foe.o ./lib/soes/esc_foe.su

.PHONY: clean-lib-2f-soes

