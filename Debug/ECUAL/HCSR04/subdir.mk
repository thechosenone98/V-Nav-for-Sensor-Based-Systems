################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/HCSR04/HCSR04.c \
../ECUAL/HCSR04/HCSR04_cfg.c 

OBJS += \
./ECUAL/HCSR04/HCSR04.o \
./ECUAL/HCSR04/HCSR04_cfg.o 

C_DEPS += \
./ECUAL/HCSR04/HCSR04.d \
./ECUAL/HCSR04/HCSR04_cfg.d 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/HCSR04/%.o: ../ECUAL/HCSR04/%.c ECUAL/HCSR04/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"/Users/zach-mcc/Documents/STM32 Projects/UltrasonicVest/ECUAL/HCSR04" -I"/Users/zach-mcc/Documents/STM32 Projects/UltrasonicVest/util" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-HCSR04

clean-ECUAL-2f-HCSR04:
	-$(RM) ./ECUAL/HCSR04/HCSR04.d ./ECUAL/HCSR04/HCSR04.o ./ECUAL/HCSR04/HCSR04_cfg.d ./ECUAL/HCSR04/HCSR04_cfg.o

.PHONY: clean-ECUAL-2f-HCSR04

