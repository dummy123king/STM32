################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bsp/lcd.c 

OBJS += \
./Bsp/lcd.o 

C_DEPS += \
./Bsp/lcd.d 


# Each subdirectory must supply rules for building sources it contributes
Bsp/%.o Bsp/%.su Bsp/%.cyclo: ../Bsp/%.c Bsp/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F401RBTx -DSTM32F4 -c -I"/home/mirafra/STM32CubeIDE/workspace_1.14.0/KernelMastersCustomBoard/Drivers/Inc" -I"/home/mirafra/STM32CubeIDE/workspace_1.14.0/KernelMastersCustomBoard/Bsp" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Bsp

clean-Bsp:
	-$(RM) ./Bsp/lcd.cyclo ./Bsp/lcd.d ./Bsp/lcd.o ./Bsp/lcd.su

.PHONY: clean-Bsp

