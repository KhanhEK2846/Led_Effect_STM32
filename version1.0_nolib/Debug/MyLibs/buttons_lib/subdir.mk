################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
E:/Study-space/First_sem_4thyear/Chuyende_TKHTN1/MyLibs/buttons_lib/button.c 

OBJS += \
./MyLibs/buttons_lib/button.o 

C_DEPS += \
./MyLibs/buttons_lib/button.d 


# Each subdirectory must supply rules for building sources it contributes
MyLibs/buttons_lib/button.o: E:/Study-space/First_sem_4thyear/Chuyende_TKHTN1/MyLibs/buttons_lib/button.c MyLibs/buttons_lib/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -IC:/Users/Hoa/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -IC:/Users/Hoa/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/STM32F1xx_HAL_Driver/Inc -IC:/Users/Hoa/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/CMSIS/Device/ST/STM32F1xx/Include -IC:/Users/Hoa/STM32Cube/Repository/STM32Cube_FW_F1_V1.8.5/Drivers/CMSIS/Include -I"E:/Study-space/First_sem_4thyear/Chuyende_TKHTN1/MyLibs" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-MyLibs-2f-buttons_lib

clean-MyLibs-2f-buttons_lib:
	-$(RM) ./MyLibs/buttons_lib/button.d ./MyLibs/buttons_lib/button.o ./MyLibs/buttons_lib/button.su

.PHONY: clean-MyLibs-2f-buttons_lib

