################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../3rd_party/tmc/helpers/CRC.c \
../3rd_party/tmc/helpers/Functions.c 

OBJS += \
./3rd_party/tmc/helpers/CRC.o \
./3rd_party/tmc/helpers/Functions.o 

C_DEPS += \
./3rd_party/tmc/helpers/CRC.d \
./3rd_party/tmc/helpers/Functions.d 


# Each subdirectory must supply rules for building sources it contributes
3rd_party/tmc/helpers/%.o 3rd_party/tmc/helpers/%.su 3rd_party/tmc/helpers/%.cyclo: ../3rd_party/tmc/helpers/%.c 3rd_party/tmc/helpers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../3rd_party -I../Middlewares/ST/EEPROM_Emul/Core -I../Middlewares/ST/EEPROM_Emul/Porting/STM32G4 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-3rd_party-2f-tmc-2f-helpers

clean-3rd_party-2f-tmc-2f-helpers:
	-$(RM) ./3rd_party/tmc/helpers/CRC.cyclo ./3rd_party/tmc/helpers/CRC.d ./3rd_party/tmc/helpers/CRC.o ./3rd_party/tmc/helpers/CRC.su ./3rd_party/tmc/helpers/Functions.cyclo ./3rd_party/tmc/helpers/Functions.d ./3rd_party/tmc/helpers/Functions.o ./3rd_party/tmc/helpers/Functions.su

.PHONY: clean-3rd_party-2f-tmc-2f-helpers

