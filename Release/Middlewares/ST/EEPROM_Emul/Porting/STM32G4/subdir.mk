################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/EEPROM_Emul/Porting/STM32G4/flash_interface.c 

OBJS += \
./Middlewares/ST/EEPROM_Emul/Porting/STM32G4/flash_interface.o 

C_DEPS += \
./Middlewares/ST/EEPROM_Emul/Porting/STM32G4/flash_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/EEPROM_Emul/Porting/STM32G4/%.o Middlewares/ST/EEPROM_Emul/Porting/STM32G4/%.su: ../Middlewares/ST/EEPROM_Emul/Porting/STM32G4/%.c Middlewares/ST/EEPROM_Emul/Porting/STM32G4/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../3rd_party -I../Middlewares/ST/EEPROM_Emul/Core -I../Middlewares/ST/EEPROM_Emul/Porting/STM32G4 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-ST-2f-EEPROM_Emul-2f-Porting-2f-STM32G4

clean-Middlewares-2f-ST-2f-EEPROM_Emul-2f-Porting-2f-STM32G4:
	-$(RM) ./Middlewares/ST/EEPROM_Emul/Porting/STM32G4/flash_interface.d ./Middlewares/ST/EEPROM_Emul/Porting/STM32G4/flash_interface.o ./Middlewares/ST/EEPROM_Emul/Porting/STM32G4/flash_interface.su

.PHONY: clean-Middlewares-2f-ST-2f-EEPROM_Emul-2f-Porting-2f-STM32G4

