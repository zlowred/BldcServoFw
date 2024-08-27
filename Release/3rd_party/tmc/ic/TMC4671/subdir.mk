################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../3rd_party/tmc/ic/TMC4671/TMC4671.c 

OBJS += \
./3rd_party/tmc/ic/TMC4671/TMC4671.o 

C_DEPS += \
./3rd_party/tmc/ic/TMC4671/TMC4671.d 


# Each subdirectory must supply rules for building sources it contributes
3rd_party/tmc/ic/TMC4671/%.o 3rd_party/tmc/ic/TMC4671/%.su 3rd_party/tmc/ic/TMC4671/%.cyclo: ../3rd_party/tmc/ic/TMC4671/%.c 3rd_party/tmc/ic/TMC4671/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../USB_Device/App -I../USB_Device/Target -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../3rd_party -I../Middlewares/ST/EEPROM_Emul/Core -I../Middlewares/ST/EEPROM_Emul/Porting/STM32G4 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-3rd_party-2f-tmc-2f-ic-2f-TMC4671

clean-3rd_party-2f-tmc-2f-ic-2f-TMC4671:
	-$(RM) ./3rd_party/tmc/ic/TMC4671/TMC4671.cyclo ./3rd_party/tmc/ic/TMC4671/TMC4671.d ./3rd_party/tmc/ic/TMC4671/TMC4671.o ./3rd_party/tmc/ic/TMC4671/TMC4671.su

.PHONY: clean-3rd_party-2f-tmc-2f-ic-2f-TMC4671

