################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SVPWM/SVPWM.c 

OBJS += \
./Drivers/SVPWM/SVPWM.o 

C_DEPS += \
./Drivers/SVPWM/SVPWM.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SVPWM/%.o Drivers/SVPWM/%.su: ../Drivers/SVPWM/%.c Drivers/SVPWM/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/Eeprom" -I../Core/Inc -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Core/Inc" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/Ramp" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/sixSector" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/EncoderCalibration" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/AS5x47PS" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/SVPWM" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_rev2/Drivers/Q15_math" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-SVPWM

clean-Drivers-2f-SVPWM:
	-$(RM) ./Drivers/SVPWM/SVPWM.d ./Drivers/SVPWM/SVPWM.o ./Drivers/SVPWM/SVPWM.su

.PHONY: clean-Drivers-2f-SVPWM

