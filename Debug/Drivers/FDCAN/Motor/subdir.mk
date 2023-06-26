################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/FDCAN/Motor/CAN_Motor.c 

OBJS += \
./Drivers/FDCAN/Motor/CAN_Motor.o 

C_DEPS += \
./Drivers/FDCAN/Motor/CAN_Motor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/FDCAN/Motor/%.o Drivers/FDCAN/Motor/%.su: ../Drivers/FDCAN/Motor/%.c Drivers/FDCAN/Motor/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Eeprom" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/GB" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PosCntrl_CL" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PosCntrl_OL" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Console" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PosRamp" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PID" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/EncoderSpeed" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/FDCAN/Motor" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Temperature" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/FDCAN" -I../Core/Inc -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Core/Inc" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Ramp" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/sixSector" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/EncoderCalibration" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/AS5x47PS" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-FDCAN-2f-Motor

clean-Drivers-2f-FDCAN-2f-Motor:
	-$(RM) ./Drivers/FDCAN/Motor/CAN_Motor.d ./Drivers/FDCAN/Motor/CAN_Motor.o ./Drivers/FDCAN/Motor/CAN_Motor.su

.PHONY: clean-Drivers-2f-FDCAN-2f-Motor

