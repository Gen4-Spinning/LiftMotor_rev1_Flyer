################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Init_TypeDefs.c \
../Core/Src/main.c \
../Core/Src/retarget.c \
../Core/Src/stm32g4xx_hal_msp.c \
../Core/Src/stm32g4xx_it.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g4xx.c 

OBJS += \
./Core/Src/Init_TypeDefs.o \
./Core/Src/main.o \
./Core/Src/retarget.o \
./Core/Src/stm32g4xx_hal_msp.o \
./Core/Src/stm32g4xx_it.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g4xx.o 

C_DEPS += \
./Core/Src/Init_TypeDefs.d \
./Core/Src/main.d \
./Core/Src/retarget.d \
./Core/Src/stm32g4xx_hal_msp.d \
./Core/Src/stm32g4xx_it.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PosCntrl_CL" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/EncoderPosition" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PositioningPoints" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Eeprom" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/GB" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PosCntrl_OL" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PosRamp" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Console" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/PID" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/EncoderSpeed" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/FDCAN/Motor" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Temperature" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/FDCAN" -I../Core/Inc -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Core/Inc" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/sixSector" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/Ramp" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/EncoderCalibration" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/Drivers/AS5x47PS" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Init_TypeDefs.d ./Core/Src/Init_TypeDefs.o ./Core/Src/Init_TypeDefs.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/retarget.d ./Core/Src/retarget.o ./Core/Src/retarget.su ./Core/Src/stm32g4xx_hal_msp.d ./Core/Src/stm32g4xx_hal_msp.o ./Core/Src/stm32g4xx_hal_msp.su ./Core/Src/stm32g4xx_it.d ./Core/Src/stm32g4xx_it.o ./Core/Src/stm32g4xx_it.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g4xx.d ./Core/Src/system_stm32g4xx.o ./Core/Src/system_stm32g4xx.su

.PHONY: clean-Core-2f-Src

