################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/sixSector/subdir.mk
-include Drivers/Temperature/subdir.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
-include Drivers/Ramp/subdir.mk
-include Drivers/PositioningPoints/subdir.mk
-include Drivers/PosRamp/subdir.mk
-include Drivers/PosCntrl_OL/subdir.mk
-include Drivers/PosCntrl_CL/subdir.mk
-include Drivers/PID/subdir.mk
-include Drivers/GB/subdir.mk
-include Drivers/FDCAN/Motor/subdir.mk
-include Drivers/FDCAN/subdir.mk
-include Drivers/EncoderSpeed/subdir.mk
-include Drivers/EncoderPosition/subdir.mk
-include Drivers/EncoderCalibration/subdir.mk
-include Drivers/Eeprom/subdir.mk
-include Drivers/Console/subdir.mk
-include Drivers/AS5x47PS/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := LiftMotorCode_Flyer_rev1
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LiftMotorCode_Flyer_rev1.elf \

MAP_FILES += \
LiftMotorCode_Flyer_rev1.map \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LiftMotorCode_Flyer_rev1.list \

OBJCOPY_HEX += \
LiftMotorCode_Flyer_rev1.hex \


# All Target
all: main-build

# Main-build Target
main-build: LiftMotorCode_Flyer_rev1.elf secondary-outputs

# Tool invocations
LiftMotorCode_Flyer_rev1.elf LiftMotorCode_Flyer_rev1.map: $(OBJS) $(USER_OBJS) /home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/STM32G431CBTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "LiftMotorCode_Flyer_rev1.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/harsha/STM32CubeIDE/workspace_1.10.0/LiftMotorCode_Flyer_rev1/STM32G431CBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LiftMotorCode_Flyer_rev1.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -u _scanf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LiftMotorCode_Flyer_rev1.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LiftMotorCode_Flyer_rev1.list"
	@echo 'Finished building: $@'
	@echo ' '

LiftMotorCode_Flyer_rev1.hex: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O ihex $(EXECUTABLES) "LiftMotorCode_Flyer_rev1.hex"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) LiftMotorCode_Flyer_rev1.elf LiftMotorCode_Flyer_rev1.hex LiftMotorCode_Flyer_rev1.list LiftMotorCode_Flyer_rev1.map default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_HEX)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
