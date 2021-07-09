################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LIS3DSH_STM32F4_Discovery.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LIS3DSH_STM32F4_Discovery.list \

OBJCOPY_BIN += \
LIS3DSH_STM32F4_Discovery.bin \


# All Target
all: LIS3DSH_STM32F4_Discovery.elf secondary-outputs

# Tool invocations
LIS3DSH_STM32F4_Discovery.elf: $(OBJS) $(USER_OBJS) /home/miirho3ein/STM32CubeIDE/workspace_1.3.0/LIS3DSH_STM32F4_Discovery/STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "LIS3DSH_STM32F4_Discovery.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/miirho3ein/STM32CubeIDE/workspace_1.3.0/LIS3DSH_STM32F4_Discovery/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LIS3DSH_STM32F4_Discovery.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LIS3DSH_STM32F4_Discovery.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LIS3DSH_STM32F4_Discovery.list"
	@echo 'Finished building: $@'
	@echo ' '

LIS3DSH_STM32F4_Discovery.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LIS3DSH_STM32F4_Discovery.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
