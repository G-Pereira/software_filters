################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
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
LowPassFilter2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LowPassFilter2.list \


# All Target
all: LowPassFilter2.elf secondary-outputs

# Tool invocations
LowPassFilter2.elf: $(OBJS) $(USER_OBJS) D:\OneDrive\ -\ Universidade\ do\ Porto\Faculdade\4\ Ano\2\ Semestre\SBIC\Projeto\stm32CubeIDE\workspace_1.0.0\LowPassFilter2\STM32F103C8TX_FLASH.ld
	arm-none-eabi-gcc -o "LowPassFilter2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"D:\OneDrive - Universidade do Porto\Faculdade\4 Ano\2 Semestre\SBIC\Projeto\stm32CubeIDE\workspace_1.0.0\LowPassFilter2\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LowPassFilter2.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LowPassFilter2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LowPassFilter2.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
