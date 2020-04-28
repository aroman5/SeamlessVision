################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
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
SeamlessVision.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SeamlessVision.list \

OBJCOPY_BIN += \
SeamlessVision.bin \


# All Target
all: SeamlessVision.elf secondary-outputs

# Tool invocations
SeamlessVision.elf: $(OBJS) $(USER_OBJS) C:\!MyProjects\MyClients\Aran\SeamlessVision\Code\SeamlessVision\STM32F031K6TX_FLASH.ld
	arm-none-eabi-gcc -o "SeamlessVision.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\!MyProjects\MyClients\Aran\SeamlessVision\Code\SeamlessVision\STM32F031K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SeamlessVision.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SeamlessVision.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SeamlessVision.list"
	@echo 'Finished building: $@'
	@echo ' '

SeamlessVision.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SeamlessVision.bin"
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
