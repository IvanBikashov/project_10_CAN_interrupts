################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32G4xx_HAL_Driver/Src/subdir.mk
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
test_project_10_CAN_interrupts.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
test_project_10_CAN_interrupts.list \

OBJCOPY_BIN += \
test_project_10_CAN_interrupts.bin \


# All Target
all: test_project_10_CAN_interrupts.elf secondary-outputs

# Tool invocations
test_project_10_CAN_interrupts.elf: $(OBJS) $(USER_OBJS) C:\Users\IvanB.HEL-KONSTR7\Documents\GitHub\motor_repository_10\STM32G431KBTX_FLASH.ld
	arm-none-eabi-gcc -o "test_project_10_CAN_interrupts.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\IvanB.HEL-KONSTR7\Documents\GitHub\motor_repository_10\STM32G431KBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="test_project_10_CAN_interrupts.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

test_project_10_CAN_interrupts.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "test_project_10_CAN_interrupts.list"
	@echo 'Finished building: $@'
	@echo ' '

test_project_10_CAN_interrupts.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "test_project_10_CAN_interrupts.bin"
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
