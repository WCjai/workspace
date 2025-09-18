################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include example/src/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := periph_uart_rb
BUILD_ARTIFACT_EXTENSION := axf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all:
	+@$(MAKE) --no-print-directory main-build && $(MAKE) --no-print-directory post-build

# Main-build Target
main-build: periph_uart_rb.axf

# Tool invocations
periph_uart_rb.axf: $(OBJS) $(USER_OBJS) makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU Linker'
	arm-none-eabi-gcc -nostdlib -L"C:\Users\mad23\Documents\MCUXpressoIDE_25.6.136\workspace\lpc_chip_175x_6x\Debug" -L"C:\Users\mad23\Documents\MCUXpressoIDE_25.6.136\workspace\lpc_board_nxp_lpcxpresso_1769\Debug" -Xlinker -Map="periph_uart_rb.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T periph_uart_rb_Debug.ld -o "periph_uart_rb.axf" $(OBJS) $(USER_OBJS) $(LIBS) -llpc_board_nxp_lpcxpresso_1769 -llpc_chip_175x_6x
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) periph_uart_rb.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "periph_uart_rb.axf" && ^ ; arm-none-eabi-objcopy -v -O binary "periph_uart_rb.axf" "periph_uart_rb.bin" && ^ ; arm-none-eabi-objcopy -v -O ihex   "periph_uart_rb.axf" "periph_uart_rb.hex" && ^ ; checksum -p LPC1769 -d "periph_uart_rb.bin"
	-@echo ' '

.PHONY: all clean dependents main-build post-build

-include ../makefile.targets
