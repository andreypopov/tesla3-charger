TARGET := build/tesla_charger

CC := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
SIZE := arm-none-eabi-size
HOST_CC ?= cc

C_SOURCES := \
	$(wildcard Core/Src/*.c) \
	$(wildcard Drivers/STM32F1xx_HAL_Driver/Src/*.c)

ASM_SOURCES := Core/Startup/startup_stm32f103c8tx.s

OBJECTS := \
	$(patsubst %.c,build/%.o,$(C_SOURCES)) \
	$(patsubst %.s,build/%.o,$(ASM_SOURCES))

INCLUDES := \
	-ICore/Inc \
	-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
	-IDrivers/STM32F1xx_HAL_Driver/Inc \
	-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
	-IDrivers/CMSIS/Include

CPU_FLAGS := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
CFLAGS := $(CPU_FLAGS) -std=gnu11 -O0 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB \
	$(INCLUDES) -Wall -Wextra -Wpedantic -ffunction-sections -fdata-sections \
	-fstack-usage -MMD -MP

LDFLAGS := $(CPU_FLAGS) -TSTM32F103C8TX_FLASH.ld --specs=nosys.specs \
	--specs=nano.specs -Wl,-Map=$(TARGET).map -Wl,--gc-sections -static \
	-Wl,--start-group -lc -lm -Wl,--end-group

.PHONY: all clean test

all: $(TARGET).elf $(TARGET).bin
	$(SIZE) $(TARGET).elf

test: build/tests/test_pcs_protocol
	./build/tests/test_pcs_protocol

build/tests/test_pcs_protocol: tests/test_pcs_protocol.c Core/Src/pcs_protocol.c Core/Inc/pcs_protocol.h
	@mkdir -p $(dir $@)
	$(HOST_CC) -std=c11 -Wall -Wextra -Wpedantic -Werror -ICore/Inc \
		tests/test_pcs_protocol.c Core/Src/pcs_protocol.c -o $@

$(TARGET).elf: $(OBJECTS) STM32F103C8TX_FLASH.ld
	@mkdir -p $(dir $@)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

build/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(CPU_FLAGS) -x assembler-with-cpp -c $< -o $@

clean:
	@find build -type f -delete 2>/dev/null || true

-include $(OBJECTS:.o=.d)
