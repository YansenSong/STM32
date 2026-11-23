CROSS_COMPILE ?= arm-none-eabi-
CC = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size
JLINK ?= JLinkExe
HAL = Libraries/STM32F4xx_HAL_Driver
CMSIS = Libraries/CMSIS
FREERTOS = Libraries/FreeRTOS-Kernel
RTOS_PORT = $(FREERTOS)/portable/GCC/ARM_CM4F
TARGET = build/vehicle
CPU = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
APP_MODULES = Application/Board Application/Commands Application/Health Application/Ota Application/Vehicle
CPPFLAGS = -DSTM32F407xx -DUSE_HAL_DRIVER $(addprefix -I,Application/Core/inc $(APP_MODULES) $(HAL)/Inc $(CMSIS)/Include $(CMSIS)/Device/ST/STM32F4xx/Include $(FREERTOS)/include $(RTOS_PORT) firmware/ota)
CFLAGS = $(CPU) -std=gnu11 -Os -g3 -Wall -Wextra -ffunction-sections -fdata-sections -MMD -MP
HAL_MODULES = hal hal_can hal_cortex hal_dma hal_dma_ex hal_eth hal_flash hal_flash_ex hal_gpio hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_uart
SOURCES = $(wildcard Application/Core/src/*.c $(addsuffix /*.c,$(APP_MODULES))) $(CMSIS)/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c $(addprefix $(HAL)/Src/stm32f4xx_,$(addsuffix .c,$(HAL_MODULES)))
SOURCES += $(FREERTOS)/tasks.c $(FREERTOS)/list.c $(FREERTOS)/queue.c $(RTOS_PORT)/port.c
OBJECTS = $(addprefix build/,$(SOURCES:.c=.o)) build/firmware/startup_stm32f407xx.o
.PHONY: all clean flash
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin
$(TARGET).elf: $(OBJECTS) firmware/stm32f407zg.ld
	$(CC) $(CPU) --specs=nano.specs --specs=nosys.specs -Tfirmware/stm32f407zg.ld -Wl,--gc-sections,-Map=$(TARGET).map $(OBJECTS) -Wl,--start-group -lc -lm -Wl,--end-group -o $@
	$(SIZE) $@
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
build/%.o: %.S
	@mkdir -p $(dir $@)
	$(CC) $(CPU) -c $< -o $@
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@
$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@
flash: all
	$(JLINK) -NoGui 1 -ExitOnError 1 -Device STM32F407ZG -If SWD -Speed 1000 -AutoConnect 1 -CommandFile firmware/flash.jlink
clean:
	rm -rf build
-include $(OBJECTS:.o=.d)

OTA_DIR = build/ota
.PHONY: ota
ota: $(OTA_DIR)/boot.hex $(OTA_DIR)/slot-a.hex $(OTA_DIR)/slot-b.hex $(OTA_DIR)/meta-a.hex $(OTA_DIR)/meta-b.hex

$(OTA_DIR)/boot.elf: firmware/ota/boot.c firmware/ota/boot.S firmware/ota/boot.ld firmware/ota/layout.h firmware/ota/flash_word.h
	@mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CPU) -Os -ffreestanding -nostdlib -Wl,-T,firmware/ota/boot.ld firmware/ota/boot.S firmware/ota/boot.c -o $@

$(OTA_DIR)/slot-a.elf: OTA_ORIGIN = 0x08020000
$(OTA_DIR)/slot-b.elf: OTA_ORIGIN = 0x08080000
$(OTA_DIR)/slot-a.elf $(OTA_DIR)/slot-b.elf: $(OBJECTS) firmware/stm32f407zg.ld
	@mkdir -p $(dir $@)
	$(CC) $(CPU) --specs=nano.specs --specs=nosys.specs -Wl,--defsym,OTA_ORIGIN=$(OTA_ORIGIN),--gc-sections -Tfirmware/stm32f407zg.ld $(OBJECTS) -Wl,--start-group -lc -lm -Wl,--end-group -o $@

$(OTA_DIR)/%.hex: $(OTA_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(OTA_DIR)/%.bin: $(OTA_DIR)/%.elf
	$(OBJCOPY) -O binary $< $@

$(OTA_DIR)/meta-a.hex: $(OTA_DIR)/slot-a.bin tools/ota_manifest.py firmware/ota/layout.h
	python3 tools/ota_manifest.py a 1 $< $@ --confirmed

$(OTA_DIR)/meta-b.hex: $(OTA_DIR)/slot-b.bin tools/ota_manifest.py firmware/ota/layout.h
	python3 tools/ota_manifest.py b 2 $< $@

MONOCYPHER = Libraries/Monocypher
OTA_SIGNED_DIR = $(OTA_DIR)/signed
.PHONY: ota-signed
ota-signed: $(OTA_DIR)/slot-a.hex $(OTA_DIR)/slot-b.hex $(OTA_DIR)/slot-a.bin $(OTA_DIR)/slot-b.bin
	@test -n "$(OTA_PUBLIC_KEY)" && test -n "$(OTA_SIGN_KEY)" || { echo 'Set OTA_PUBLIC_KEY and OTA_SIGN_KEY to Ed25519 PEM files'; exit 2; }
	@mkdir -p $(OTA_SIGNED_DIR)
	python3 tools/ota_public_key.py "$(OTA_PUBLIC_KEY)" $(OTA_SIGNED_DIR)/ota_public_key.h --match-private "$(OTA_SIGN_KEY)"
	$(CC) $(CPPFLAGS) $(filter-out -mfpu=% -mfloat-abi=%,$(CPU)) -mfloat-abi=soft -DOTA_SIGNED_BOOT -I$(OTA_SIGNED_DIR) -I$(MONOCYPHER) -Os -ffreestanding -ffunction-sections -fdata-sections --specs=nano.specs --specs=nosys.specs -nostartfiles -Wl,--gc-sections,-T,firmware/ota/boot.ld firmware/ota/boot.S firmware/ota/boot.c firmware/ota/auth.c $(MONOCYPHER)/monocypher.c $(MONOCYPHER)/monocypher-ed25519.c -o $(OTA_SIGNED_DIR)/boot.elf
	$(OBJCOPY) -O ihex $(OTA_SIGNED_DIR)/boot.elf $(OTA_SIGNED_DIR)/boot.hex
	python3 tools/ota_manifest.py a 1 $(OTA_DIR)/slot-a.bin $(OTA_SIGNED_DIR)/meta-a.hex --confirmed --sign-key "$(OTA_SIGN_KEY)"
	python3 tools/ota_manifest.py b 2 $(OTA_DIR)/slot-b.bin $(OTA_SIGNED_DIR)/meta-b.hex --sign-key "$(OTA_SIGN_KEY)"
