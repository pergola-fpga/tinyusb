MCU_DIR = hw/mcu/nxp/sdk/devices/MIMXRT1011

APP_START_ADDRESS ?= 0x6000C000
BOARD_FLASH_BASE ?= 0x60000000

CFLAGS += \
  -mthumb \
  -mabi=aapcs \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv5-d16 \
  -D__ARMVFP__=0 -D__ARMFPV5__=0\
  -D__START=main \
  -DCFG_TUSB_MCU=OPT_MCU_MIMXRT10XX \
  -D__STARTUP_CLEAR_BSS \
  -DCPU_MIMXRT1011DAE5A \
  -DXIP_EXTERNAL_FLASH=1 \
  -DXIP_BOOT_HEADER_ENABLE=1 \
  -DCFG_TUSB_MCU=OPT_MCU_MIMXRT10XX

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=implicit-fallthrough=

CFLAGS += \
	-DAPP_START_ADDRESS=$(APP_START_ADDRESS) \
	-DBOARD_FLASH_BASE=$(BOARD_FLASH_BASE) \
	-DUF2_FAMILY=0x4FB2D5BD


SRC_C += \
	$(MCU_DIR)/system_MIMXRT1011.c \
	$(MCU_DIR)/xip/fsl_flexspi_nor_boot.c \
	$(MCU_DIR)/project_template/clock_config.c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(MCU_DIR)/drivers/fsl_cache.c \
	$(MCU_DIR)/drivers/fsl_gpio.c \
	$(MCU_DIR)/drivers/fsl_flexspi.c \
	$(MCU_DIR)/drivers/fsl_common.c \
	$(MCU_DIR)/drivers/fsl_lpuart.c \
	$(MCU_DIR)/drivers/fsl_ocotp.c

INC += \
	$(TOP)/hw/bsp/$(BOARD) \
	$(TOP)/$(MCU_DIR)/../../CMSIS/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(MCU_DIR)/project_template \

SRC_S += $(MCU_DIR)/gcc/startup_MIMXRT1011.S

LD_FILE = hw/bsp/pergola/MIMXRT1011xxxxx_flexspi_nor_app.ld

# For TinyUSB port source
VENDOR = nxp
CHIP_FAMILY = transdimension

# For freeRTOS port source
FREERTOS_PORT = ARM_CM7

# For flash-jlink target
JLINK_DEVICE = MIMXRT1011DAE5A
JLINK_IF = swd

# flash using hf2 bootloader
flash-hf2: $(BUILD)/$(BOARD)-firmware.bin
	RUST_LOG=debug RUST_BACKTRACE=full hf2 -v 0x239a -p 0x0058 flash --address $(APP_START_ADDRESS) --file $^

# flash using redlink
flash: $(BUILD)/$(BOARD)-firmware.elf
	$(CRT_EMU_CM_REDLINK) \
			-g  \
			-p MIMXRT1011xxxxx \
			--flash-load-exec $(BUILD)/$(BOARD)-firmware.elf \
			--connectscript=RT1010_connect.scp \
			--debug 4 \
			--no-packed

# debug without flashing
debug: $(BUILD)/$(BOARD)-firmware.elf
	arm-none-eabi-gdb $< \
		-ex "target extended-remote | $(CRT_EMU_CM_REDLINK) -g -pMIMXRT1011xxxxx --connectscript=RT1010_connect.scp --no-packed"
