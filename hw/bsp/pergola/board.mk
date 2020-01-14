CFLAGS += \
  -mthumb \
  -mapcs \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv5-d16 \
  -D__ARMVFP__=0 -D__ARMFPV5__=0\
  -DCPU_MIMXRT1011DAE5A \
  -DDEBUG \
  -D__START=main \
  -DXIP_EXTERNAL_FLASH=1 \
  -DXIP_BOOT_HEADER_ENABLE=1 \
  -DCFG_TUSB_MCU=OPT_MCU_MIMXRT10XX \
  -D__STARTUP_CLEAR_BSS

# mcu driver cause following warnings
CFLAGS += -Wno-error=unused-parameter -Wno-error=implicit-fallthrough=

MCU_DIR = hw/mcu/nxp/sdk/devices/MIMXRT1011

# All source paths should be relative to the top level.
LD_FILE = hw/bsp/$(BOARD)/MIMXRT1011xxxxx_flexspi_nor.ld

SRC_C += \
	$(MCU_DIR)/system_MIMXRT1011.c \
	$(MCU_DIR)/xip/fsl_flexspi_nor_boot.c \
	$(MCU_DIR)/project_template/clock_config.c \
	$(MCU_DIR)/drivers/fsl_clock.c \
	$(MCU_DIR)/drivers/fsl_gpio.c \
	$(MCU_DIR)/drivers/fsl_flexspi.c \
	$(MCU_DIR)/drivers/fsl_cache.c \
	$(MCU_DIR)/drivers/fsl_common.c \
	$(MCU_DIR)/drivers/fsl_lpuart.c

INC += \
	$(TOP)/hw/bsp/$(BOARD) \
	$(TOP)/$(MCU_DIR)/../../CMSIS/Include \
	$(TOP)/$(MCU_DIR) \
	$(TOP)/$(MCU_DIR)/drivers \
	$(TOP)/$(MCU_DIR)/project_template \

SRC_S += $(MCU_DIR)/gcc/startup_MIMXRT1011.S

# For TinyUSB port source
VENDOR = nxp
CHIP_FAMILY = transdimension

# For freeRTOS port source
FREERTOS_PORT = ARM_CM7

# For flash-jlink target
JLINK_DEVICE = MIMXRT1011DAE5A
JLINK_IF = swd

# flash by using redlink
flash: $(BUILD)/$(BOARD)-firmware.elf
	$(CRT_EMU_CM_REDLINK) \
			-g  \
			-p MIMXRT1011xxxxx \
			--flash-load-exec $(BUILD)/$(BOARD)-firmware.elf \
			--connectscript=RT1010_connect.scp \
			--debug 4 \
			--no-packed
