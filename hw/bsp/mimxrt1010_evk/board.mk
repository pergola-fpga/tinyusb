CFLAGS += \
  -DCPU_MIMXRT1011DAE5A \
  -DXIP_EXTERNAL_FLASH=1 \
  -DXIP_BOOT_HEADER_ENABLE=1 \
  -D__START=main \
  -D__ARMVFP__ -D__ARMFPV5__\
  -mthumb \
  -mabi=aapcs-linux \
  -mcpu=cortex-m7 \
  -mfloat-abi=hard \
  -mfpu=fpv4-sp-d16 \
  -fno-short-enums \
  -nostdlib -nostartfiles \
  -DCFG_TUSB_MCU=OPT_MCU_RT10XX

# Yeah, the SDK is not perfect, but we don't want to modify the files
CFLAGS += -Wno-error=undef -Wno-unused-parameter -Wno-nested-externs

# All source paths should be relative to the top level.
LD_FILE = hw/mcu/nxp/rt10xx/devices/MIMXRT1011/gcc/MIMXRT1011xxxxx_flexspi_nor.ld

SDK := $(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011

SRC_C += \
	$(SDK)/system_MIMXRT1011.c \
	$(SDK)/xip/fsl_flexspi_nor_boot.c \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011/drivers/fsl_lpuart.c \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011/drivers/fsl_gpio.c \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011/drivers/fsl_clock.c \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011/project_template/clock_config.c \
	$(TOP)/src/portable/$(VENDOR)/$(CHIP_FAMILY)/hal_$(CHIP_FAMILY).c

SRC_S += \
	$(SDK)/gcc/startup_MIMXRT1011.S

INC += \
	$(TOP)/hw/mcu/nxp/rt10xx/CMSIS/Include \
	$(TOP)/hw/bsp/mimxrt1010_evk \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011/drivers \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011 \
	$(TOP)/hw/mcu/nxp/rt10xx/devices/MIMXRT1011/project_template

# For TinyUSB port source
VENDOR = nxp
CHIP_FAMILY = rt10xx

# For freeRTOS port source
#FREERTOS_PORT = ARM_CM4F

# For flash-jlink target
#JLINK_DEVICE = ATSAMD51J19
#JLINK_IF = swd

# flash using jlink
#flash: flash-jlink
