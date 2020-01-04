include ../../../tools/top.mk
include ../../make.mk

INC += \
	src \
	$(TOP)/hw \
	$(BUILD)

# Example source
EXAMPLE_SOURCE += $(wildcard src/*.c)
SRC_C += $(addprefix $(CURRENT_PATH)/, $(EXAMPLE_SOURCE))

UF2_VERSION_BASE = $(shell git describe --always --tags)

$(BUILD)/uf2_version.h: Makefile
	echo "#define UF2_VERSION_BASE \"$(UF2_VERSION_BASE)\""> $@

OBJ += $(BUILD)/uf2_version.h

include ../../rules.mk
