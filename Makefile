# Seymour - Feedback Mixer with Safety Limiter
# Makefile for distingNT plugin

# Plugin name
PLUGIN_NAME = seymour

# Source files
SOURCES = src/seymour.cpp

# distingNT API location
API_DIR = distingNT_API/include

# Output directories
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# ARM toolchain (for hardware)
ARM_PREFIX = arm-none-eabi-
ARM_CC = $(ARM_PREFIX)gcc
ARM_CXX = $(ARM_PREFIX)g++
ARM_OBJCOPY = $(ARM_PREFIX)objcopy

# Desktop toolchain (for testing with nt_emu)
CXX = clang++

# ARM Cortex-M7 flags
ARM_ARCH_FLAGS = -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard

# Common flags
COMMON_FLAGS = -I$(API_DIR) -fno-exceptions -fno-rtti -fno-threadsafe-statics
COMMON_FLAGS += -Wall -Wextra -Wno-unused-parameter

# ARM-specific flags (hardware build)
ARM_CXXFLAGS = $(ARM_ARCH_FLAGS) $(COMMON_FLAGS) -Os -ffunction-sections -fdata-sections
ARM_LDFLAGS = $(ARM_ARCH_FLAGS) -nostartfiles -Wl,--gc-sections

# Desktop flags (testing build)
# Note: NT_globals, NT_intToString etc are provided by nt_emu at runtime
ifeq ($(shell uname),Darwin)
    # macOS - use dynamic_lookup for runtime symbol resolution
    DESKTOP_CXXFLAGS = $(COMMON_FLAGS) -O2 -fPIC -std=c++11
    DESKTOP_LDFLAGS = -dynamiclib -undefined dynamic_lookup
    DESKTOP_EXT = .dylib
else ifeq ($(OS),Windows_NT)
    # Windows
    DESKTOP_CXXFLAGS = $(COMMON_FLAGS) -O2 -fPIC -std=c++11
    DESKTOP_LDFLAGS = -shared
    DESKTOP_EXT = .dll
else
    # Linux - allow undefined symbols for runtime resolution
    DESKTOP_CXXFLAGS = $(COMMON_FLAGS) -O2 -fPIC -std=c++11
    DESKTOP_LDFLAGS = -shared -Wl,--allow-shlib-undefined
    DESKTOP_EXT = .so
endif

# Targets
.PHONY: all hardware test clean help

all: hardware

# Hardware build (for distingNT SD card)
hardware: $(BUILD_DIR)/$(PLUGIN_NAME).o
	@echo "Hardware build complete: $(BUILD_DIR)/$(PLUGIN_NAME).o"
	@echo "Copy this file to your distingNT SD card"

$(BUILD_DIR)/$(PLUGIN_NAME).o: $(SOURCES) | $(BUILD_DIR)
	$(ARM_CXX) $(ARM_CXXFLAGS) -c $(SOURCES) -o $@

# Desktop build (for nt_emu testing)
test: $(BUILD_DIR)/$(PLUGIN_NAME)$(DESKTOP_EXT)
	@echo "Test build complete: $(BUILD_DIR)/$(PLUGIN_NAME)$(DESKTOP_EXT)"
	@echo "Copy this file to your VCV Rack nt_emu plugins folder"

$(BUILD_DIR)/$(PLUGIN_NAME)$(DESKTOP_EXT): $(SOURCES) | $(BUILD_DIR)
	$(CXX) $(DESKTOP_CXXFLAGS) $(SOURCES) -o $@ $(DESKTOP_LDFLAGS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "Seymour - distingNT Plugin Build"
	@echo ""
	@echo "Targets:"
	@echo "  make hardware  - Build for distingNT hardware (.o file)"
	@echo "  make test      - Build for desktop testing with nt_emu"
	@echo "  make clean     - Remove build artifacts"
	@echo ""
	@echo "Requirements:"
	@echo "  Hardware: arm-none-eabi-gcc toolchain"
	@echo "  Testing:  clang++ or g++, VCV Rack with nt_emu module"
	@echo ""
	@echo "API Setup:"
	@echo "  git submodule add https://github.com/expertsleepersltd/distingNT_API.git"
