CXX = arm-none-eabi-g++
OBJCOPY = arm-none-eabi-objcopy
CXXFLAGS = -mcpu=cortex-m3 -mthumb -Wall -fno-exceptions -fno-rtti -std=gnu++11
LDFLAGS = -T $(wildcard Linker/*.ld) -nostartfiles


BUILD_DIR = bin
TARGET = $(BUILD_DIR)/main

C_SOURCES = $(wildcard myCppLib/src/*.cpp)
S_SOURCES = $(wildcard Startup/*.s)

OBJ = $(patsubst %.cpp,$(BUILD_DIR)/%.o,$(C_SOURCES))
OBJ += $(patsubst %.s,$(BUILD_DIR)/%.o,$(S_SOURCES))

all: $(TARGET).hex
	@echo Outputs created!

$(TARGET).hex: $(TARGET).elf
	@$(OBJCOPY) -O ihex $< $@

$(TARGET).elf: $(OBJ)
	@$(CXX) $(CXXFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD_DIR)/%.o: %.cpp
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s				
	@if not exist "$(subst /,\,$(@D))" mkdir "$(subst /,\,$(@D))"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	@if exist "$(subst /,\,$(BUILD_DIR))" rmdir /s /q "$(subst /,\,$(BUILD_DIR))"
	@echo Outputs cleared!

.PHONY: all clean