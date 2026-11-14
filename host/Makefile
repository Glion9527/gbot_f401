# 宿主骨架本机构建（macOS/Linux，无需 CMake）。
# 树莓派上也可以用：make && ./build/f401_host --dry-run
# 如果装了 CMake，也可以用 CMakeLists.txt（更适合树莓派/后续 colcon 集成）。

CXX ?= c++
CC ?= cc
CXXFLAGS ?= -std=c++20 -O2 -g -Wall -Wextra -Wpedantic
CFLAGS ?= -std=c11 -O2 -g -Wall -Wextra -Wpedantic
CPPFLAGS := -Iinclude -I../protocol -Itests
BUILD := build

LIB_SOURCES := src/drivers/serial_port.cpp src/drivers/link_v2.cpp src/drivers/serial_base.cpp
LIB_OBJECTS := $(patsubst src/%.cpp,$(BUILD)/%.o,$(LIB_SOURCES))
APP_OBJECT := $(BUILD)/app/main.o
TEST_OBJECTS := $(BUILD)/tests/codec_test.o $(BUILD)/tests/core_test.o \
                $(BUILD)/tests/serial_port_test.o
TEST_BINS := $(BUILD)/codec_test $(BUILD)/core_test $(BUILD)/serial_port_test

.PHONY: all test sanitize clean

all: $(BUILD)/f401_host

$(BUILD)/%.o: src/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/tests/%.o: tests/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/f401_host: $(LIB_OBJECTS) $(APP_OBJECT)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/codec_test: $(BUILD)/tests/codec_test.o $(LIB_OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/core_test: $(BUILD)/tests/core_test.o $(LIB_OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@

$(BUILD)/serial_port_test: $(BUILD)/tests/serial_port_test.o $(LIB_OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@

# 共享头文件必须能用 C11 编译，避免变成“只能 C++ 用”。
$(BUILD)/c_header_test: tests/c_header_test.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -I../protocol $< -o $@

test: $(TEST_BINS) $(BUILD)/c_header_test
	./$(BUILD)/codec_test
	./$(BUILD)/core_test
	./$(BUILD)/serial_port_test
	./$(BUILD)/c_header_test

# 可选：ASan/UBSan 构建，专门抓泄漏/越界/未定义行为。
# 用独立目录，避免和普通构建的对象文件混在一起。
sanitize:
	$(MAKE) BUILD=build/sanitize CXXFLAGS="-std=c++20 -O1 -g -Wall -Wextra -Wpedantic \
		-fsanitize=address,undefined -fno-omit-frame-pointer" test

clean:
	rm -rf $(BUILD)

-include $(LIB_OBJECTS:.o=.d) $(APP_OBJECT:.o=.d) $(TEST_OBJECTS:.o=.d)
