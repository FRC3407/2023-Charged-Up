rwildcard = $(foreach d,$(wildcard $(addsuffix *,$(1))),$(call rwildcard,$(d)/,$(2)) $(filter $(subst *,%,$(2)),$(d)))

CROSS_PREFIX := aarch64-bullseye-linux-gnu-
CXX := g++

NAME := pixy2usb

SRC_DIR := src
OBJ_DIR := obj
OUT_DIR := out
LIB_DIR := lib
INCLUDE_DIR := include

SRCS := $(call rwildcard,$(SRC_DIR)/,*.cpp)
OBJS := $(SRCS:$(SRC_DIR)/%=$(OBJ_DIR)/%.o)
TARGET := $(OUT_DIR)/lib$(NAME).so

CPPFLAGS := -I$(INCLUDE_DIR) -MMD -MP
CFLAGS := -O3 -Wall -fpermissive -fPIC
LDFLAGS := -Wall -L$(LIB_DIR)
LDLIBS := -lusb-1.0

ifeq ($(OS),Windows_NT)
CXX := $(CROSS_PREFIX)$(CXX)
RM-R := del /s /Q
CP := copy
/ := \\

else
RM-R := rm -r
CP := cp
/ := /

endif

$(OBJ_DIR)/%.cpp.o : $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) -c -o $(OBJ_DIR)/$(@F) $(CPPFLAGS) $(CFLAGS) $<

$(TARGET): $(OBJS) | $(OUT_DIR)
	$(CXX) -shared -o $@ $(LDFLAGS) $(foreach file,$(^F),$(OBJ_DIR)/$(file)) $(LDLIBS)

$(OUT_DIR) $(OBJ_DIR):
	mkdir $@

.PHONY: build clean rebuild

build: $(TARGET)

clean:
	$(RM-R) $(OBJ_DIR)
	$(RM-R) $(OUT_DIR)

rebuild: build | clean


-include $(OBJS:.o=.d)