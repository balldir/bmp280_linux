# Copyright (c) 2016 Max Asaulov

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to 
# deal in the Software without restriction, including without limitation the 
# rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
# sell copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in 
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
# IN THE SOFTWARE.

# Project name
NAME := bmp280_linux

# Echo
NOECHO ?= @

# Base pas
SOURCE := .
BUILD := build

# Files
objs :=  \
        $(patsubst %.c,%.o,$(wildcard *.c)) \
        BMP280_driver/bmp280.o\

IPATH += -Iinclude/ -IBMP280_driver
LIBS += -lm

# Toolchain config
CC := gcc
LD := ld

# Options

WARN_FLAGS = -Wall -Wextra
              
CFLAGS := -Os -std=gnu11 $(WARN_FLAGS) $(CONFIG_DEFINES) $(IPATH) -g

##########################################################################
OBJS := $(addprefix $(BUILD)/, $(objs))
DIR_LIST := $(dir $(OBJS))

#Rules
all : dir $(BUILD)/$(NAME).elf 
.PHONY : all

show_vars: ## Show all env variables
	$(NOECHO) $(foreach v, $(.VARIABLES), $(info $(v) = $($(v))))

print-%: ## Print variable
	$(NOECHO) $* = $($*)

.PHONY : dir
dir: ## Generate dir tree
	$(NOECHO) echo "Creating dir tree $< ..."
	$(NOECHO) mkdir -p $(DIR_LIST)

$(BUILD)/%.o: %.c ## Generate object files from c source files
	$(NOECHO) echo "Compile $< ..."
	$(NOECHO) $(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/$(NAME).elf: $(OBJS) ## Generate elf $(Name)
	$(NOECHO) echo "Link $@ ..."
	$(NOECHO) $(CC) $(CFLAGS) $(LIBS) -o $@ $^ 

clean: ## Remove build artifacts
	$(NOECHO) echo "Clean up ..."
	$(NOECHO) rm -rf $(BUILD)
