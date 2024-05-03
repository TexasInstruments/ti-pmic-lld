# Copyright (c) 2024, Texas Instruments Incorporated
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# *  Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# *  Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# *  Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ifdef OS
RM = rm
RMDIR = rmdir
MKDIR = mkdir
unix_to_dos = $(subst /,\,$1)
OSP = windows
else
ifeq ($(shell uname), Linux)
RM = rm -f
RMDIR = rm -rf
MKDIR = mkdir -p
OSP = unix
else
RM = rm -f
RMDIR = rm -rf
MKDIR = mkdir -p
OSP = macos
ifeq ($(CC),)
CC := clang
endif
endif
endif

ifeq ($(CC),)
CC := gcc
endif

INC	= -I./include

CFLAGS = -fPIC $(INC) -Wall
LDFLAGS = -shared
LIB_DIR = lib

DEBUGFLAGS = -O0 -g -D _DEBUG
RELEASEFLAGS = -O2 -D NDEBUG
build ?= release

ifeq ($(build), debug)
    CFLAGS += $(DEBUGFLAGS)
    TARGET = $(LIB_DIR)/pmic_lib_$(build).so
endif

ifeq ($(build), release)
    CFLAGS += $(RELEASEFLAGS)
    TARGET = $(LIB_DIR)/pmic_lib_$(build).so
endif

SOURCES = src/pmic_core.c \
	src/pmic_esm.c \
	src/pmic_fsm.c \
	src/pmic_gpio.c \
	src/pmic_ilim.c \
	src/pmic_io.c \
	src/pmic_irq.c \
	src/pmic_low_iq_timer.c \
	src/pmic_power.c \
	src/pmic_sw_shutdown.c \
	src/pmic_wdg.c

OBJECTS = $(SOURCES:.c=.o)

.PHONY: docs clean help

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(MKDIR) $(LIB_DIR)
	$(call $(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS))
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS)
	$(RM) $(OBJECTS)

docs:
	doxygen docs/ti.doxyfile

help:
	@echo "# make help"
	@echo "# ------------------------------------------------------"
	@echo "#  make [OPTIONAL MAKE VARIABLES] Note: use gmake for windows"
	@echo "Supported targets: "
	@echo "------------------"
	@echo "all            : Builds the library"
	@echo "clean          : Cleans the library"
	@echo "Optional make variables:"
	@echo "------------------------"
	@echo "CC=[Cross-compiler to be used]"
	@echo "    Default: gcc"
	@echo "BUILD_PROFILE=[release debug]"
	@echo "    Default: release"
	@echo "OS=[Windows_NT linux]"
	@echo "    Default: Windows_NT"

clean:
	$(RM) $(TARGET)
	$(RM) $(OBJECTS)
	$(RMDIR) docs/html/
	$(RMDIR) $(LIB_DIR)
