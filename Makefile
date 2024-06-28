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

CFLAGS = -fPIC $(INC) -Weverything \
				 -Wno-documentation \
				 -Wno-padded \
				 -Wno-poison-system-directories
LDFLAGS = -shared
LIB_DIR = lib

DEBUGFLAGS = -O0 -g -D _DEBUG
RELEASEFLAGS = -O2 -D NDEBUG
build ?= release

ifeq ($(build), debug)
    CFLAGS += $(DEBUGFLAGS)
    TARGET = $(LIB_DIR)/libpmic.so
endif

ifeq ($(build), release)
    CFLAGS += $(RELEASEFLAGS)
    TARGET = $(LIB_DIR)/libpmic.so
endif

SOURCES = \
	src/pmic.c \
	src/pmic_core.c \
	src/pmic_io.c \
	src/pmic_irq.c \
	src/pmic_power.c \
	src/pmic_wdg.c

OBJECTS = $(SOURCES:.c=.o)

.PHONY: docs clean

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(MKDIR) $(LIB_DIR)
	$(call $(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS))
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS)
	$(RM) $(OBJECTS)

docs:
	doxygen docs/ti.doxyfile

clean:
	$(RM) $(TARGET)
	$(RM) $(OBJECTS)
	$(RMDIR) docs/html/
	$(RMDIR) $(LIB_DIR)
