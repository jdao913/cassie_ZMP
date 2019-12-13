# Build for linux by default
PLATFORM := LINUX

# Compilation settings
INC     := -Iinclude -I$${HOME}/.mujoco/mujoco200_linux/include
CFLAGS  := -std=gnu11 -Wall -Wextra -O3 -march=sandybridge -flto
LDFLAGS := -L. -L$${HOME}/.mujoco/mujoco200_linux/bin
LIBS    := -lcassiemujoco -lmujoco200 -lglfw3 -lGLEW -lGL -lX11 -lXi -lXrandr -lXxf86vm -lXinerama -lXcursor -lrt -lm -pthread -ldl

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
CC      := x86_64-w64-mingw32-gcc
TESTOUT := cassietest.exe
SIMOUT  := cassiesim.exe
CTRLOUT := cassiectrl.exe
else
CC      := gcc
LDFLAGS += -Wl,-rpath,'$$ORIGIN'
TESTOUT := cassietest
SIMOUT  := cassiesim
SIMOUTSTIFF := cassiesim_stiff
CTRLOUT := cassiectrl
endif

# Default target
all: zmp_test

# Normal targets
clean:
	rm -f zmp_test

zmp_test: ./zmp_test.c
	$(CC) zmp_test.c simulate.c $(INC) $(CFLAGS) $(LDFLAGS) $(LIBS) -o zmp_test
	# gcc $(INC) zmp_test.c

# Virtual targets
.PHONY: all clean


