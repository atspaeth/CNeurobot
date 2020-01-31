PLATFORM=-mcpu=cortex-a8 -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8
OPTIMIZE=-O2 -ffast-math
WARN=-Wall -Wextra
CFLAGS=-std=gnu99 $(OPTIMIZE) $(PLATFORM) $(WARN) 
LDFLAGS=
LDLIBS=-lrt -lpruio 

all : neurotic reset

neurotic reset : libneurobot.o
