EXECUTABLES=neurotic reset backwards

PLATFORM=-mcpu=cortex-a8 -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8
OPTIMIZE=-O2 -ffast-math
WARN=-Wall -Wextra
CFLAGS=-std=gnu99 $(OPTIMIZE) $(PLATFORM) $(WARN) 
LDFLAGS=
LDLIBS=-lrt -lpruio 


all : $(EXECUTABLES)

neurotic.o : neurotic.h
neurotic.h : cpg-compiler.py
	python3 $< 'SingleCPG()' > $@

$(EXECUTABLES) : libneurobot.o

.PHONY : clean
clean :
	-@rm $(EXECUTABLES) neurotic.h *.o tags

tags :
	ctags -R .

%.o : tags
