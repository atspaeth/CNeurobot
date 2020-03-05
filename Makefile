CPGS=neurotic # backwards
EXECUTABLES=reset $(CPGS)

PLATFORM=-mcpu=cortex-a8 -mfloat-abi=hard -mfpu=neon -mtune=cortex-a8
OPTIMIZE=-O2 -ffast-math
WARN=-Wall -Wextra
CFLAGS=-std=gnu99 $(OPTIMIZE) $(PLATFORM) $(WARN) 
LDFLAGS=
LDLIBS=-lrt -lpruio 

CPGHEADERS=$(addsuffix .h,$(CPGS))
CPGOFILES=$(addsuffix .o,$(CPGS))
GENFILES=$(EXECUTABLES) $(wildcard *.o) tags $(CPGHEADERS)

all : $(EXECUTABLES)

$(CPGOFILES): %.o : %.h
$(CPGHEADERS): %.h : %.py
	python3 $< > $@

$(EXECUTABLES) : libneurobot.o

.PHONY : clean
clean :
	-@rm $(GENFILES)

tags :
	ctags -R .

%.o : tags
