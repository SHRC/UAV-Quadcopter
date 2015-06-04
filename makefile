.PHONY: all
.PHONY: clean
.PHONY: upload
CC=avr-gcc
MCU=atmega328
TARGET= main
PROGRAMMER=usbtiny
BOARD=m328p
CFLAGS= -mmcu=$(MCU) -Wall -Os -mcall-prologues
SOURCES= main.c include/I2C_master.c
OBJECTS= $(SOURCES:.c=.o)
HEADERS= $(SOURCES:.c=.h) include/linmath.h

INCLUDES = -I.

all: main.hex

clean:
	rm -rf *.o *.elf *.hex build/*.o build/*.elf include/*.o

upload: main.hex
	avrdude -p $(BOARD) -c $(PROGRAMMER) -e -U flash:w:main.hex -v

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o build/$(lastword $(subst /, ,$@))

$(TARGET).elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(addprefix build/, $(notdir $^)) -o build/main.elf

%.hex: %.elf
	avr-objcopy -j .text -j .data -O ihex build/$< $@
