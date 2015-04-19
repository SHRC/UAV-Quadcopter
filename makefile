.PHONY: all
.PHONY: clean
.PHONY: upload
CC=avr-gcc
MCU=atmega328
TARGET= main.c
PROGRAMMER=usbtiny
BOARD=m328p
CFLAGS= -mmcu=$(MCU) -Wall -Os -mcall-prologues

INCLUDES = -I include

all: quadcontrol.o
	$(CC) -o build/quadcontrol.elf build/quadcontrol.o
	rm -f quadcontrol.hex
	avr-objcopy -j .text -j .data -O ihex build/quadcontrol.elf quadcontrol.hex
	avr-size --format=avr --mcu=$(MCU) build/quadcontrol.elf

clean:
	rm -rf *.o *.elf *.hex build/*.o build/*.elf

upload:
	avrdude -p $(BOARD) -c $(PROGRAMMER) -e -U flash:w:quadcontrol.hex

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o build/$@

quadcontrol.o: main.o
	$(CC) $(CFLAGS) $(INCLUDES) -c main.c -o build/quadcontrol.o
