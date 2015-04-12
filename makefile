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
	$(CC) -o quadcontrol.elf quadcontrol.o
	rm -f quadcontrol.hex
	avr-objcopy -j .text -j .data -O ihex quadcontrol.elf quadcontrol.hex
	avr-size --format=avr --mcu=$(MCU) quadcontrol.elf

clean:
	rm -rf *.o *.elf *.hex objects/*.o

upload:
	avrdude -p $(BOARD) -c $(PROGRAMMER) -e -U flash:w:quadcontrol.hex

quadcontrol.o: main.o
	$(CC) $(CFLAGS) $(INCLUDES) -c main.c -o quadcontrol.o

%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
