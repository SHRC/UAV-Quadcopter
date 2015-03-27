CC=avr-gcc
MCU=atmega328
TARGET= blink.c
PROGRAMMER=avrisp2
CFLAGS= -mmcu=$(MCU) -Wall -Os -mcall-prologues

DEPS = i2csoft.h io.h delay.h linmath.h
OBJ = blink.o

%.o: %.c $(DEPS)
	$(CC) -o $@ $^ $(CFLAGS)

all: $(OBJ)
	avr-gcc $(CFLAGS) -o quadcontrol.o $(TARGET)
	avr-gcc -o quadcontrol.elf quadcontrol.o
	rm -f quadcontrol.hex
	avr-objcopy -j .text -j .data -O ihex quadcontrol.elf quadcontrol.hex
	avr-size --format=avr --mcu=$(MCU) quadcontrol.elf

clean:
	rm -f *.o *.elf *.hex

upload:
	avrdude -p $(MCU) -c $(PROGRAMMER) -e -U flash:w:quadcontrol.hex