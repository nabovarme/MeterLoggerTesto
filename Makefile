PK2CMD = pk2cmd -PPIC18F4685
PROCESSOR = 18f4685
#PROCESSOR = 18f4620
MODEL = pic16
GPSIM_PROCESSOR = pic18f4620
SDCC = /opt/local/bin/sdcc
GPLINK = /opt/local/bin/gplink
GPASM = /opt/local/bin/gpasm
GPSIM =/usr/local/bin/gpsim

all:	testo_printer_emulator

testo_printer_emulator: testo_printer_emulator.c
	$(SDCC) \
	--verbose \
	-V \
	-m$(MODEL) \
	--use-crt=crt0.o \
	--use-non-free \
	-Wl '-m' \
	-p$(PROCESSOR) \
	--debug-info \
	$<
#	-Wl '-m -s18f4620.lkr' \
#	-I"./" glcd.o menu_system.o \


testo_printer_emulator.hex: testo_printer_emulator.o
	$(GPLINK) \
	-c \
	-o $@ \
	-m \
	-r \
	-d \
	testo_printer_emulator testo_printer_emulator.o crt0.o \
	$^

testo_printer_emulator.o: testo_printer_emulator.asm
	$(GPASM) \
	--extended \
	-pp$(PROCESSOR) \
	-c $<

testo_printer_emulator.asm: testo_printer_emulator.c
	$(SDCC) \
	-V \
	--verbose \
	-S \
	--debug \
	-m$(MODEL) \
	--use-crt=crt0.o \
	--use-non-free \
	-p$(PROCESSOR) $<

clean:
	rm -f *.adb *.asm *.cod *.cof *.hex *.lst *.map *.o *.sym *.lib

sim:
	$(GPSIM) -p$(GPSIM_PROCESSOR) -c testo_printer_emulator.stc -s testo_printer_emulator.cod
	# && killall -9 X11.bin

flash:
	$(PK2CMD) -F testo_printer_emulator.hex -M

flash_erase:
	$(PK2CMD) -E

flash_master:
	$(PK2CMD) -F testo_printer_emulator_master.hex -M

on:
	$(PK2CMD) -R -T

off:
	$(PK2CMD) -R

