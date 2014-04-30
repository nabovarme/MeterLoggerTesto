PK2CMD = pk2cmd -PPIC18F2550
PROCESSOR = 18f2550
#PROCESSOR = 18f2455
MODEL = pic16
GPSIM_PROCESSOR = 18f2455
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
	-p$(PROCESSOR) \
	--obanksel=2 \
	$<
#	-I"./" glcd.o menu_system.o \
#	-Wl '-m -s18f2550_g.lkr' \
#	-Wa '-g' \
#	--debug \

fifo_test: fifo_test.c
	$(SDCC) \
	--verbose \
	-V \
	-m$(MODEL) \
	--use-crt=crt0.o \
	--use-non-free \
	-p$(PROCESSOR) \
	--debug-info \
	$<
#	-I"./" glcd.o menu_system.o \
#	-Wl '-m -s18f2550_g.lkr' \


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

flash_fifo_test:
	$(PK2CMD) -F fifo_test.hex -M

on:
	$(PK2CMD) -R -T

off:
	$(PK2CMD) -R

