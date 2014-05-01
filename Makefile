PK2CMD = pk2cmd -PPIC18F2550
PROCESSOR = 18f2550
#PROCESSOR = 18f2455
MODEL = pic16
GPSIM_PROCESSOR = 18f2455
SDCC = /opt/local/bin/sdcc
GPLINK = /opt/local/bin/gplink
GPASM = /opt/local/bin/gpasm
GPSIM =/usr/local/bin/gpsim

all:	meter_logger

meter_logger: meter_logger.c
	$(SDCC) \
	--verbose \
	-V \
	-m$(MODEL) \
	--use-crt=crt0.o \
	--use-non-free \
	-p$(PROCESSOR) \
	--obanksel=2 \
	-Wl '-m' \
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


meter_logger.hex: meter_logger.o
	$(GPLINK) \
	-c \
	-o $@ \
	-m \
	-r \
	-d \
	meter_logger meter_logger.o crt0.o \
	$^

meter_logger.o: meter_logger.asm
	$(GPASM) \
	--extended \
	-pp$(PROCESSOR) \
	-c $<

meter_logger.asm: meter_logger.c
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
	$(GPSIM) -p$(GPSIM_PROCESSOR) -c meter_logger.stc -s meter_logger.cod
	# && killall -9 X11.bin

flash:
	$(PK2CMD) -F meter_logger.hex -M

flash_erase:
	$(PK2CMD) -E

flash_master:
	$(PK2CMD) -F meter_logger_master.hex -M

flash_fifo_test:
	$(PK2CMD) -F fifo_test.hex -M

on:
	$(PK2CMD) -R -T

off:
	$(PK2CMD) -R

