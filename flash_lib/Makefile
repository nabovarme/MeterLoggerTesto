PK2CMD = pk2cmd -PPIC18F2550
PROCESSOR = 18f2550
#PROCESSOR = 18f2455
MODEL = pic16
GPSIM_PROCESSOR = 18f2455
SDCC = /opt/local/bin/sdcc
GPLINK = /opt/local/bin/gplink
GPASM = /opt/local/bin/gpasm
GPSIM =/usr/local/bin/gpsim

all:	flash_lib

flash_lib: flash.c
	$(SDCC) \
	-g \
	--verbose \
	-V \
	--debug \
	-m$(MODEL) \
	-p$(PROCESSOR) \
	--use-non-free \
	-c $<

clean:
	rm -f *.adb *.asm *.cod *.cof *.hex *.lst *.map *.o *.sym *.lib

