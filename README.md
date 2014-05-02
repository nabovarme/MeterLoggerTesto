MeterLogger
===========

Get data via IR and log it to iOS device via SoftModem FSK.
https://code.google.com/p/arms22/source/browse/#svn%2Ftrunk%2FSoftModem

Testo protocol
--------------
IR Printer emulator for Testo flue gas analysator <http://www.testousa.com/combustion/testo-310-residential-combustion-analyzer.html>.

Coded for pic18f2550 compiled with SDCC

send data at 19200 bps over serial

protocol http://members.ziggo.nl/kees.van.der.sanden/downloads/82240bte.pdf (HP-IR, HP48)

http://www.keesvandersanden.nl/calculators/hp82240.php

Video showing "printing": https://www.youtube.com/watch?v=RM3XE1EymNQ


Kamstrup Meter Protocol
-----------------------
http://kamstrup.com/media/19757/file.pdf

The protocol is based on half duplex serial asynchronous communication with the setup: 8 databits, no parity and 2 stopbits. The data bit rate is 1200 or 2400 baud. CRC16 is used in both request and response.

http://ing.dk/blog/tal-med-din-elmaaler-og-ingenioeren-120524

https://github.com/bsdphk/PyDLMS/blob/master/dlms.py