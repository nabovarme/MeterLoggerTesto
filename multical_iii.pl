#! /opt/bin/perl -w

use strict;
use Data::Dumper;
use Data::Hexdumper;
use Device::SerialPort;
use Digest::CRC;

my $port_obj;

#my $port_obj = new Device::SerialPort('/dev/ttyUSB0') || die "$!\n";
$port_obj = new Device::SerialPort('/dev/tty.usbserial-A6YNEE07') || die "$!\n";
$port_obj->baudrate(300);
$port_obj->databits(7);
$port_obj->stopbits(2);
$port_obj->parity('even');

# send
$port_obj->write('/#C');

# receive
#$port_obj->baudrate(1200);
#$port_obj->stopbits(1);
my $res = '';
my ($c, $s);
do {
	($c, $s) = $port_obj->read(1);
	if ($c) {
		$res .= $s;
		warn hexdump(data => $res, suppress_warnings => 1);
	}
} while (1);
#} while (ord($s) != 0x0d);

#warn hexdump(data => kmpByteUnstuff($res), suppress_warnings => 1);

1;