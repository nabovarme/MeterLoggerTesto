#! /usr/bin/perl -w

use strict;
use Data::Dumper;
use Data::Hexdumper;
use Device::SerialPort;
use Digest::CRC;


my $tmp = kmpGetType();
warn hexdump(data => $tmp, suppress_warnings => 1);

my $port_obj = new Device::SerialPort('/dev/ttyUSB0') || die "$!\n";
$port_obj->baudrate(1200);
$port_obj->databits(8);
$port_obj->stopbits(2);
$port_obj->parity("none");

# send
$port_obj->write($tmp);


# receive
my $res = '';
my ($c, $s);
do {
	($c, $s) = $port_obj->read(255);
	if ($c) {
		$res .= $s;
	}
} while (ord($s) != 13);

warn hexdump(data => $res, suppress_warnings => 1);



sub kmpGetType {
	my $ctx = Digest::CRC->new(width=>16, init=>0x0000, xorout=>0x0000, 
							refout=>0, poly=>0x1021, refin=>0, cont=>0);
	
	my $start_byte = chr(0x80);
	my $dst = chr(0x3f);
	my $cid = chr(0x01);
	my $stop_byte = chr(0x0d);
	
	my $i;
	my $data = $dst . $cid;
	for ($i = 0; $i < length($data); $i++) {
		$ctx->add(substr($data, $i, 1));
	}
	my $calculated_crc = $ctx->digest;
	my $calculated_crc_low = chr($calculated_crc & 0xff);
	my $calculated_crc_high = chr($calculated_crc >> 8);
	return $start_byte . $dst . $cid . $calculated_crc_high . $calculated_crc_low . $stop_byte;
}

sub kmpGetSerialNo {
	
}

sub kmpSetClock {
	
}

sub kmpGetRegister {
	
}

sub kmpPutRegister {
	
}


sub kmpByteStuff {
	my $i;
	my $data = shift;
	my $res = '';
	for ($i = 0; $i < length($data); $i++) {
		if (substr($data, $i, 1) eq chr(0x80)) {
			warn "0x80 stuffed";
			$res .= chr(0x1b) . chr(0x7f);
		}
		elsif (substr($data, $i, 1) eq chr(0x40)) {
			warn "0x40 stuffed";
			$res .= chr(0x1b) . chr(0xbf);
		}
		elsif (substr($data, $i, 1) eq chr(0x0d)) {
			warn "0x0d stuffed";
			$res .= chr(0x1b) . chr(0xf2);
		}
		elsif (substr($data, $i, 1) eq chr(0x06)) {
			warn "0x06 stuffed";
			$res .= chr(0x1b) . chr(0xf9);
		}
		elsif (substr($data, $i, 1) eq chr(0x1b)) {
			warn "0x1b stuffed";
			$res .= chr(0x1b) . chr(0xe4);
		}
		else {
			$res .= substr($data, $i, 1);
		}
	}
	return $res;
}

sub kmpByteUnstuff {
	
}

1;
