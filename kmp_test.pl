#! /opt/local/bin/perl -w

use strict;
use Data::Dumper;
use Digest::CRC;

#my @t = qw[3f 01];
#my @crc = qw[05 8a];

my $tmp = kmpGetType();
for (0..(length($tmp) - 1)) {
	print sprintf("%02x", ord(substr($tmp, $_, 1)));
}
print "\n";

$tmp = kmpByteStuff(chr(0x04) . chr(0x0d) . chr(0x00) . chr(0x06));
for (0..(length($tmp) - 1)) {
	print sprintf("%02x", ord(substr($tmp, $_, 1)));
}
print "\n";

sub kmpGetType {
	my $ctx = Digest::CRC->new(width=>16, init=>0x0000, xorout=>0x0000, 
							refout=>0, poly=>0x1021, refin=>0, cont=>0);
	
	my $start_byte = chr(0x80);
	my $dst = chr(0x3f);
	my $cid = chr(0x01);
	
	my $i;
	my $data = $dst . $cid;
	for ($i = 0; $i < length($data); $i++) {
		$ctx->add(substr($data, $i, 1));
	}
	my $calculated_crc = $ctx->digest;
	my $calculated_crc_low = chr($calculated_crc & 0xff);
	my $calculated_crc_high = chr($calculated_crc >> 8);
	return $start_byte . $dst . $cid . $calculated_crc_high . $calculated_crc_low;
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
