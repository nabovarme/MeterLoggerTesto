#! /usr/bin/perl -w

use strict;
use Data::Dumper;
use Data::Hexdumper;
use Device::SerialPort;
use Digest::CRC;


my $port_obj = new Device::SerialPort('/dev/ttyUSB0') || die "$!\n";
$port_obj->baudrate(1200);
$port_obj->databits(8);
$port_obj->stopbits(2);
$port_obj->parity("none");

kmpGetType();
kmpGetSerialNo();
#kmpSetClock({'year' => 2014,
#			 'month' => 5,
#			 'day' => 29,
#			 'hour' => 18,
#			 'min' => 40,
#			 'sec' => 57});
kmpGetRegister({'rid' => 0x98});


sub kmpGetType {
	my $ctx = Digest::CRC->new(width=>16, init=>0x0000, xorout=>0x0000, 
							refout=>0, poly=>0x1021, refin=>0, cont=>0);
	
	my $start_byte = chr(0x80);
	my $dst = chr(0x3f);
	my $cid = chr(0x01);
	my $stop_byte = chr(0x0d);
	
	# calculate crc
	my $i;
	my $data = $dst . $cid;
	for ($i = 0; $i < length($data); $i++) {
		$ctx->add(substr($data, $i, 1));
	}
	my $calculated_crc = $ctx->digest;
	my $calculated_crc_low = chr($calculated_crc & 0xff);
	my $calculated_crc_high = chr($calculated_crc >> 8);

	# send
	warn hexdump(data => $start_byte . $dst . $cid . $calculated_crc_high . $calculated_crc_low . $stop_byte, suppress_warnings => 1);
	$port_obj->write($start_byte . $dst . $cid . $calculated_crc_high . $calculated_crc_low . $stop_byte);

	# receive
	my $res = '';
	my ($c, $s);
	do {
		($c, $s) = $port_obj->read(255);
		if ($c) {
			$res .= $s;
		}
	} while (ord($s) != 0x0d);

	warn hexdump(data => kmpByteUnstuff($res), suppress_warnings => 1);
	return kmpByteUnstuff($res);
}

sub kmpGetSerialNo {
	my $ctx = Digest::CRC->new(width=>16, init=>0x0000, xorout=>0x0000, 
							refout=>0, poly=>0x1021, refin=>0, cont=>0);
	
	my $start_byte = chr(0x80);
	my $dst = chr(0x3f);
	my $cid = chr(0x02);
	my $stop_byte = chr(0x0d);
	
	# calculate crc
	my $i;
	my $data = $dst . $cid;
	for ($i = 0; $i < length($data); $i++) {
		$ctx->add(substr($data, $i, 1));
	}
	my $calculated_crc = $ctx->digest;
	my $calculated_crc_low = chr($calculated_crc & 0xff);
	my $calculated_crc_high = chr($calculated_crc >> 8);

	# send
	warn hexdump(data => $start_byte . $dst . $cid . $calculated_crc_high . $calculated_crc_low . $stop_byte, suppress_warnings => 1);
	$port_obj->write($start_byte . $dst . $cid . $calculated_crc_high . $calculated_crc_low . $stop_byte);

	# receive
	my $res = '';
	my ($c, $s);
	do {
		($c, $s) = $port_obj->read(255);
		if ($c) {
			$res .= $s;
		}
	} while (ord($s) != 0x0d);

	warn hexdump(data => kmpByteUnstuff($res), suppress_warnings => 1);
	return kmpByteUnstuff($res);
}

sub kmpSetClock {
	$_ = shift;
	my ($year, $month, $day, $hour, $min, $sec);
	$year = $_->{'year'};
	$month = $_->{'month'};
	$day = $_->{'day'};
	$hour = $_->{'hour'};
	$min = $_->{'min'};
	$sec = $_->{'sec'};

	my ($date, $time);
	$date = kmpDate({'year' => $year, 'month' => $month, 'day' => $day});
	$time = kmpTime({'hour' => $hour, 'min' => $min, 'sec' => $sec});
	my $date_time = $date . $time;
	
	my $ctx = Digest::CRC->new(width=>16, init=>0x0000, xorout=>0x0000, 
							refout=>0, poly=>0x1021, refin=>0, cont=>0);
	
	my $start_byte = chr(0x80);
	my $dst = chr(0x3f);
	my $cid = chr(0x09);
	my $stop_byte = chr(0x0d);
	
	# calculate crc
	my $i;
	my $data = $dst . $cid . $date_time;
	
	for ($i = 0; $i < length($data); $i++) {
		$ctx->add(substr($data, $i, 1));
	}
	my $calculated_crc = $ctx->digest;
	my $calculated_crc_low = chr($calculated_crc & 0xff);
	my $calculated_crc_high = chr($calculated_crc >> 8);
	
	# send
	warn hexdump(data => $start_byte . kmpByteStuff($data . $calculated_crc_high . $calculated_crc_low) . $stop_byte, suppress_warnings => 1);
	$port_obj->write($start_byte . kmpByteStuff($data . $calculated_crc_high . $calculated_crc_low) . $stop_byte);

	# receive
	my $res = '';
	my ($c, $s);
	do {
		($c, $s) = $port_obj->read(255);
		if ($c) {
			$res .= $s;
		}
	} while (ord($s) != 0x06);

	warn hexdump(data => $res, suppress_warnings => 1);
	return $res;
}

sub kmpGetRegister {
	$_ = shift;
	my ($rid);
	$rid = $_->{'rid'};
	
	my $ctx = Digest::CRC->new(width=>16, init=>0x0000, xorout=>0x0000, 
							refout=>0, poly=>0x1021, refin=>0, cont=>0);
	
	my $start_byte = chr(0x80);
	my $dst = chr(0x3f);
	my $cid = chr(0x10);
	my $regs = chr(0x01);
	my $rid_low = chr($rid & 0xff);
	my $rid_high = chr($rid >> 8);
	my $stop_byte = chr(0x0d);
	
	# calculate crc
	my $i;
	my $data = $dst . $cid . $regs . $rid_high . $rid_low;
	
	for ($i = 0; $i < length($data); $i++) {
		$ctx->add(substr($data, $i, 1));
	}
	my $calculated_crc = $ctx->digest;
	my $calculated_crc_low = chr($calculated_crc & 0xff);
	my $calculated_crc_high = chr($calculated_crc >> 8);
	
	# send
	warn hexdump(data => $start_byte . kmpByteStuff($data . $calculated_crc_high . $calculated_crc_low) . $stop_byte, suppress_warnings => 1);
	$port_obj->write($start_byte . kmpByteStuff($data . $calculated_crc_high . $calculated_crc_low) . $stop_byte);

	# receive
	my $res = '';
	my ($c, $s);
	do {
		($c, $s) = $port_obj->read(255);
		if ($c) {
			$res .= $s;
		}
	} while (ord($s) != 0x0d);

	warn hexdump(data => kmpByteUnstuff($res), suppress_warnings => 1);
	return kmpByteUnstuff($res);

}

sub kmpPutRegister {
	
}



# helper functions

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
	my $data = shift;
	
	if ($data =~ s/\x1b\x7f/\x80/sm) {
		warn "0x80 unstuffed";
	}
	elsif ($data =~ s/\x1b\xbf/\x40/sm) {
		warn "0x40 unstuffed";
	}
	elsif ($data =~ s/\x1b\xf2/\x0d/sm) {
		warn "0x0d unstuffed";
	}
	elsif ($data =~ s/\x1b\xf9/\x06/sm) {
		warn "0x06 unstuffed";
	}
	elsif ($data =~ s/\x1b\xe4/\x1b/sm) {
		warn "0x1b unstuffed";
	}

	return $data;
}

sub kmpDate {
	$_ = shift;
	my ($year, $month, $day);
	$year = $_->{'year'} - 2000;
	$month = $_->{'month'};
	$day = $_->{'day'};
	my $hex_date = sprintf("%08x", sprintf("%02d%02d%02d", $year, $month, $day));
	my $date = '';
	for (0..3) {
		$date .= chr(hex(substr($hex_date, 2 * $_, 2)));
	}
	return $date;
}

sub kmpTime {
	$_ = shift;
	my ($hour, $min, $sec);
	$hour = $_->{'hour'};
	$min = $_->{'min'};
	$sec = $_->{'sec'};
	my $hex_time = sprintf("%08x", sprintf("%02d%02d%02d", $hour, $min, $sec));
	my $time = '';
	for (0..3) {
		$time .= chr(hex(substr($hex_time, 2 * $_, 2)));
	}
	return $time;
}

1;
