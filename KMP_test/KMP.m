//
//  KMP.m
//  MeterLogger
//
//  Created by stoffer on 28/05/14.
//  Copyright (c) 2014 9Lab. All rights reserved.
//

#import "KMP.h"

@implementation KMP

@synthesize frame;

@synthesize responseData;
@synthesize crc16Table;


#pragma mark - Init

-(id)init {
    self = [super init];
    
    self.frame = [[NSMutableData alloc] init];
    self.responseData = [[NSMutableDictionary alloc] init];
    
    self.crc16Table = @[
                        @0x0000, @0x1021, @0x2042, @0x3063, @0x4084, @0x50a5, @0x60c6, @0x70e7,
                        @0x8108, @0x9129, @0xa14a, @0xb16b, @0xc18c, @0xd1ad, @0xe1ce, @0xf1ef,
                        @0x1231, @0x0210, @0x3273, @0x2252, @0x52b5, @0x4294, @0x72f7, @0x62d6,
                        @0x9339, @0x8318, @0xb37b, @0xa35a, @0xd3bd, @0xc39c, @0xf3ff, @0xe3de,
                        @0x2462, @0x3443, @0x0420, @0x1401, @0x64e6, @0x74c7, @0x44a4, @0x5485,
                        @0xa56a, @0xb54b, @0x8528, @0x9509, @0xe5ee, @0xf5cf, @0xc5ac, @0xd58d,
                        @0x3653, @0x2672, @0x1611, @0x0630, @0x76d7, @0x66f6, @0x5695, @0x46b4,
                        @0xb75b, @0xa77a, @0x9719, @0x8738, @0xf7df, @0xe7fe, @0xd79d, @0xc7bc,
                        @0x48c4, @0x58e5, @0x6886, @0x78a7, @0x0840, @0x1861, @0x2802, @0x3823,
                        @0xc9cc, @0xd9ed, @0xe98e, @0xf9af, @0x8948, @0x9969, @0xa90a, @0xb92b,
                        @0x5af5, @0x4ad4, @0x7ab7, @0x6a96, @0x1a71, @0x0a50, @0x3a33, @0x2a12,
                        @0xdbfd, @0xcbdc, @0xfbbf, @0xeb9e, @0x9b79, @0x8b58, @0xbb3b, @0xab1a,
                        @0x6ca6, @0x7c87, @0x4ce4, @0x5cc5, @0x2c22, @0x3c03, @0x0c60, @0x1c41,
                        @0xedae, @0xfd8f, @0xcdec, @0xddcd, @0xad2a, @0xbd0b, @0x8d68, @0x9d49,
                        @0x7e97, @0x6eb6, @0x5ed5, @0x4ef4, @0x3e13, @0x2e32, @0x1e51, @0x0e70,
                        @0xff9f, @0xefbe, @0xdfdd, @0xcffc, @0xbf1b, @0xaf3a, @0x9f59, @0x8f78,
                        @0x9188, @0x81a9, @0xb1ca, @0xa1eb, @0xd10c, @0xc12d, @0xf14e, @0xe16f,
                        @0x1080, @0x00a1, @0x30c2, @0x20e3, @0x5004, @0x4025, @0x7046, @0x6067,
                        @0x83b9, @0x9398, @0xa3fb, @0xb3da, @0xc33d, @0xd31c, @0xe37f, @0xf35e,
                        @0x02b1, @0x1290, @0x22f3, @0x32d2, @0x4235, @0x5214, @0x6277, @0x7256,
                        @0xb5ea, @0xa5cb, @0x95a8, @0x8589, @0xf56e, @0xe54f, @0xd52c, @0xc50d,
                        @0x34e2, @0x24c3, @0x14a0, @0x0481, @0x7466, @0x6447, @0x5424, @0x4405,
                        @0xa7db, @0xb7fa, @0x8799, @0x97b8, @0xe75f, @0xf77e, @0xc71d, @0xd73c,
                        @0x26d3, @0x36f2, @0x0691, @0x16b0, @0x6657, @0x7676, @0x4615, @0x5634,
                        @0xd94c, @0xc96d, @0xf90e, @0xe92f, @0x99c8, @0x89e9, @0xb98a, @0xa9ab,
                        @0x5844, @0x4865, @0x7806, @0x6827, @0x18c0, @0x08e1, @0x3882, @0x28a3,
                        @0xcb7d, @0xdb5c, @0xeb3f, @0xfb1e, @0x8bf9, @0x9bd8, @0xabbb, @0xbb9a,
                        @0x4a75, @0x5a54, @0x6a37, @0x7a16, @0x0af1, @0x1ad0, @0x2ab3, @0x3a92, 
                        @0xfd2e, @0xed0f, @0xdd6c, @0xcd4d, @0xbdaa, @0xad8b, @0x9de8, @0x8dc9, 
                        @0x7c26, @0x6c07, @0x5c64, @0x4c45, @0x3ca2, @0x2c83, @0x1ce0, @0x0cc1, 
                        @0xef1f, @0xff3e, @0xcf5d, @0xdf7c, @0xaf9b, @0xbfba, @0x8fd9, @0x9ff8, 
                        @0x6e17, @0x7e36, @0x4e55, @0x5e74, @0x2e93, @0x3eb2, @0x0ed1, @0x1ef0
    ];

    return self;
}


#pragma mark - KMP CIDs

-(void)getType {
    // start byte
    self.frame = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x80} length:1];
    
    // data
    NSMutableData *data = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x3f, 0x01} length:2];
    
    // append crc 16 to data
    [data appendData:[self crc16ForData:data]];
    
    // stuff data
    data = [[self kmpByteStuff:data] mutableCopy];
    
    // create frame
    [self.frame appendData:data];
    [self.frame appendData:[[NSMutableData alloc] initWithBytes:(unsigned char[]){0x0d} length:1]];
    NSLog(@"%@", self.frame);
}

-(void)getSerialNo {
    // start byte
    self.frame = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x80} length:1];

    // data
    NSMutableData *data = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x3f, 0x02} length:2];

    // append crc 16 to data
    [data appendData:[self crc16ForData:data]];
    
    // stuff data
    data = [[self kmpByteStuff:data] mutableCopy];
    
    // create frame
    [self.frame appendData:data];
    [self.frame appendData:[[NSMutableData alloc] initWithBytes:(unsigned char[]){0x0d} length:1]];
    NSLog(@"%@", self.frame);
}

-(void)setClock:(NSDate *)theDate {
    // start byte
    self.frame = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x80} length:1];
    
    // data
    NSMutableData *data = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x3f, 0x09} length:2];
    
    NSMutableData *kmpDateTime = [[NSMutableData alloc] init];
    [kmpDateTime appendData:[self kmpDateWithDate:theDate]];
    [kmpDateTime appendData:[self kmpTimeWithDate:theDate]];
    [data appendData:kmpDateTime];
    NSLog(@"%@", data);
    
    // append crc 16 to data
    [data appendData:[self crc16ForData:data]];
    
    // stuff data
    data = [[self kmpByteStuff:data] mutableCopy];
    
    // create frame
    [self.frame appendData:data];
    [self.frame appendData:[[NSMutableData alloc] initWithBytes:(unsigned char[]){0x0d} length:1]];
    NSLog(@"%@", self.frame);
}

-(void)getRegister:(NSNumber *)theRegister {
    // start byte
    self.frame = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x80} length:1];
    
    // data
    NSMutableData *data = [[NSMutableData alloc] initWithBytes:(unsigned char[]){0x3f, 0x10} length:2];
    [data appendBytes:(unsigned char[]){0x01} length:1];  // number of registers
    
//    NSMutableData *kmpDateTime = [[NSMutableData alloc] init];
//    [kmpDateTime appendData:[self kmpDateWithDate:theDate]];
//    [kmpDateTime appendData:[self kmpTimeWithDate:theDate]];
//    [data appendData:kmpDateTime];
    unsigned char registerHigh = (unsigned char)(theRegister.intValue >> 8);
    unsigned char registerLow = (unsigned char)(theRegister.intValue & 0xff);
    
    [data appendData:[NSData dataWithBytes:(unsigned char[]){registerHigh, registerLow} length:2]];
    NSLog(@"%@", data);
    
    // append crc 16 to data
    [data appendData:[self crc16ForData:data]];
    
    // stuff data
    data = [[self kmpByteStuff:data] mutableCopy];
    
    // create frame
    [self.frame appendData:data];
    [self.frame appendData:[[NSMutableData alloc] initWithBytes:(unsigned char[]){0x0d} length:1]];
    NSLog(@"%@", self.frame);
    
	
}

-(void)putRegister {
	
}

-(void)decodeFrame:(NSData *)theFrame {
    [self.frame appendData:theFrame];
    if ([theFrame isEqualToData:[[NSData alloc] initWithBytes:(unsigned char[]){0x0d} length:1]]) {
        // end of data - get params from frame
        unsigned char *bytes = (unsigned char*)self.frame.bytes;
        
        [self.responseData setObject:[NSData dataWithBytes:bytes length:1] forKey:@"starByte"];
        [self.responseData setObject:[NSData dataWithBytes:(bytes + self.frame.length - 1) length:1] forKey:@"stopByte"];

        // unstuff data
        NSRange range = NSMakeRange(1, self.frame.length - 2);
        NSData *unstuffedFrame = [self kmpByteUnstuff:[self.frame subdataWithRange:range]];
        bytes = (unsigned char*)unstuffedFrame.bytes;

        [self.responseData setObject:[NSData dataWithBytes:bytes length:1] forKey:@"dst"];
        [self.responseData setObject:[NSData dataWithBytes:(bytes + 1) length:1] forKey:@"cid"];
        range = NSMakeRange(unstuffedFrame.length - 2, 2);
        [self.responseData setObject:[unstuffedFrame subdataWithRange:range] forKey:@"crc"];

        // calculate crc
        range = NSMakeRange(0, unstuffedFrame.length - 2);
        NSData *data = [unstuffedFrame subdataWithRange:range];

        if ([[self crc16ForData:data] isEqualToData:responseData[@"crc"]]) {
            NSLog(@"crc ok");
        }

        // decode application layer
        unsigned char *cid_ptr = (unsigned char *)[self.responseData[@"cid"] bytes];
        unsigned char cid = cid_ptr[0];
        if (cid == 0x01) {         // GetType
            NSLog(@"GetType");
            
            range = NSMakeRange(2, 2);
            [self.responseData setObject:[data subdataWithRange:range] forKey:@"meterType"];
            
            range = NSMakeRange(4, 2);
            [self.responseData setObject:[data subdataWithRange:range] forKey:@"swRevision"];
        }
        else if (cid == 0x02) {
            NSLog(@"GetSerialNo"); // GetSerialNo
            range = NSMakeRange(2, data.length - 2);
            bytes = (unsigned char *)[[data subdataWithRange:range] bytes];
            unsigned int serialNo = (bytes[0] << 24) + (bytes[1] << 16) + (bytes[2] << 8) + bytes[3];
            [self.responseData setObject:[NSNumber numberWithUnsignedInt:serialNo] forKey:@"serialNo"] ;
            NSLog(@"%d", serialNo);
        }
        else if (cid == 0x10) {    // GetRegister
            NSLog(@"GetRegister");
            range = NSMakeRange(2, 2);
            bytes = (unsigned char *)[[data subdataWithRange:range] bytes];
            unsigned int rid = (bytes[0] << 8) + bytes[1];
            [self.responseData setObject:[NSNumber numberWithUnsignedInt:rid] forKey:@"rid"];

            range = NSMakeRange(4, 1);
            bytes = (unsigned char *)[[data subdataWithRange:range] bytes];
            unsigned int unit = bytes[0];
            [self.responseData setObject:[NSNumber numberWithUnsignedInt:unit] forKey:@"unit"];

            range = NSMakeRange(5, 1);
            bytes = (unsigned char *)[[data subdataWithRange:range] bytes];
            unsigned int length = bytes[0];
            [self.responseData setObject:[NSNumber numberWithUnsignedInt:length] forKey:@"length"];
            
            range = NSMakeRange(6, 1);
            bytes = (unsigned char *)[[data subdataWithRange:range] bytes];
            unsigned int siEx = bytes[0];
            [self.responseData setObject:[NSNumber numberWithUnsignedInt:siEx] forKey:@"siEx"];
            
            range = NSMakeRange(7, 4);
            [self.responseData setObject:[data subdataWithRange:range] forKey:@"value"];
        }
        else if (cid == 0x11) {    // PutRegister
            NSLog(@"PutRegister");
            range = NSMakeRange(2, data.length - 2);
            NSLog(@"%@", [data subdataWithRange:range]);
        }
        CFShow((__bridge CFTypeRef)(self.responseData));

    }
    else if ([theFrame isEqualToData:[[NSData alloc] initWithBytes:(unsigned char[]){0x06} length:1]]) {
        NSLog(@"SetClock no CRC");      // SetClock
        CFShow((__bridge CFTypeRef)(self.responseData));
    }
}


#pragma mark - Helper methods

-(NSData *)crc16ForData:(NSData *)theData {
    char *buf = (char *)theData.bytes;

    int counter;
    unsigned short crc16 = 0;
    for (counter = 0; counter < theData.length; counter++) {
        crc16 = (crc16 << 8) ^ [crc16Table[((crc16 >> 8) ^ *(char *)buf++) & 0x00FF] intValue];
    }
    unsigned char crcHigh = (unsigned char)(crc16 >> 8);
    unsigned char crcLow = (unsigned char)(crc16 & 0xff);

    return [[NSData alloc] initWithBytes:(unsigned char[]){crcHigh, crcLow} length:2];
}

-(NSData *)kmpDateWithDate:(NSDate *)theDate {
    NSCalendar* calendar = [NSCalendar currentCalendar];
    NSDateComponents* components = [calendar components:NSYearCalendarUnit|NSMonthCalendarUnit|NSDayCalendarUnit fromDate:theDate];
    
    unsigned int year = (int)(components.year - 2000);
    unsigned int month = (int)(components.month);
    unsigned int day = (int)(components.day);
    
    NSString *dateString = [NSString stringWithFormat:@"%02d%02d%02d", year, month, day];
    NSString *hexDate = [NSString stringWithFormat:@"%08x", dateString.intValue];
    NSLog(@"%@", hexDate);

    NSMutableData *result = [[NSMutableData alloc] init];
    unsigned int i;
    for (i = 0; i < 4; i++) {
        NSRange range = NSMakeRange(2 * i, 2);
        NSString* hexValue = [hexDate substringWithRange:range];
        NSScanner* scanner = [NSScanner scannerWithString:hexValue];
        unsigned int intValue;
        [scanner scanHexInt:&intValue];
        unsigned char uc = (unsigned char) intValue;
        [result appendBytes:&uc length:1];
    }
    return result;
}

-(NSData *)kmpTimeWithDate:(NSDate *)theDate {
    NSCalendar* calendar = [NSCalendar currentCalendar];
    NSDateComponents* components = [calendar components:NSHourCalendarUnit|NSMinuteCalendarUnit|NSSecondCalendarUnit fromDate:theDate];
    
    unsigned int hour = (int)(components.hour);
    unsigned int minute = (int)(components.minute);
    unsigned int second = (int)(components.second);
    
    NSString *dateString = [NSString stringWithFormat:@"%02d%02d%02d", hour, minute, second];
    NSString *hexDate = [NSString stringWithFormat:@"%08x", dateString.intValue];
    NSLog(@"%@", hexDate);
    
    NSMutableData *result = [[NSMutableData alloc] init];
    unsigned int i;
    for (i = 0; i < 4; i++) {
        NSRange range = NSMakeRange(2 * i, 2);
        NSString* hexValue = [hexDate substringWithRange:range];
        NSScanner* scanner = [NSScanner scannerWithString:hexValue];
        unsigned int intValue;
        [scanner scanHexInt:&intValue];
        unsigned char uc = (unsigned char) intValue;
        [result appendBytes:&uc length:1];
    }
    return result;
}

-(NSDate *)dateWithKmpDate:(NSData *)theData {
    return [NSDate date];
}

-(NSDate *)dateWithKmpTime:(NSData *)theData {
    return [NSDate date];
}

-(NSData *)kmpByteStuff:(NSData *)theData {
    unsigned char *bytes = (unsigned char *)theData.bytes;
    unsigned long len = theData.length;
    
    NSMutableData *stuffedData = [[NSMutableData alloc] init];
    
    unsigned long i;
    for (i = 0; i < len; i++) {
        if (bytes[i] == 0x80) {
            [stuffedData appendBytes:(unsigned char[]){0x1b, 0x7f} length:2];
            NSLog(@"0x80 stuffed");
        }
        else if (bytes[i] == 0x40) {
            [stuffedData appendBytes:(unsigned char[]){0x1b, 0xbf} length:2];
            NSLog(@"0x40 stuffed");
        }
        else if (bytes[i] == 0x0d) {
            [stuffedData appendBytes:(unsigned char[]){0x1b, 0xf2} length:2];
            NSLog(@"0x0d stuffed");
        }
        else if (bytes[i] == 0x06) {
            [stuffedData appendBytes:(unsigned char[]){0x1b, 0xf9} length:2];
            NSLog(@"0x06 stuffed");
        }
        else if (bytes[i] == 0x1b) {
            [stuffedData appendBytes:(unsigned char[]){0x1b, 0xe4} length:2];
            NSLog(@"0x1b stuffed");
        }
        else {
            [stuffedData appendBytes:(bytes + i) length:1];
        }
    }
    return stuffedData;
}

-(NSData *)kmpByteUnstuff:(NSData *)theData {
    NSLog(@"unstuffed %@", theData);
    return theData;
}

@end
