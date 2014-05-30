//
//  KMP.h
//  MeterLogger
//
//  Created by stoffer on 28/05/14.
//  Copyright (c) 2014 9Lab. All rights reserved.
//

#import <Foundation/Foundation.h>

@interface KMP : NSObject;

@property NSMutableData *frame;
@property unsigned char startByte;
@property unsigned char dst;
@property unsigned char cid;
@property int16_t rid;
@property int16_t crc;
@property unsigned char stopByte;
@property NSArray *crc16Table;


-(void)getType;
-(void)getSerialNo;
-(void)setClock:(NSDate *)theDate;
-(void)getRegister;
-(void)putRegister;

-(void)decodeFrame:(NSData *)theFrame;

-(NSData *)crc16ForData:(NSData *)data;
-(NSData *)kmpDate:(NSDate *)theDate;
-(NSData *)kmpTime:(NSDate *)theDate;
-(NSData *)kmpByteStuff:(NSData *)theData;
-(NSData *)kmpByteUnstuff:(NSData *)theData;


@end
