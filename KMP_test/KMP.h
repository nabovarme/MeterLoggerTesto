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

@property NSMutableDictionary *responseData;
@property NSArray *crc16Table;


-(void)getType;
-(void)getSerialNo;
-(void)setClock:(NSDate *)theDate;
-(void)getRegister:(NSNumber *)theRegister;
-(void)putRegister;

-(void)decodeFrame:(NSData *)theFrame;

-(NSData *)crc16ForData:(NSData *)theData;

-(NSData *)kmpDateWithDate:(NSDate *)theDate;
-(NSData *)kmpTimeWithDate:(NSDate *)theDate;

-(NSDate *)dateWithKmpDate:(NSData *)theData;
-(NSDate *)dateWithKmpTime:(NSData *)theData;

-(NSData *)kmpByteStuff:(NSData *)theData;
-(NSData *)kmpByteUnstuff:(NSData *)theData;


@end
