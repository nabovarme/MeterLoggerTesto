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
@property NSArray *crc16Table;


-(void)getType;
-(void)getSerialNo;
-(void)setClock:(NSDate *)theDate;
-(void)getRegister;
-(void)putRegister;

-(NSData *)crc16ForData:(NSData *)data;
-(NSData *)kmpDate:(NSDate *)theDate;
-(NSData *)kmpTime:(NSDate *)theDate;

@end
