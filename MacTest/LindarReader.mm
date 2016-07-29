//
//  LindarReader.m
//  MacTest
//
//  Created by 郭龙 on 16/7/29.
//  Copyright © 2016年 郭龙. All rights reserved.
//

#import "LindarReader.h"
#include <stdio.h>
#include <fcntl.h>
#include "URG04LX2.hpp"

static const char * DEVICE = "/dev/cu.usbmodem1411";
//static const char * DEVICE = "/dev/ttyACM0";

static const int    NITER  = 20;


@implementation LindarReader

-(void) readData
{
    
    //    int op = open(DEVICE, O_RDWR | O_NOCTTY);
    //    if (op < 0) {
    //        printf("open failed: %d\n",op);
    //    }else
    //        printf("open success");
    
    URG04LX2 laser;
    
    cout << "===============================================================" << endl;
    laser.connect(DEVICE, 115200);
    cout << "===============================================================" << endl;
    cout << laser << endl;
    cout << "===============================================================" << endl;
    
    for (int i=1;i<=NITER;i++)
    {
        int data[1000];
        
        int ndata = laser.getScan(data);
        
        printf("Iteration: %3d: ", i);
        
        if (i == 1) {
            for (int j = 0; j < ndata; j++) {
                printf("%d,",data[j]);
            }
        }
        
        if (ndata)
        {
            printf("got %3d data points\n", ndata);
        }
        else
        {
            printf("=== SCAN FAILED ===\n");
        }
    }
    
    //laser.~URG04LX();
    
}


@end
