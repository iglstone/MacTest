//
//  ViewController.m
//  MacTest
//
//  Created by 郭龙 on 16/7/28.
//  Copyright © 2016年 郭龙. All rights reserved.
//

#import "ViewController.h"
#import <Masonry.h>
#import "SlamDev.h"
#import "LindarReader.h"

@interface ViewController()
{
    SlamDev *slam;
    LindarReader *lindar;
}
@end

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];
    slam = [[SlamDev alloc] init];
    lindar = [[LindarReader alloc] init];

    NSTextField *texted = [[NSTextField alloc] initWithFrame:NSRectFromCGRect(CGRectMake(100, 100, 50, 100))];
    [self.view addSubview:texted];
    texted.placeholderString = @"guolong";
    texted.backgroundColor = [NSColor yellowColor];
    texted.editable = NO;
    NSLog(@"guolong");
    
    NSButton *btn = [[NSButton alloc] initWithFrame:NSRectFromCGRect(CGRectMake(0, 0, 100, 50))];
    [self.view addSubview:btn];
    [btn setTitle:@"读取激光数据"];
    [btn setTarget:self];
    [btn setAction:@selector(btnClicked:)];
    
    NSButton *btn2 = [[NSButton alloc] initWithFrame:NSRectFromCGRect(CGRectMake(120, 0, 100, 50))];
    [self.view addSubview:btn2];
    [btn2 setTitle:@"Slam计算"];
    [btn2 setTarget:self];
    [btn2 setAction:@selector(slamCalc:)];
    
    // Do any additional setup after loading the view.
}

- (void)btnClicked:(id)sender {
    NSLog(@"btn clcked");
    [lindar readData];
}

- (void )slamCalc: (id)sender
{
    NSLog(@"slam clicked");
    NSThread* myThread = [[NSThread alloc] initWithTarget:self selector:@selector(slamStart:) object:nil];
    [myThread start];
}

- (void )slamStart:(id)sender
{
    [slam slamMap];
}

//- (NSButton *)btnProcess:(CGRect )rect title:(NSString *)title slect:(sele)


- (void)setRepresentedObject:(id)representedObject {
    [super setRepresentedObject:representedObject];

    // Update the view, if already loaded.
}

@end
