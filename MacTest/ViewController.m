//
//  ViewController.m
//  MacTest
//
//  Created by 郭龙 on 16/7/28.
//  Copyright © 2016年 郭龙. All rights reserved.
//

#import "ViewController.h"

@implementation ViewController

- (void)viewDidLoad {
    [super viewDidLoad];

    NSTextField *texted = [[NSTextField alloc] initWithFrame:NSRectFromCGRect(CGRectMake(100, 100, 50, 100))];
    [self.view addSubview:texted];
    texted.placeholderString = @"guolong";
    texted.backgroundColor = [NSColor yellowColor];
    texted.editable = NO;
    NSLog(@"guolong");
    
    NSButton *btn = [[NSButton alloc] initWithFrame:NSRectFromCGRect(CGRectMake(0, 0, 100, 50))];
    [self.view addSubview:btn];
    [btn setTitle:@"初始化"];
    [btn setTarget:self];
    [btn setAction:@selector(btnClicked:)];
    
    // Do any additional setup after loading the view.
}

- (void)btnClicked:(id)sender {
    NSLog(@"btn clcked");
}

- (void)setRepresentedObject:(id)representedObject {
    [super setRepresentedObject:representedObject];

    // Update the view, if already loaded.
}

@end
