//
//  JoyViewController.h
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_joy.h"

@interface JoyViewController : UIViewController
{
    RosJoy * ros_controller_;
    NSTimer * timer;
    BOOL ballPushed1;
    BOOL ballPushed2;
    
    CGPoint currentPoint;
    CGPoint center1;
    CGPoint center2;
    double angle;
    int jlength;
}

@property (strong, nonatomic) IBOutlet UIImageView *image;
@property (strong, nonatomic) IBOutlet UIView *ballView;
@property (strong, nonatomic) IBOutlet UIView *ballView2;

- (void)timerCB;

- (IBAction)startTimer;

- (IBAction)stopTimer;

- (void)generateCmds;

- (void)vibrate;

@end
