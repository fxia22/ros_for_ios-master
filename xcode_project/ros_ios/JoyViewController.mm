//
//  JoyViewController.mm
//  ros_ios
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "JoyViewController.h"
#import <QuartzCore/QuartzCore.h>
#import <AudioToolbox/AudioToolbox.h>

#import <vector>
#import <math.h>

@interface JoyViewController ()

@end

@implementation JoyViewController

@synthesize ballView;
@synthesize ballView2;

const double max_vel_x = 0.6;
const double max_vel_y = 0.6;
const double max_vel_th = 1.0;

- (void)viewDidLoad
{
    [super viewDidLoad];
    [self.view setMultipleTouchEnabled:YES];
    [self.view setBackgroundColor:[UIColor whiteColor]];
	// Do any additional setup after loading the view, typically from a nib.
    NSLog(@"AccJoyViewController : viewDidLoad");
    
    ros_controller_ = new RosJoy();
    
    ballView = [[UIImageView alloc] initWithFrame:CGRectMake(0, 0, 30, 30)];
    [ballView setBackgroundColor:[UIColor redColor]];
    ballView.layer.cornerRadius = 6;
    ballView.userInteractionEnabled = YES;
    [self.view addSubview:ballView];
    
    
    ballView2 = [[UIImageView alloc] initWithFrame:CGRectMake(0, 0, 30, 30)];
    [ballView2 setBackgroundColor:[UIColor orangeColor]];
    ballView2.layer.cornerRadius = 15;
    ballView2.userInteractionEnabled = YES;
    [self.view addSubview:ballView2];
    angle = 0.0;
    
    
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(restart)
                                                 name:UIApplicationDidBecomeActiveNotification object:nil];
    
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
    NSLog(@"didReceiveMemoryWarning");
}

- (void)viewWillAppear:(BOOL)animated
{
    NSLog(@"AccJoyViewController : viewWillAppear");
    center1.x = self.view.frame.size.width/2;
    center1.y = self.view.frame.size.height/3+22;
    ballView.center = center1;
    
    jlength = self.view.frame.size.width/4;
    center2.x = self.view.frame.size.width/2;//+jlength*cos(angle);
    center2.y = self.view.frame.size.height/3*2+11;//+jlength*sin(angle);
    ballView2.center = CGPointMake(center2.x+jlength*cos(angle), center2.y+jlength*sin(angle));
    [self startTimer];
}

- (void)viewWillDisappear:(BOOL)animated
{
    NSLog(@"AccJoyViewController : viewWillDisappear");
    [self stopTimer];
    delete ros_controller_;
    
}

-(void)dealloc
{
    NSLog(@"AccJoyViewController : dealloc");
}

- (void)touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event
{
   for (UITouch *touch in touches)
   {
        NSLog(@" - %p",touch);
       
       if (touch.view == self.ballView)
       {
           [ballView setBackgroundColor:[UIColor blueColor]];
           [self vibrate];
           ballPushed1 = YES;
       }
       else
           if (touch.view == self.ballView2) {
               [ballView2 setBackgroundColor:[UIColor yellowColor]];
               [self vibrate];
               ballPushed2 = YES;
               
           }
   }
    
   
}

- (void)touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event
{
    for (UITouch *touch in touches)
    {
    CGPoint touch_pt = [touch locationInView:self.view];
    if(touch.view == self.ballView)
    {
        currentPoint = touch_pt;
        self.image.image = nil;
        
        UIGraphicsBeginImageContext(self.view.frame.size);
        [self.image.image drawInRect:self.view.frame];
        CGContextMoveToPoint(UIGraphicsGetCurrentContext(), center1.x, center1.y);
        CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), currentPoint.x, currentPoint.y);
        CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
        CGContextSetLineWidth(UIGraphicsGetCurrentContext(), 5.0);
        CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), 1, 1, 1, 1.0);
        CGContextSetBlendMode(UIGraphicsGetCurrentContext(),kCGBlendModeNormal);
        
        CGContextStrokePath(UIGraphicsGetCurrentContext());
        self.image.image = UIGraphicsGetImageFromCurrentImageContext();
        UIGraphicsEndImageContext();
        
        ballView.center = currentPoint;
    }
        else if (touch.view==self.ballView2)
        {
            currentPoint = touch_pt;
            self.image.image = nil;
            angle = atan2(currentPoint.y-center2.y, currentPoint.x-center2.x);
            ballView2.center = CGPointMake(center2.x+jlength*cos(angle), center2.y+jlength*sin(angle));;

            UIGraphicsBeginImageContext(self.view.frame.size);
            [self.image.image drawInRect:self.view.frame];
            CGContextMoveToPoint(UIGraphicsGetCurrentContext(), center2.x, center2.y);
            CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), ballView2.center.x, ballView2.center.y);
            CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
            CGContextSetLineWidth(UIGraphicsGetCurrentContext(), 5.0);
            CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), 1, 1, 1, 1.0);
            CGContextSetBlendMode(UIGraphicsGetCurrentContext(),kCGBlendModeNormal);
            
            CGContextStrokePath(UIGraphicsGetCurrentContext());
            self.image.image = UIGraphicsGetImageFromCurrentImageContext();
            UIGraphicsEndImageContext();

        }
    }
}

- (void)touchesEnded:(NSSet *)touches withEvent:(UIEvent *)event
{
    
    for (UITouch *touch in touches)
    {
    
    if (touch.view == self.ballView)
    {
        self.image.image = nil;
        [ballView setBackgroundColor:[UIColor redColor]];
        [self vibrate];
        ballPushed1 = NO;
        ballView.center = center1;
    }
    else if (touch.view == self.ballView2)
    {
        self.image.image = nil;
        [ballView2 setBackgroundColor:[UIColor orangeColor]];
        [self vibrate];
        ballPushed2 = NO;
        angle = 0.0;
        ballView2.center = CGPointMake(center2.x+jlength*cos(angle), center2.y+jlength*sin(angle));

    }
    
    }
    
    
}

- (void)timerCB
{
    [self generateCmds];
}

- (IBAction)startTimer
{
    timer = [NSTimer scheduledTimerWithTimeInterval:0.1 target:self selector:@selector(timerCB) userInfo:nil repeats:YES];
}

- (IBAction)stopTimer
{
    [timer invalidate];
}

- (void)generateCmds
{
 
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    x = (self.ballView.center.x - center1.x)/(self.view.frame.size.width/2)*max_vel_x;
    y = (self.ballView.center.y - center1.y)/(self.view.frame.size.height/4)*max_vel_y;
    th = angle / 3.14*max_vel_th;
    ros_controller_->sendCmds(x, -y, -th);

}

- (void)vibrate
{
   // AudioServicesPlaySystemSound(kSystemSoundID_Vibrate);
}

-(void) restart
{
    delete ros_controller_;
    ros_controller_ = new RosJoy();
}


@end
