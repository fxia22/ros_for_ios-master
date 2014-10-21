//
//  ViewController.mm
//  robot_help_me
//
//  Created by Ronan Chauvin on 2013-02-13.
//  Copyright (c) 2013 Ronan Chauvin. All rights reserved.
//

#import "ViewController.h"

#import <cstdlib>
#import <ros/init.h>
#import <ros/master.h>

#import <ifaddrs.h>
#import <arpa/inet.h>

@interface ViewController ()


@end

@implementation ViewController

@synthesize defaults, ip_text_field;

- (void)viewDidLoad
{
    
    
    
    [super viewDidLoad];
	// Do any additional setup after loading the view, typically from a nib.
    defaults = [NSUserDefaults standardUserDefaults];
    [ip_text_field setText:[defaults objectForKey:@"master_uri"]];
    [_scanButton addTarget:self action:@selector(setupCamera) forControlEvents:UIControlEventTouchUpInside];
    [self.view addSubview:_scanButton];
    [[NSNotificationCenter defaultCenter] addObserver:self selector:@selector(changeLabelText:) name:@"ChangeLabelTextNotification" object:Nil];
    
    
}

-(void) setupCamera
{
    
    NSLog(@"ok");
    IDVideoViewController * rt = [[IDVideoViewController alloc] init];
    rt.view.backgroundColor = [UIColor whiteColor];
    //[self presentViewController:rt animated:YES completion:^{
        
    //}];
    [self.navigationController pushViewController:rt animated:true];
}
 /*   if(IOS7)
    {
        RootViewController * rt = [[RootViewController alloc]init];
        [self presentViewController:rt animated:YES completion:^{
            
        }];
        
    }
    else
    {
        [self scanBtnAction];
    }
    NSLog(@"ok");
}

-(void)scanBtnAction
{
    num = 0;
    upOrdown = NO;
    //初始话ZBar
    ZBarReaderViewController * reader = [ZBarReaderViewController new];
    //设置代理
    reader.readerDelegate = self;
    //支持界面旋转
    reader.supportedOrientationsMask = ZBarOrientationMaskAll;
    reader.showsHelpOnFail = NO;
    reader.scanCrop = CGRectMake(0.1, 0.2, 0.8, 0.8);//扫描的感应框
    ZBarImageScanner * scanner = reader.scanner;
    [scanner setSymbology:ZBAR_I25
                   config:ZBAR_CFG_ENABLE
                       to:0];
    UIView * view = [[UIView alloc] initWithFrame:CGRectMake(0, 0, 320, 420)];
    view.backgroundColor = [UIColor clearColor];
    reader.cameraOverlayView = view;
    
    
    UILabel * label = [[UILabel alloc] initWithFrame:CGRectMake(20, 20, 280, 40)];
    label.text = @"请将扫描的二维码至于下面的框内\n谢谢！";
    label.textColor = [UIColor whiteColor];
    label.textAlignment = 1;
    label.lineBreakMode = 0;
    label.numberOfLines = 2;
    label.backgroundColor = [UIColor clearColor];
    [view addSubview:label];
    
    UIImageView * image = [[UIImageView alloc] initWithImage:[UIImage imageNamed:@"pick_bg.png"]];
    image.frame = CGRectMake(20, 80, 280, 280);
    [view addSubview:image];
    
    
    _line = [[UIImageView alloc] initWithFrame:CGRectMake(30, 10, 220, 2)];
    _line.image = [UIImage imageNamed:@"line.png"];
    [image addSubview:_line];
    //定时器，设定时间过1.5秒，
    timer = [NSTimer scheduledTimerWithTimeInterval:.02 target:self selector:@selector(animation1) userInfo:nil repeats:YES];
    
    [self presentViewController:reader animated:YES completion:^{
        
    }];
}
-(void)animation1
{
    if (upOrdown == NO) {
        num ++;
        _line.frame = CGRectMake(30, 10+2*num, 220, 2);
        if (2*num == 260) {
            upOrdown = YES;
        }
    }
    else {
        num --;
        _line.frame = CGRectMake(30, 10+2*num, 220, 2);
        if (num == 0) {
            upOrdown = NO;
        }
    }
    
    
}
-(void)imagePickerControllerDidCancel:(UIImagePickerController *)picker
{
    [timer invalidate];
    _line.frame = CGRectMake(30, 10, 220, 2);
    num = 0;
    upOrdown = NO;
    [picker dismissViewControllerAnimated:YES completion:^{
        [picker removeFromParentViewController];
    }];
}
-(void)imagePickerController:(UIImagePickerController *)picker didFinishPickingMediaWithInfo:(NSDictionary *)info
{
    [timer invalidate];
    _line.frame = CGRectMake(30, 10, 220, 2);
    num = 0;
    upOrdown = NO;
    [picker dismissViewControllerAnimated:YES completion:^{
        [picker removeFromParentViewController];
        UIImage * image = [info objectForKey:UIImagePickerControllerOriginalImage];
        //初始化
        ZBarReaderController * read = [ZBarReaderController new];
        //设置代理
        read.readerDelegate = self;
        CGImageRef cgImageRef = image.CGImage;
        ZBarSymbol * symbol = nil;
        id <NSFastEnumeration> results = [read scanImage:cgImageRef];
        for (symbol in results)
        {
            break;
        }
        NSString * result;
        if ([symbol.data canBeConvertedToEncoding:NSShiftJISStringEncoding])
            
        {
            result = [NSString stringWithCString:[symbol.data cStringUsingEncoding: NSShiftJISStringEncoding] encoding:NSUTF8StringEncoding];
        }
        else
        {
            result = symbol.data;
        }
        
        
        NSLog(@"%@",result);
        
    }];
}

*/

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}

- (IBAction)ip_edit_ended:(id)sender
{
    [sender resignFirstResponder];
}

- (BOOL)shouldPerformSegueWithIdentifier:(NSString *)identifier sender:(id)sender
{
    if([ViewController isValidIp:[ip_text_field.attributedText string]])
    {
        [defaults setObject:[ip_text_field.attributedText string] forKey:@"master_uri"];
        [defaults synchronize];
        
        NSString * master_uri = [@"ROS_MASTER_URI=http://" stringByAppendingString:[[ip_text_field.attributedText string] stringByAppendingString:@":11311/"]];
        NSLog(@"%@",master_uri);
        
        NSString * ip = [ViewController getIPAddress];
        NSString * hostname = [@"ROS_HOSTNAME=" stringByAppendingString:ip];
        NSLog(@"%@",hostname);
        putenv((char *)[master_uri UTF8String]);
        putenv((char *)[hostname UTF8String]);
        putenv((char *)"ROS_HOME=/tmp");
        
        int argc = 0;
        char ** argv = NULL;
        NSUInteger ha = ip.hash;
        ha = ha %10000;
        NSString * nodesuffix = [NSString stringWithFormat: @"%s", "ros4ios"];
        NSString * nodename = [NSString stringWithFormat: @"%d", ha];
        NSString * nodewithsuffix = [nodesuffix stringByAppendingString:nodename];
         const char * nodechar =[nodewithsuffix UTF8String];
        if(!ros::isInitialized())
        {
            ros::init(argc,argv,nodechar,ros::init_options::AnonymousName);
            NSLog(@"initialized");
        }
        else
        {
            NSLog(@"ROS already initialised. Can't change the ROS_MASTER_URI");            
        }
        
    
        /*ros::V_string nodelist;
        ros::master::getNodes(nodelist);
        for (auto i = nodelist.begin(); i<nodelist.end(); ++i)
        {
            NSLog(@"Nodes:%s\n",(*i).c_str());
        }
        */
        struct sigaction sa;
        sa.sa_handler = SIG_IGN;
        sigaction(SIGPIPE, &sa, 0);
        //Ignore the SIGPIPE signal
        
        
        
        if(ros::master::check())
        {
            NSLog(@"Connected to the ROS master !");
        }
        else
        {
            UIAlertView * alert = [[UIAlertView alloc] initWithTitle:@"Error !" message:@"Couldn't join the ROS master" delegate:self cancelButtonTitle:@"Ok" otherButtonTitles:nil];
            [alert show];
            return NO;
            ros::shutdown();
            while (ros::isShuttingDown());
            
        }
    }
    else
    {
        UIAlertView * alert = [[UIAlertView alloc] initWithTitle:@"Error !" message:@"ROS Master's IPv4 address is not valid" delegate:self cancelButtonTitle:@"Ok" otherButtonTitles:nil];
        [alert show];
        return NO;
    }
    return YES;
}

-(void)prepareForSegue:(UIStoryboardSegue *)segue sender:(id)sender
{

}

+ (NSString *)getIPAddress
{
    struct ifaddrs *interfaces = NULL;
    struct ifaddrs *temp_addr = NULL;
    NSString *wifiAddress = nil;
    NSString *cellAddress = nil;
    
    // retrieve the current interfaces - returns 0 on success
    if(!getifaddrs(&interfaces)) {
        // Loop through linked list of interfaces
        temp_addr = interfaces;
        while(temp_addr != NULL) {
            sa_family_t sa_type = temp_addr->ifa_addr->sa_family;
            if(sa_type == AF_INET || sa_type == AF_INET6) {
                NSString *name = [NSString stringWithUTF8String:temp_addr->ifa_name];
                NSString *addr = [NSString stringWithUTF8String:inet_ntoa(((struct sockaddr_in *)temp_addr->ifa_addr)->sin_addr)]; // pdp_ip0
                NSLog(@"NAME: \"%@\" addr: %@", name, addr); // see for yourself
                
                if([name isEqualToString:@"en0"]) {
                    // Interface is the wifi connection on the iPhone
                    wifiAddress = addr;
                } else
                    if([name isEqualToString:@"pdp_ip0"]) {
                        // Interface is the cell connection on the iPhone
                        cellAddress = addr;
                    }
            }
            temp_addr = temp_addr->ifa_next;
        }
        // Free memory
        freeifaddrs(interfaces);
    }
    NSString *addr = wifiAddress ? wifiAddress : cellAddress;
    return addr ? addr : @"0.0.0.0";
}

+ (BOOL)isValidIp:(NSString*)string
{
    struct in_addr pin;
    int success = inet_pton(AF_INET,[string UTF8String],&pin);
    if(success == 1) return YES;
    return NO;
}

- (void)changeLabelText:(NSNotification *)notification
{
    id text = notification.object;
    ip_text_field.text = text;
}



@end
