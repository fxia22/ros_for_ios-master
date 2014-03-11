//
//  TempViewController.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_temp.h"

@interface TempViewController : UITableViewController
{
    RosTemp * ros_controller_;
    float temperature;
}

@end
