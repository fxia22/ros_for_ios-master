//
//  SwitchTableViewController.h
//  ros_ios
//
//  Created by xf on 3/10/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_relay.h"

@interface SwitchTableViewController : UITableViewController
{
    bool status;
    RosRelay*  ros_controller_;
}


@end
