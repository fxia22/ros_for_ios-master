//
//  SwitchTableViewCell.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "ros_relay.h"
@interface SwitchTableViewCell : UITableViewCell
{
    UILabel * deviceName;
    bool status;
    RosRelay * ros_controller_;
    
}
@property (strong, nonatomic) IBOutlet UILabel *deviceName;


@end
