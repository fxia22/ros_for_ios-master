//
//  SwitchTableViewCell.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface SwitchTableViewCell : UITableViewCell
{
    UILabel * deviceName;
    bool status;
}
@property (strong, nonatomic) IBOutlet UILabel *deviceName;
- (IBAction)switchDevice:(id)sender;


@end
