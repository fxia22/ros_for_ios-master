//
//  TempCell.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import <UIKit/UIKit.h>

@interface TempCell : UITableViewCell
{
    UILabel * temp_name;
    UILabel * temp_value;
}
@property (nonatomic, strong) IBOutlet UILabel *temp_name;
@property (nonatomic, strong) IBOutlet UILabel *temp_value;

@end
