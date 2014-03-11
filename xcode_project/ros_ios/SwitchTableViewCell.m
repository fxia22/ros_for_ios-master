//
//  SwitchTableViewCell.m
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import "SwitchTableViewCell.h"

@implementation SwitchTableViewCell
@synthesize deviceName = _deviceNmae;

- (id)initWithStyle:(UITableViewCellStyle)style reuseIdentifier:(NSString *)reuseIdentifier
{
    self = [super initWithStyle:style reuseIdentifier:reuseIdentifier];
    if (self) {
        // Initialization code
        self->status = false;
    }
    return self;
}

- (void)setSelected:(BOOL)selected animated:(BOOL)animated
{
    [super setSelected:selected animated:animated];

    // Configure the view for the selected state
 
}

- (IBAction)switchDevice:(id)sender
{
    self->status = !self->status;
    NSLog(@"%d",status);
}
@end
