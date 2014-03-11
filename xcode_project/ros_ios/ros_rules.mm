//
//  ros_Rule.m
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import "ros_rules.h"
#import "RuleViewController.h"
#import <std_msgs/Bool.h>
RosRule::RosRule()
{
    sub_ = n_.subscribe("/building1/room1/sensors/light/light1", 1, &RosRule::renewData, this);
    pub_ = n_.advertise<std_msgs::Bool>("/building1/room1/acuator/relay/relay1", 1);
    ros_thread_ = new boost::thread(&RosRule::rosSpin, this);
    this->Rule = 0.0f;
}

RosRule::~RosRule()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosRule::rosSpin()
{
    ros::spin();
}


void RosRule::renewData(const std_msgs::Bool::ConstPtr & msg)
{
    std_msgs::Bool mesg;
    mesg.data = !msg->data;
    pub_.publish(mesg);
}
