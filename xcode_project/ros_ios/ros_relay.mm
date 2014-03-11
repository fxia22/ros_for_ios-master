//
//  ros_Relay.m
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import "ros_relay.h"
#import "SwitchTableViewCell.h"
#import <ros/time.h>
#import <std_msgs/Bool.h>

RosRelay::RosRelay()
{
   // sub_ = n_.subscribe("/building1/room1/sensors/Relayerature/Relay", 1, &RosRelay::renewData, this);
    pub_ = n_.advertise<std_msgs::Bool>("/building1/room1/acuator/relay/relay1", 1, true);
    ros_thread_ = new boost::thread(&RosRelay::rosSpin, this);
}

RosRelay::~RosRelay()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosRelay::rosSpin()
{
    ros::spin();
}



void RosRelay::sendCmds(bool status)
{
    std_msgs::Bool cmd;
    cmd.data = status;
    pub_.publish(cmd);
}
