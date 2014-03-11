//
//  ros_temp.m
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#import "ros_temp.h"
#import "TempViewController.h"

RosTemp::RosTemp()
{
    sub_ = n_.subscribe("/building1/room1/sensors/temperature/temp", 1, &RosTemp::renewData, this);
    ros_thread_ = new boost::thread(&RosTemp::rosSpin, this);
    this->temp = 0.0f;
}

RosTemp::~RosTemp()
{
    ros::shutdown();
    ros_thread_->join();
    delete ros_thread_;
}

void RosTemp::rosSpin()
{
    ros::spin();
}




void RosTemp::renewData(const std_msgs::Float32::ConstPtr & msg)
{
    this->temp = msg->data;
}
