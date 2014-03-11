//
//  ros_Rule.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#ifndef ros_rules_h
#define ros_rules_h

#import <ros/node_handle.h>
#import <ros/publisher.h>
#import <ros/subscriber.h>
#import <rt_audio_ros/AudioStream.h>
#import <boost/thread/thread.hpp>
#import <boost/thread/mutex.hpp>
#import <std_msgs/Float32.h>
#import <std_msgs/Bool.h>
#import <vector>

@class RuleViewController;

class RosRule
{
public:
    RuleViewController __weak * view_controller_;
    RosRule();
    ~RosRule();
    void rosSpin();
    float Rule;
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    boost::thread * ros_thread_;
    boost::mutex mtx_;
    void renewData(const std_msgs::Bool::ConstPtr & msg);
};

#endif
