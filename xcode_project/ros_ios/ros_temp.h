//
//  ros_temp.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//

#ifndef ros_temp_h
#define ros_temp_h

#import <ros/node_handle.h>
#import <ros/publisher.h>
#import <ros/subscriber.h>
#import <rt_audio_ros/AudioStream.h>
#import <boost/thread/thread.hpp>
#import <boost/thread/mutex.hpp>
#import <std_msgs/Float32.h>

#import <vector>

@class TempViewController;

class RosTemp
{
public:
    TempViewController __weak * view_controller_;
    RosTemp();
    ~RosTemp();
    void rosSpin();
    float temp;
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    boost::thread * ros_thread_;
    boost::mutex mtx_;
    void renewData(const std_msgs::Float32::ConstPtr & msg);
};

#endif
