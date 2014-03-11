//
//  ros_Relay.h
//  ros_ios
//
//  Created by xf on 3/11/14.
//  Copyright (c) 2014 Ronan Chauvin. All rights reserved.
//


#ifndef ros_relay_h
#define ros_relay_h

#import <ros/node_handle.h>
#import <ros/publisher.h>
#import <boost/thread/thread.hpp>

class RosRelay
{
public:
    RosRelay();
    ~RosRelay();
    void rosSpin();
    void sendCmds(bool status);
    
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    boost::thread * ros_thread_;
};

#endif
