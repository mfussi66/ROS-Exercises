#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "exercise2/control.h"
#include <iostream>

ros::Publisher pub;

float v;
float w;

void commandCallback(const exercise2::control::ConstPtr& msg)
{
    ROS_INFO("Callback is working!");
    v = msg->lin_vel;
    w = msg->ang_vel;
}

float checkLimits(float var, float var_min, float var_max)
{
    if (var > var_max)
    {
        ROS_INFO("value higher than saturation - Capping.");
        var = var_max;
    }
    else if (var < var_min)
    {
        ROS_INFO("value lower than saturation - Capping.");
        var = var_min;
    }
    
    return var;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "exercise2");
    ros::NodeHandle nh("~");

    double v_min = -1.0;
    double v_max = 1.0;
    double w_min = -1.0;
    double w_max = 1.0;
    
    nh.getParam("vel_lin_min", v_min);
    nh.getParam("vel_lin_max", v_max);
    nh.getParam("vel_ang_min", w_min);
    nh.getParam("vel_ang_max", w_max);
    
    ros::Rate rate(100);
    
    ros::Subscriber sub = nh.subscribe("/control", 10, &commandCallback);
    
    ROS_INFO("Subscribing to topic: /control");
    
    pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    sleep(1);
    
    while (ros::ok())
    {
        geometry_msgs::Twist cmd;
        cmd.linear.x = checkLimits(v, v_min, v_max);
        cmd.angular.z = checkLimits(w, w_min, w_max);
        pub.publish(cmd);
        
        //ROS_INFO("Sending v: %f, w: %f", v, w);
        
        ros::spinOnce();
        rate.sleep();
    }

	return 0;

}

