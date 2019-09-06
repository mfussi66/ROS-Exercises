#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <cstdlib>
#include <cmath>

// Global variables init to use between functions
float pose_x = 0.0;
float pose_y = 0.0;
float pose_theta = 0.0;

geometry_msgs::Twist ctrl;

void checkPoseCallback(const turtlesim::PoseConstPtr msg){
/* Callback function to get turtle pose from simulator. */

    pose_x = msg->x;
    pose_y = msg->y;
    pose_theta = msg->theta;
    
}

void adjustPose(float setpoint, float gain_ang, float gain_lin, float deadband){
/* Corrects the turtle pose to given setpoint when it is under the boundary. 
 * Uses a P control with a dead band on the orientation, then sets a linear escape velocity. */

  if (fabs(pose_theta - setpoint) < deadband)
  {
    ctrl.angular.z = 0;
    ctrl.linear.x = gain_lin;
    ROS_INFO("-- Escaping --");
  }
  else
  {
    ctrl.angular.z = gain_ang * (setpoint - pose_theta);
    ctrl.linear.x = 0;  
    ROS_INFO("-- Correcting angle --");
  }
  
}

void randomControl(int limit){
/* Sets random angular and linear velocities according to the limit set.
 * Can be positive or negative.*/
  
  int rnd_linear = 0;
  int rnd_angular = 0;
 
  rnd_linear = rand() % (limit*2+1) + (-limit);
  rnd_angular = rand() % (limit*2+1) + (-limit);
  
  ctrl.linear.x = (float)rnd_linear;
  ctrl.angular.z = (float)rnd_angular;
    
}

int main(int argc, char **argv){
  
    ros::init(argc, argv, "exercise3");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
  
    float limit_pose_y = 5.5;

    srand(time(NULL)); // Initialize random seed with time
    
    // Subscribe and advertise to relevant topics
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    ros::Subscriber sub = nh.subscribe("/turtle1/pose", 100, &checkPoseCallback);
    
    ros::spinOnce(); // Spin once to get initial pose
    
    // Main control loop
    while(ros::ok())
    {     
        if (pose_y < limit_pose_y)
        {    
	  adjustPose(M_PI/2, 0.4, 2.0, 0.1);
        }
        else
        {  
	  randomControl(2);
        }

        pub.publish(ctrl);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
   
}
