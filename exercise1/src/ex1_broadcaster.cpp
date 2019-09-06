#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

void poseCallback(const turtlesim::PoseConstPtr& msg){
/* Callback function for subscriber to broadcast position and orientation.
 * Used to be displayed by rviz. */

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "turtle1";
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
  
// Sends transform with broadcaster 
  br.sendTransform(transformStamped);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "ex1_pub");
    ros::NodeHandle nh("~");
    ros::Rate rate(100);

    geometry_msgs::Twist cmd;

    // Subscribe and advertise in necessary topics to send commands and read pose
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 10, &poseCallback);

    ROS_INFO("Sending Twist commands");

    while (ros::ok())
    {
        cmd.linear.x = 0.5;
        cmd.linear.y = 0.5;
        cmd.angular.z = 1;
        pub.publish(cmd);
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
