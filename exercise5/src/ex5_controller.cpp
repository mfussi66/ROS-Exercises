#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#define N_OBSTACLES 2

// Structures initalizations
struct vect{
  float x;
  float y;
};

struct status{
    vect position;
    float theta;
};

// Structures global variables initializations
status robot_state;
vect obstacles[N_OBSTACLES];
int obs_pose_sent = 0;

float turtle_radius = 2.5;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;

float dist_vect(vect* v1, vect* v2){
  return sqrt(pow(v1->x - v2->x, 2) + pow(v1->y - v2->y, 2));
  }

vect compute_force(vect* goal, float attr_gain, float rep_gain){
  /* Computes attractive and repulsive forces to reach the goal according to
  * the theory of potentials. */

  float distance_from_obs = 0.0;
  vect repulsive_force = {0.0, 0.0};
  vect resulting_force = {0.0, 0.0};
  vect distance_from_goal = {0.0, 0.0};
  float vect_amplitude = 0.0;
  int i = 0;

  distance_from_goal.x = goal->x - robot_state.position.x;
  distance_from_goal.y = goal->y - robot_state.position.y;

  resulting_force.x = attr_gain * distance_from_goal.x;
  resulting_force.y = attr_gain * distance_from_goal.y;
      
  for(i = 0; i < N_OBSTACLES; i++)
  {
      distance_from_obs = dist_vect(&obstacles[i], &robot_state.position);
      
      if(distance_from_obs < turtle_radius)
      {
	  vect_amplitude = (rep_gain / pow(distance_from_obs, 2)) * (1/distance_from_obs - 1/turtle_radius);
	  repulsive_force.x = vect_amplitude * (robot_state.position.x - obstacles[i].x);
	  repulsive_force.y = vect_amplitude * (robot_state.position.y - obstacles[i].y);
      }
      else
      {	  
	  repulsive_force.x = 0.0;
	  repulsive_force.y = 0.0;
      }
      resulting_force.x += repulsive_force.x;
      resulting_force.y += repulsive_force.y;
  }

  //ROS_INFO("Resulting Force: (%.5f, %.5f)", resulting_force.x, resulting_force.y);
  return resulting_force;
    
}

void reorient_robot(vect* force, float gain){
  /* Corrects robot orientation with proportional feedback control.*/
  
  geometry_msgs::Twist cmd;
  ros::Rate moveRate(500);
  
  float angle = atan2(force->y, force->x) - robot_state.theta;
  
  do{
    angle = atan2(force->y, force->x) - robot_state.theta;
    cmd.angular.z = gain * angle;
    vel_pub.publish(cmd);

    ros::spinOnce();
    moveRate.sleep();    
  }while(fabs(angle) > 0.02);

  cmd.angular.z = 0.0;
  cmd.linear.x = 0.0;
  vel_pub.publish(cmd);
  
}

void move_to_goal(vect* goal, float lin_gain, float ang_gain){
  /* Moves the robot towards the goal first orienting it then sending a linear velocity command. */

  geometry_msgs::Twist cmd;
  ros::Rate moveRate(50);

  vect force;
  float distance;
    
  ROS_INFO("Moving towards goal");
  do{
    
    force = compute_force(goal, 1.5, 1.5);
    
    reorient_robot(&force, ang_gain);

    cmd.linear.x = lin_gain * sqrt(pow(force.x, 2) + pow(force.y, 2));
    vel_pub.publish(cmd);

    distance = dist_vect(goal, &robot_state.position);
    
    ros::spinOnce();
    moveRate.sleep(); 
    
  }while(distance > 0.2);

  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  vel_pub.publish(cmd);    
}

void get_pose(const turtlesim::Pose::ConstPtr& pose_msg, int& topic_id){
  /* Callback function that stores pose of obstacle or robot. */
        
  if (topic_id > 0)
  {
    ROS_INFO("Found obstacle in position (%.1f, %.1f)", pose_msg->x, pose_msg->y);
    obstacles[topic_id - 1].x = pose_msg->x;
    obstacles[topic_id - 1].y = pose_msg->y;
    obs_pose_sent++;
  }
  else
  {
    robot_state.position.x = pose_msg->x;
    robot_state.position.y = pose_msg->y;
    robot_state.theta = pose_msg->theta;
  }
}

void get_obstacles_positions(ros::NodeHandle* node){
    /* Creates subscribers for each obstacle and waits until the positions are sent. */
    
    ros::Subscriber obs1_sub;
    ros::Subscriber obs2_sub;
    ros::Rate waitRate(50);

    obs1_sub = node->subscribe<turtlesim::Pose>("/obstacle1/pose", 10, boost::bind(get_pose, _1, 1));
    obs2_sub = node->subscribe<turtlesim::Pose>("/obstacle2/pose", 10, boost::bind(get_pose, _1, 2));
    
    // Wait for subscribers
    sleep(2);
    
    while(obs_pose_sent < N_OBSTACLES)
    {        
        ros::spinOnce();
        waitRate.sleep();        
    }   
    obs_pose_sent = 0;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh("~");
    ros::Rate mainRate(50);

    vect goal = {0.0, 0.0};

    nh.getParam("goal_x", goal.x);
    nh.getParam("goal_y", goal.y);
    
    vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10,  boost::bind(get_pose, _1, 0));
       
    get_obstacles_positions(&nh);

    move_to_goal(&goal, 0.7, 1.5);
    
    ROS_INFO("turtle1 survived!");
    
    ros::spin();
    return 0;

}
