#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

struct vect{
  float x;
  float y;
};

struct status{
    vect position;
    float theta;
};

status robot_state;
vect obstacles[2];
int obs_pose_sent = 0;
int num_of_obstacles = 2;

float turtle_radius = 1;

ros::Publisher vel_pub;
ros::Subscriber pose_sub;

float norm_vect(vect* v1, vect* v2){
  
  return sqrt(pow(v1->x - v2->x, 2) + pow(v1->y - v2->y, 2));
  
}

vect compute_force(vect* goal, float attr_gain, float rep_gain){
  
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
    
    ROS_INFO("Attractive Force: (%f,%f)", resulting_force.x, resulting_force.y);
    
    for(i = 0; i < num_of_obstacles; i++)
    {
        distance_from_obs = norm_vect(&obstacles[i], &robot_state.position);
        
        if(distance_from_obs < turtle_radius)
        {
            vect_amplitude = (rep_gain / pow(distance_from_obs, 2)) * (1/distance_from_obs - 1/turtle_radius);
            repulsive_force.x = vect_amplitude * (robot_state.position.x - obstacles[i].x);
            repulsive_force.y = vect_amplitude * (robot_state.position.y - obstacles[i].y);
            ROS_INFO("Repulsive amplitude %i: %f",i + 1, vect_amplitude);
        }
        else
        {
            repulsive_force.x = 0.0;
            repulsive_force.y = 0.0;
        }
        resulting_force.x += repulsive_force.x;
        resulting_force.y += repulsive_force.y;
        
        ROS_INFO("Repulsive Force %i: (%f,%f)", i, repulsive_force.x, repulsive_force.y);
    }
  
    //ROS_INFO("Force: (%.5f, %.5f)", resulting_force.x, resulting_force.y);
    return resulting_force;
    
}

void reorient_robot(vect* force, float gain){
  
  geometry_msgs::Twist cmd;
  ros::Rate moveRate(100);
  
  float angle = atan2(force->y, force->x) - robot_state.theta;
  
  //ROS_INFO("Correcting orientation");
  do{
    angle = atan2(force->y, force->x) - robot_state.theta;
    cmd.angular.z = gain * angle;
    //cmd.linear.x = 0.0;
    vel_pub.publish(cmd);

    ros::spinOnce();
    moveRate.sleep();    
  }while(fabs(angle) > 0.02);

cmd.angular.z = 0.0;
cmd.linear.x = 0.0;
vel_pub.publish(cmd);
  
}

void move_to_goal(vect* goal, float lin_gain, float ang_gain){

  geometry_msgs::Twist cmd;
  ros::Rate moveRate(50);

  vect force;
  float distance;
    
  ROS_INFO("Moving towards goal");
  do{
    
    force = compute_force(goal, 2, 0.8);
    
    reorient_robot(&force, ang_gain);

    //cmd.angular.z = 0.0;
    cmd.linear.x = lin_gain * sqrt(pow(force.x, 2) + pow(force.y, 2));
    vel_pub.publish(cmd);

    distance = norm_vect(goal, &robot_state.position);
    
    ros::spinOnce();
    moveRate.sleep(); 
    
  }while(distance > 0.2);

  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;
  vel_pub.publish(cmd);
    
}

void get_pose(const turtlesim::Pose::ConstPtr& pose_msg, int& topic_id){
        
  if (topic_id > 0)
  {
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
    
    ros::Subscriber obs1_sub;
    ros::Subscriber obs2_sub;
    ros::Rate waitRate(50);

    obs1_sub = node->subscribe<turtlesim::Pose>("/obstacle1/pose", 10, boost::bind(get_pose, _1, 1));
    obs2_sub = node->subscribe<turtlesim::Pose>("/obstacle2/pose", 10, boost::bind(get_pose, _1, 2));

    sleep(2);
    ros::spinOnce();
    
    while(obs_pose_sent < num_of_obstacles){        
        ros::spinOnce();
        waitRate.sleep();        
    }
    
    obs_pose_sent = 0;
    
}


int main(int argc, char **argv){

    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh("~");
    ros::Rate mainRate(50);

    int i;
    vect goal;
    goal.x = 0;
    goal.y = 0;
    
    nh.getParam("goal_x", goal.x);
    nh.getParam("goal_y", goal.y);
    
    vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    pose_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10,  boost::bind(get_pose, _1, 0));
       
    get_obstacles_positions(&nh);

    for(i = 0; i < num_of_obstacles; i++)
      ROS_INFO("Found obstacle in position (%.2f, %.2f)", obstacles[i].x, obstacles[i].y);
    
    move_to_goal(&goal, 0.7, 1.5);
    
    ROS_INFO("turtle1 survived!");
    
    ros::spin();
    return 0;

}
