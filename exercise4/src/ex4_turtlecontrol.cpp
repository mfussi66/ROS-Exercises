#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "exercise4/RobotStatus.h"

// Structure definition for robot info
struct status
{
  std::string name;
  bool ready;
  bool arrived;
  float x;
  float y;
  float theta;
};

// Global variables definitions
ros::Subscriber team_status_sub;
ros::Publisher team_status_pub;
ros::Publisher vel_pub;
ros::Subscriber pose_sub;

geometry_msgs::Twist cmd;

std::map<std::string, status> robots_state;

bool team_ready = false;
bool team_arrived = false;
int robot_id = 0;
const int robots_number = 4;
float baricenter[2] = {0, 0};
std::string robot_name = "turtle1";

// Functions definitions
void pub_robot_status(float pos_x, float pos_y, bool is_ready, bool is_arrived){
 /* publishes robot status in custom topic. Updates pose, ready to move status 
  * and goal reached status */
  
    exercise4::RobotStatus status_msg;

    status_msg.header.stamp = ros::Time::now();
    status_msg.id = robot_id;
    status_msg.name = robot_name;
    status_msg.is_ready = is_ready;
    status_msg.is_arrived = is_arrived;
    status_msg.pose_x = pos_x;
    status_msg.pose_y = pos_y;

    sleep(1.0);

    team_status_pub.publish(status_msg);
}

void wait_for_team(float pos_x, float pos_y){
// Wait until all robots are ready to move
  
  ros::Rate waitRate(2);
  
  while (!team_ready)
  {
    ROS_INFO_STREAM(robot_name << " waiting for team");
    pub_robot_status(pos_x, pos_y, true, false);
    ros::spinOnce();
    waitRate.sleep();
  }
  
  return;
  
}

void check_previous_robot(){
 /* checks if robot with previous number in the name has reached the goal.
  * if the robot is called "turtle1" it is assumed that it's the first one */
 
  ros::Rate waitRate(2);
  std::string prev_robot_name = "turtle" + std::to_string(robot_id);
  
  if (robot_name.compare("turtle1") == 0)
    return;
      
  while(ros::ok())
  {

    if (robots_state[prev_robot_name].arrived == true)
    {
      return;
    }
    
    ros::spinOnce();
    waitRate.sleep();
    
  }
  
}

void compute_goal(){
 /* Computes point in turtlesim space to reach. In this case
  * it is the baricenter of starting positions. */
 
    status temp;
    
    for(auto rbt : robots_state)
    {
        temp = rbt.second;
        baricenter[0] = baricenter[0] + temp.x;
        baricenter[1] = baricenter[1] + temp.y;
    }

    baricenter[0] = baricenter[0]/robots_number;
    baricenter[1] = baricenter[1]/robots_number;

}

void team_status_callback(const exercise4::RobotStatus::ConstPtr status_msg){
  /* Callback that updates the map containing all the info of robots coming from
   * the custom message topic. Communicates on screen when all robots are ready
   to move and have reached the goal */
     
    int ready_counter = 0;
    int arrived_counter = 0;
    status temp;

    if(team_ready && team_arrived) return;

    robots_state[status_msg->name].ready = status_msg->is_ready;
    robots_state[status_msg->name].arrived = status_msg->is_arrived;
    robots_state[status_msg->name].name = status_msg->name;
    robots_state[status_msg->name].x = status_msg->pose_x;
    robots_state[status_msg->name].y = status_msg->pose_y;
    
    for (auto rbt : robots_state)
    {
        temp = rbt.second;
        if (temp.ready) ready_counter++;
	if (temp.arrived) arrived_counter++;
    }
        
    if(ready_counter==robots_number && team_ready == false)
    {
        ROS_INFO_STREAM(robot_name << ": Team is ready!");
        team_ready = true;
    }
    
    if(arrived_counter == robots_number)
    {
        ROS_INFO_STREAM(robot_name << ": Team has reached the goal!");
        team_arrived = true;
    }
        
}

void get_pose(const turtlesim::Pose::ConstPtr pose_msg){
  /* Callback that saves the position and orientation of the robot */
  
    robots_state[robot_name].x = pose_msg->x;
    robots_state[robot_name].y = pose_msg->y;
    robots_state[robot_name].theta = pose_msg->theta;
    
}

void move_to_goal(float gain_dist, float gain_angle, float tol_dist, float tol_angle){
  /* Sends angular and linear speed commands to reach the goal with a P feedback
   * control for polar coordinates. Uses tolerances to stabilize the control. */
  
    ros::Rate moveRate(10);
   
    float distance = sqrt(pow(baricenter[0] - robots_state[robot_name].x, 2) + pow(baricenter[1] - robots_state[robot_name].y, 2));  
    float angle = atan2(baricenter[1] - robots_state[robot_name].y, baricenter[0] - robots_state[robot_name].x) - robots_state[robot_name].theta;

    ROS_INFO_STREAM(robot_name << ": moving of (distance, angle): ("<< distance << ", "<< angle << ")");
    
    while(fabs(angle) > tol_angle){
        angle = atan2(baricenter[1] - robots_state[robot_name].y, baricenter[0] - robots_state[robot_name].x) - robots_state[robot_name].theta;
        
        cmd.angular.z = gain_angle * angle;
        cmd.linear.x = 0;
        vel_pub.publish(cmd);

        ros::spinOnce();
        moveRate.sleep();    
    }
    
    while(fabs(distance) > tol_dist){
        distance = sqrt(pow(baricenter[0] - robots_state[robot_name].x, 2) + pow(baricenter[1] - robots_state[robot_name].y, 2));  
        
        cmd.angular.z = 0;
        cmd.linear.x = gain_dist * distance;
        vel_pub.publish(cmd);

        ros::spinOnce();
        moveRate.sleep();    
    }
    
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0; 
    vel_pub.publish(cmd);

}

int main(int argc, char **argv){
  
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh("~");
    ros::Rate mainRate(100);
    
    nh.getParam("robot_name", robot_name);  
        
    robot_id = (robot_name.back() - '0') - 1;
    
    //Publish and subscribe to team status messages
    team_status_pub = nh.advertise<exercise4::RobotStatus>("/team_status", 10);
    team_status_sub = nh.subscribe("/team_status", 20, &team_status_callback);
    
    //Publish and subscribe to topic to send commands and receive position
    vel_pub = nh.advertise<geometry_msgs::Twist>("/" + robot_name + "/cmd_vel",10);
    pose_sub = nh.subscribe("/" + robot_name + "/pose",10, &get_pose);
    
    // Wait for subscriber for all robots
    sleep(3);
    ros::spinOnce();
    
    sleep(1);
    
    pub_robot_status(robots_state[robot_name].x, robots_state[robot_name].y, true, false);
    
    wait_for_team(robots_state[robot_name].x, robots_state[robot_name].y);     
    
    compute_goal();
    
    check_previous_robot();
    
    move_to_goal(0.7, 1.2, 0.1, 0.02);
    
    pub_robot_status(robots_state[robot_name].x, robots_state[robot_name].y, true, true); 
   
    ROS_INFO_STREAM(robot_name << ": Reached the goal!");

    ros::spin();
        
    return 0;
    
}
